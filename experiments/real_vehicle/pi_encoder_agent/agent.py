#!/usr/bin/env python3
"""Standalone Raspberry Pi encoder UDP agent."""

import argparse
import os
import socket
import sys
import time
import uuid

try:
    from .protocol import encode_sample, make_sample
    from .quadrature import QuadratureDecoder
except ImportError:  # Direct execution on the Raspberry Pi.
    from protocol import encode_sample, make_sample
    from quadrature import QuadratureDecoder


GPIO_A_DEFAULT = 22
GPIO_B_DEFAULT = 27
UDP_PORT_DEFAULT = 5011


class PigpioEncoderSource:
    """Read both encoder channels using pigpio edge callbacks."""

    def __init__(self, gpio_a, gpio_b, direction_sign):
        try:
            import pigpio
        except ImportError as exc:
            raise RuntimeError("pigpio Python module is not installed") from exc

        self._pigpio = pigpio
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.decoder = QuadratureDecoder(direction_sign=direction_sign)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("cannot connect to pigpiod")

        self.pi.set_mode(self.gpio_a, pigpio.INPUT)
        self.pi.set_mode(self.gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(self.gpio_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.gpio_b, pigpio.PUD_UP)
        self._prime_state()
        self._callbacks = [
            self.pi.callback(self.gpio_a, pigpio.EITHER_EDGE, self._on_edge),
            self.pi.callback(self.gpio_b, pigpio.EITHER_EDGE, self._on_edge),
        ]

    def _prime_state(self):
        self.decoder.update(
            int(self.pi.read(self.gpio_a)),
            int(self.pi.read(self.gpio_b)),
            time.monotonic_ns(),
        )

    def _on_edge(self, _gpio, level, _tick):
        # pigpio level 2 means watchdog timeout. EITHER_EDGE normally gives 0/1.
        if level not in (0, 1):
            return
        self.decoder.update(
            int(self.pi.read(self.gpio_a)),
            int(self.pi.read(self.gpio_b)),
            time.monotonic_ns(),
        )

    def poll(self, _now_ns):
        pass

    def snapshot(self, now_ns):
        return self.decoder.snapshot(now_ns)

    def close(self):
        if getattr(self, "_closed", False):
            return
        for callback in getattr(self, "_callbacks", []):
            callback.cancel()
        if getattr(self, "pi", None) is not None:
            self.pi.stop()
        self._closed = True


class SyntheticEncoderSource:
    """Generate deterministic quadrature transitions without GPIO or pigpio."""

    _STATES = ((0, 0), (0, 1), (1, 1), (1, 0))

    def __init__(self, direction_sign, counts_per_second):
        if counts_per_second <= 0.0:
            raise ValueError("counts_per_second must be positive")
        self.decoder = QuadratureDecoder(direction_sign=direction_sign)
        self.counts_per_second = float(counts_per_second)
        self._state_index = 0
        self._last_poll_ns = None
        self._remainder = 0.0
        self.decoder.update(0, 0, time.monotonic_ns())

    def poll(self, now_ns):
        if self._last_poll_ns is None:
            self._last_poll_ns = now_ns
            return
        elapsed_s = max(0.0, (now_ns - self._last_poll_ns) / 1_000_000_000.0)
        self._last_poll_ns = now_ns
        self._remainder += elapsed_s * self.counts_per_second
        transitions = int(self._remainder)
        self._remainder -= transitions
        for _ in range(transitions):
            self._state_index = (self._state_index + 1) % len(self._STATES)
            gpio_a, gpio_b = self._STATES[self._state_index]
            self.decoder.update(gpio_a, gpio_b, now_ns)

    def snapshot(self, now_ns):
        return self.decoder.snapshot(now_ns)

    def close(self):
        pass


class EncoderAgent:
    def __init__(self, source, host, port, rate_hz, session_id=None):
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.source = source
        self.destination = (host, int(port))
        self.rate_hz = float(rate_hz)
        self.session_id = session_id or self._make_session_id()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.last_send_ns = None
        self.last_count = 0
        self._closed = False

    @staticmethod
    def _make_session_id():
        return f"{socket.gethostname()}-{os.getpid()}-{uuid.uuid4().hex[:8]}"

    def send_once(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        self.source.poll(now_ns)
        state = self.source.snapshot(now_ns)
        if self.last_send_ns is None:
            sample_period_s = 0.0
        else:
            sample_period_s = max(
                0.0,
                (now_ns - self.last_send_ns) / 1_000_000_000.0,
            )
        cumulative_count = state["cumulative_count"]
        delta_count = cumulative_count - self.last_count
        direction = 1 if delta_count > 0 else -1 if delta_count < 0 else 0
        sample = make_sample(
            session_id=self.session_id,
            sequence=self.sequence,
            source_monotonic_ns=now_ns,
            cumulative_count=cumulative_count,
            delta_count=delta_count,
            direction=direction,
            sample_period_s=sample_period_s,
            pulse_age_s=state["pulse_age_s"],
            invalid_transition_count=state["invalid_transition_count"],
            gpio_a=state["gpio_a"],
            gpio_b=state["gpio_b"],
            encoder_healthy=True,
        )
        self.socket.sendto(encode_sample(sample), self.destination)
        self.last_send_ns = now_ns
        self.last_count = cumulative_count
        self.sequence += 1
        return sample

    def run(self, max_packets=0):
        period_s = 1.0 / self.rate_hz
        next_send = time.monotonic()
        sent = 0
        try:
            while max_packets <= 0 or sent < max_packets:
                now_ns = time.monotonic_ns()
                self.send_once(now_ns)
                sent += 1
                next_send += period_s
                time.sleep(max(0.0, next_send - time.monotonic()))
        finally:
            self.close()

    def close(self):
        if self._closed:
            return
        try:
            self.source.close()
        finally:
            self.socket.close()
            self._closed = True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("PC_ENCODER_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("PC_ENCODER_PORT", UDP_PORT_DEFAULT)))
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--gpio-a", type=int, default=int(os.environ.get("ENCODER_GPIO_A", GPIO_A_DEFAULT)))
    parser.add_argument("--gpio-b", type=int, default=int(os.environ.get("ENCODER_GPIO_B", GPIO_B_DEFAULT)))
    # 実機試験で前進がプラスになることを確認済み。
    parser.add_argument("--direction-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--session-id", default="")
    parser.add_argument("--max-packets", type=int, default=0)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--synthetic-counts-per-second", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    source = None
    agent = None
    try:
        if args.synthetic:
            source = SyntheticEncoderSource(
                direction_sign=args.direction_sign,
                counts_per_second=args.synthetic_counts_per_second,
            )
            mode = "synthetic"
        else:
            source = PigpioEncoderSource(
                gpio_a=args.gpio_a,
                gpio_b=args.gpio_b,
                direction_sign=args.direction_sign,
            )
            mode = "pigpio"
        agent = EncoderAgent(
            source=source,
            host=args.host,
            port=args.port,
            rate_hz=args.rate_hz,
            session_id=args.session_id or None,
        )
        print(
            f"encoder mode={mode} gpio_a={args.gpio_a} gpio_b={args.gpio_b} "
            f"destination={args.host}:{args.port} rate_hz={args.rate_hz:g} "
            f"session={agent.session_id}",
            flush=True,
        )
        agent.run(max_packets=args.max_packets)
    except KeyboardInterrupt:
        print("Encoder agent stopped cleanly", flush=True)
    finally:
        if agent is None and source is not None:
            source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
