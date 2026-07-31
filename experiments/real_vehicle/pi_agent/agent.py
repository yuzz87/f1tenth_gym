#!/usr/bin/env python3
"""Standalone safe UDP actuator endpoint for Raspberry Pi OS."""

import argparse
import signal
import socket
import sys
import time

try:
    from .policy import AgentLimits, SafeCommandPolicy
    from .protocol import encode_status, make_status
except ImportError:
    from policy import AgentLimits, SafeCommandPolicy
    from protocol import encode_status, make_status


ESC_GPIO_BCM = 12
STEERING_GPIO_BCM = 13
PWM_FREQUENCY_HZ = 70
DEFAULT_PORT = 5005
DEFAULT_WATCHDOG_TIMEOUT_S = 0.20


def duty_to_pigpio(duty_percent):
    return int(round(float(duty_percent) * 10000.0))


class DryRunBackend:
    hardware_output_enabled = False

    def __init__(self):
        self.last_output = None

    def apply(self, esc_duty_percent, steering_duty_percent, reason):
        self.last_output = (
            float(esc_duty_percent),
            float(steering_duty_percent),
            str(reason),
        )

    def close(self):
        return None


class PigpioHardwareBackend:
    hardware_output_enabled = True

    def __init__(self, limits):
        import pigpio

        self._pigpio = pigpio
        self._limits = limits
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("cannot connect to pigpiod")
        self._pi.set_mode(ESC_GPIO_BCM, pigpio.OUTPUT)
        self._pi.set_mode(STEERING_GPIO_BCM, pigpio.OUTPUT)
        self.apply(
            limits.esc_neutral_duty_percent,
            limits.steering_neutral_duty_percent,
            "backend_startup",
        )

    def apply(self, esc_duty_percent, steering_duty_percent, _reason):
        esc_result = self._pi.hardware_PWM(
            ESC_GPIO_BCM,
            PWM_FREQUENCY_HZ,
            duty_to_pigpio(esc_duty_percent),
        )
        steering_result = self._pi.hardware_PWM(
            STEERING_GPIO_BCM,
            PWM_FREQUENCY_HZ,
            duty_to_pigpio(steering_duty_percent),
        )
        if esc_result != 0 or steering_result != 0:
            raise RuntimeError(
                "hardware_PWM failed: "
                f"esc={esc_result}, steering={steering_result}"
            )

    def close(self):
        try:
            self.apply(
                self._limits.esc_neutral_duty_percent,
                self._limits.steering_neutral_duty_percent,
                "backend_close",
            )
            time.sleep(0.2)
        finally:
            self._pi.stop()


class UdpActuatorAgent:
    def __init__(self, args, backend, wall_clock=time.time, monotonic=time.monotonic):
        self.args = args
        self.backend = backend
        self.wall_clock = wall_clock
        self.monotonic = monotonic
        self.limits = AgentLimits(
            esc_neutral_duty_percent=args.esc_neutral_duty,
            esc_start_duty_percent=args.esc_start_duty,
            esc_forward_min_duty_percent=args.esc_forward_min_duty,
            steering_neutral_duty_percent=args.steering_neutral_duty,
            steering_min_duty_percent=args.steering_min_duty,
            steering_max_duty_percent=args.steering_max_duty,
            message_max_age_s=args.message_max_age,
            future_tolerance_s=args.future_tolerance,
        )
        self.policy = SafeCommandPolicy(self.limits)
        self.accepted_packets = 0
        self.rejected_packets = 0
        self.watchdog_count = 0
        self.last_packet_monotonic = None
        self.last_sender = None
        self.last_decision = self.policy.neutral("startup")
        self.watchdog_active = False
        self.running = True
        self._last_log_key = None

    @property
    def output_armed(self):
        return bool(self.args.arm_output)

    def _effective_decision(self, decision):
        if not decision.accepted:
            return self.policy.neutral(
                decision.reason,
                decision.session_id,
                decision.sequence_id,
            )
        if not decision.enable_output:
            return decision
        if not self.output_armed:
            return self.policy.neutral(
                "output_disarmed",
                decision.session_id,
                decision.sequence_id,
                accepted=True,
            )
        drive_requested = decision.esc_duty_percent < (
            self.limits.esc_neutral_duty_percent - 1e-6
        )
        if drive_requested and not self.args.allow_motion:
            return self.policy.neutral(
                "motion_disabled",
                decision.session_id,
                decision.sequence_id,
                accepted=True,
            )
        return decision

    def process_packet(self, packet, sender):
        decision = self.policy.process(packet, sender[0], self.wall_clock())
        if decision.accepted:
            self.accepted_packets += 1
            self.last_packet_monotonic = self.monotonic()
            self.last_sender = sender
            self.watchdog_active = False
        else:
            self.rejected_packets += 1
        decision = self._effective_decision(decision)
        self.apply(decision)
        return decision

    def apply(self, decision):
        try:
            self.backend.apply(
                decision.esc_duty_percent,
                decision.steering_duty_percent,
                decision.reason,
            )
            self.last_decision = decision
        except Exception as exc:
            neutral = self.policy.neutral(f"backend_error:{exc}")
            self.last_decision = neutral
            try:
                self.backend.apply(
                    neutral.esc_duty_percent,
                    neutral.steering_duty_percent,
                    neutral.reason,
                )
            finally:
                raise

    def check_watchdog(self):
        if self.last_packet_monotonic is None:
            return False
        age_s = self.monotonic() - self.last_packet_monotonic
        if age_s <= self.args.watchdog_timeout or self.watchdog_active:
            return False
        self.watchdog_active = True
        self.watchdog_count += 1
        self.policy.release_session()
        self.apply(self.policy.neutral("command_timeout"))
        return True

    def request_stop(self, _signum=None, _frame=None):
        self.running = False

    def status_packet(self):
        decision = self.last_decision
        state = "running" if decision.enable_output else "stopped"
        if (
            decision.enable_output
            and decision.esc_duty_percent
            >= self.limits.esc_neutral_duty_percent - 1e-6
        ):
            state = "steering"
        if decision.enable_output and not self.backend.hardware_output_enabled:
            state = "dry_run"
        if not decision.accepted and decision.reason != "startup":
            state = "fault"
        return encode_status(make_status(
            session_id=decision.session_id,
            sequence_id=decision.sequence_id,
            sent_at_unix_s=self.wall_clock(),
            state=state,
            reason=decision.reason,
            esc_duty_percent=decision.esc_duty_percent,
            steering_duty_percent=decision.steering_duty_percent,
            hardware_output_enabled=self.backend.hardware_output_enabled,
            output_armed=self.output_armed,
            accepted_packets=self.accepted_packets,
            rejected_packets=self.rejected_packets,
            watchdog_count=self.watchdog_count,
        ))

    def run(self):
        self.apply(self.policy.neutral("startup"))
        next_status_at = self.monotonic()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.args.host, self.args.port))
            sock.settimeout(min(self.args.watchdog_timeout / 4.0, 0.02))
            print(
                "listening "
                f"host={self.args.host} port={self.args.port} "
                f"mode={'hardware' if self.backend.hardware_output_enabled else 'dry-run'} "
                f"armed={self.output_armed} allow_motion={self.args.allow_motion} "
                f"watchdog_timeout_s={self.args.watchdog_timeout:.3f}",
                flush=True,
            )
            try:
                while self.running:
                    try:
                        packet, sender = sock.recvfrom(2048)
                        decision = self.process_packet(packet, sender)
                        sock.sendto(self.status_packet(), sender)
                        log_key = (
                            decision.enable_output,
                            decision.reason,
                        )
                        if decision.reason != "safe_stop" and log_key != self._last_log_key:
                            print(
                                f"sender={sender[0]} seq={decision.sequence_id} "
                                f"esc={decision.esc_duty_percent:.3f}% "
                                f"steering={decision.steering_duty_percent:.3f}% "
                                f"enabled={decision.enable_output} "
                                f"reason={decision.reason}",
                                flush=True,
                            )
                        self._last_log_key = log_key
                    except socket.timeout:
                        pass
                    if self.check_watchdog():
                        print("watchdog: command_timeout -> neutral", flush=True)
                    now = self.monotonic()
                    if self.last_sender is not None and now >= next_status_at:
                        sock.sendto(self.status_packet(), self.last_sender)
                        next_status_at = now + 1.0 / self.args.status_rate_hz
            except KeyboardInterrupt:
                print("stopped by Ctrl-C", flush=True)
            finally:
                self.apply(self.policy.neutral("agent_shutdown"))
                print("shutdown: agent_shutdown -> neutral", flush=True)


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=DEFAULT_WATCHDOG_TIMEOUT_S,
    )
    parser.add_argument("--message-max-age", type=float, default=0.10)
    parser.add_argument("--future-tolerance", type=float, default=0.10)
    parser.add_argument("--status-rate-hz", type=float, default=10.0)
    parser.add_argument("--esc-neutral-duty", type=float, default=10.30)
    parser.add_argument("--esc-start-duty", type=float, default=10.16)
    parser.add_argument("--esc-forward-min-duty", type=float, default=10.10)
    parser.add_argument("--steering-neutral-duty", type=float, default=10.895)
    parser.add_argument("--steering-min-duty", type=float, default=9.05)
    parser.add_argument("--steering-max-duty", type=float, default=12.42)
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--arm-output", action="store_true")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--confirm-wheels-lifted", action="store_true")
    parser.add_argument("--confirm-track-clear", action="store_true")
    parser.add_argument("--confirm-power-cutoff", action="store_true")
    return parser


def validate_args(args, parser):
    if not 0.0 < args.watchdog_timeout <= 1.0:
        parser.error("--watchdog-timeout must be in (0, 1.0]")
    if args.status_rate_hz <= 0.0:
        parser.error("--status-rate-hz must be positive")
    if args.message_max_age <= 0.0:
        parser.error("--message-max-age must be positive")
    if args.future_tolerance < 0.0:
        parser.error("--future-tolerance must be non-negative")
    if not (
        args.esc_forward_min_duty
        <= args.esc_start_duty
        < args.esc_neutral_duty
    ):
        parser.error("ESC limits must satisfy forward_min <= start < neutral")
    if not args.steering_min_duty < args.steering_neutral_duty < args.steering_max_duty:
        parser.error("steering limits must contain the neutral duty")
    if args.enable_hardware:
        safe_test_area_confirmed = (
            args.confirm_wheels_lifted or args.confirm_track_clear
        )
        if not safe_test_area_confirmed or not args.confirm_power_cutoff:
            parser.error(
                "hardware mode requires --confirm-wheels-lifted or "
                "--confirm-track-clear, plus "
                "--confirm-power-cutoff"
            )
        if args.allow_motion and not args.arm_output:
            parser.error("hardware motion requires --arm-output")


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)
    limits = AgentLimits(
        esc_neutral_duty_percent=args.esc_neutral_duty,
        esc_start_duty_percent=args.esc_start_duty,
        esc_forward_min_duty_percent=args.esc_forward_min_duty,
        steering_neutral_duty_percent=args.steering_neutral_duty,
        steering_min_duty_percent=args.steering_min_duty,
        steering_max_duty_percent=args.steering_max_duty,
        message_max_age_s=args.message_max_age,
        future_tolerance_s=args.future_tolerance,
    )
    backend = PigpioHardwareBackend(limits) if args.enable_hardware else DryRunBackend()
    agent = UdpActuatorAgent(args, backend)
    signal.signal(signal.SIGINT, agent.request_stop)
    signal.signal(signal.SIGTERM, agent.request_stop)
    try:
        agent.run()
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
