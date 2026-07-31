"""Thread-safe quadrature decoder for a two-channel wheel encoder."""

import threading


# 00 -> 01 -> 11 -> 10 -> 00 is one logical positive cycle.
_TRANSITIONS = {
    (0, 1): 1,
    (1, 3): 1,
    (3, 2): 1,
    (2, 0): 1,
    (1, 0): -1,
    (3, 1): -1,
    (2, 3): -1,
    (0, 2): -1,
}


class QuadratureDecoder:
    """Decode every valid A/B state transition into signed counts."""

    def __init__(self, direction_sign=1):
        if direction_sign not in (-1, 1):
            raise ValueError("direction_sign must be 1 or -1")
        self.direction_sign = direction_sign
        self._lock = threading.Lock()
        self._previous_state = None
        self._gpio_a = 0
        self._gpio_b = 0
        self._cumulative_count = 0
        self._invalid_transition_count = 0
        self._last_direction = 0
        self._first_state_ns = None
        self._last_transition_ns = None

    @staticmethod
    def _state(gpio_a, gpio_b):
        if gpio_a not in (0, 1) or gpio_b not in (0, 1):
            raise ValueError("GPIO levels must be 0 or 1")
        return (gpio_a << 1) | gpio_b

    def update(self, gpio_a, gpio_b, timestamp_ns):
        """Apply a new GPIO state and return -1, 0, or +1."""

        state = self._state(gpio_a, gpio_b)
        if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")

        with self._lock:
            self._gpio_a = gpio_a
            self._gpio_b = gpio_b
            if self._first_state_ns is None:
                self._first_state_ns = timestamp_ns
            if self._previous_state is None:
                self._previous_state = state
                return 0
            if state == self._previous_state:
                return 0

            logical_delta = _TRANSITIONS.get((self._previous_state, state))
            self._previous_state = state
            if logical_delta is None:
                self._invalid_transition_count += 1
                return 0

            delta = self.direction_sign * logical_delta
            self._cumulative_count += delta
            self._last_direction = 1 if delta > 0 else -1
            self._last_transition_ns = timestamp_ns
            return delta

    def snapshot(self, now_ns):
        """Return a consistent state snapshot for a UDP sample."""

        if not isinstance(now_ns, int) or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        with self._lock:
            if self._last_transition_ns is None:
                pulse_age_s = 0.0
                if self._first_state_ns is not None:
                    pulse_age_s = max(
                        0.0,
                        (now_ns - self._first_state_ns) / 1_000_000_000.0,
                    )
            else:
                pulse_age_s = max(
                    0.0,
                    (now_ns - self._last_transition_ns) / 1_000_000_000.0,
                )
            return {
                "cumulative_count": self._cumulative_count,
                "invalid_transition_count": self._invalid_transition_count,
                "gpio_a": self._gpio_a,
                "gpio_b": self._gpio_b,
                "last_direction": self._last_direction,
                "pulse_age_s": pulse_age_s,
            }
