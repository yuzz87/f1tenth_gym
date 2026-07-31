"""Deterministic delayed/dropout queue used by ROS message relays."""

from dataclasses import dataclass
import heapq
import itertools

import numpy as np


@dataclass(frozen=True)
class FaultProfile:
    delay_s: float = 0.0
    delay_jitter_s: float = 0.0
    dropout_probability: float = 0.0
    burst_start_probability: float = 0.0
    burst_length_messages: int = 1
    outage_after_s: float = -1.0
    outage_duration_s: float = 0.0
    duplicate_probability: float = 0.0
    stale_replay_probability: float = 0.0
    minimum_publish_period_s: float = 0.0
    seed: int = 1
    max_pending: int = 1000

    def __post_init__(self):
        if self.delay_s < 0.0:
            raise ValueError("delay_s must be non-negative")
        if self.delay_jitter_s < 0.0:
            raise ValueError("delay_jitter_s must be non-negative")
        for name in (
            "dropout_probability",
            "burst_start_probability",
            "duplicate_probability",
            "stale_replay_probability",
        ):
            if not 0.0 <= getattr(self, name) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.burst_length_messages < 1:
            raise ValueError("burst_length_messages must be positive")
        if self.outage_duration_s < 0.0 or self.minimum_publish_period_s < 0.0:
            raise ValueError("outage duration and publish period must be non-negative")
        if self.max_pending < 1:
            raise ValueError("max_pending must be positive")


class FaultInjectionQueue:
    def __init__(self, profile=FaultProfile()):
        self.profile = profile
        self.rng = np.random.default_rng(profile.seed)
        self.pending = []
        self.counter = itertools.count()
        self.received_count = 0
        self.dropped_count = 0
        self.relayed_count = 0
        self.overflow_count = 0
        self.duplicate_count = 0
        self.stale_replay_count = 0
        self.reordered_count = 0
        self.outage_drop_count = 0
        self.burst_drop_count = 0
        self._burst_remaining = 0
        self._first_receive_s = None
        self._last_message = None
        self._last_release_s = -float("inf")

    def _is_outage(self, receive_time_s):
        if self._first_receive_s is None:
            self._first_receive_s = float(receive_time_s)
        if self.profile.outage_after_s < 0.0:
            return False
        elapsed = float(receive_time_s) - self._first_receive_s
        return (
            self.profile.outage_after_s
            <= elapsed
            < self.profile.outage_after_s + self.profile.outage_duration_s
        )

    def _schedule(self, message, receive_time_s, extra_delay_s=0.0):
        if len(self.pending) >= self.profile.max_pending:
            self.overflow_count += 1
            return False
        jitter = 0.0
        if self.profile.delay_jitter_s > 0.0:
            jitter = float(self.rng.normal(0.0, self.profile.delay_jitter_s))
        delay = max(self.profile.delay_s + jitter + extra_delay_s, 0.0)
        release_time = float(receive_time_s) + delay
        if self.pending and release_time < max(item[0] for item in self.pending):
            self.reordered_count += 1
        heapq.heappush(self.pending, (release_time, next(self.counter), message))
        return True

    def push(self, message, receive_time_s):
        self.received_count += 1
        if self._is_outage(receive_time_s):
            self.outage_drop_count += 1
            self.dropped_count += 1
            return False
        if self._burst_remaining <= 0:
            if self.rng.random() < self.profile.burst_start_probability:
                self._burst_remaining = self.profile.burst_length_messages
        if self._burst_remaining > 0:
            self._burst_remaining -= 1
            self.burst_drop_count += 1
            self.dropped_count += 1
            return False
        if self.rng.random() < self.profile.dropout_probability:
            self.dropped_count += 1
            return False
        scheduled_message = message
        if self._last_message is not None:
            if self.rng.random() < self.profile.stale_replay_probability:
                scheduled_message = self._last_message
                self.stale_replay_count += 1
        accepted = self._schedule(scheduled_message, receive_time_s)
        if accepted and self.rng.random() < self.profile.duplicate_probability:
            if self._schedule(scheduled_message, receive_time_s, 1e-6):
                self.duplicate_count += 1
        self._last_message = message
        return accepted

    def pop_ready(self, now_s):
        ready = []
        while self.pending and self.pending[0][0] <= float(now_s):
            if (
                float(now_s) - self._last_release_s
                < self.profile.minimum_publish_period_s
            ):
                break
            _release_time, _sequence, message = heapq.heappop(self.pending)
            ready.append(message)
            self.relayed_count += 1
            self._last_release_s = float(now_s)
            if self.profile.minimum_publish_period_s > 0.0:
                break
        return ready

    def statistics(self):
        return {
            "received": self.received_count,
            "dropped": self.dropped_count,
            "relayed": self.relayed_count,
            "overflow": self.overflow_count,
            "pending": len(self.pending),
            "duplicates": self.duplicate_count,
            "stale_replays": self.stale_replay_count,
            "reordered": self.reordered_count,
            "outage_drops": self.outage_drop_count,
            "burst_drops": self.burst_drop_count,
        }
