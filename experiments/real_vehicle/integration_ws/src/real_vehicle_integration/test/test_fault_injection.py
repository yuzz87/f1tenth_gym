import pytest

from real_vehicle_integration.fault_injection import FaultInjectionQueue, FaultProfile


def test_delayed_messages_are_released_in_order():
    queue = FaultInjectionQueue(FaultProfile(delay_s=0.2, seed=3))
    assert queue.push("first", 1.0)
    assert queue.push("second", 1.1)
    assert queue.pop_ready(1.19) == []
    assert queue.pop_ready(1.20) == ["first"]
    assert queue.pop_ready(1.30) == ["second"]


def test_full_dropout_is_deterministic():
    queue = FaultInjectionQueue(FaultProfile(dropout_probability=1.0, seed=4))
    assert not queue.push("message", 0.0)
    statistics = queue.statistics()
    assert statistics["received"] == 1
    assert statistics["dropped"] == 1
    assert statistics["relayed"] == 0
    assert statistics["pending"] == 0


def test_pending_queue_has_a_hard_limit():
    queue = FaultInjectionQueue(FaultProfile(delay_s=1.0, max_pending=1))
    assert queue.push("first", 0.0)
    assert not queue.push("second", 0.0)
    assert queue.statistics()["overflow"] == 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"delay_s": -0.1},
        {"dropout_probability": -0.1},
        {"dropout_probability": 1.1},
        {"delay_jitter_s": -0.1},
        {"burst_start_probability": 1.1},
        {"burst_length_messages": 0},
        {"outage_duration_s": -0.1},
        {"max_pending": 0},
    ],
)
def test_invalid_profiles_are_rejected(kwargs):
    with pytest.raises(ValueError):
        FaultProfile(**kwargs)


def test_burst_dropout_drops_consecutive_messages():
    queue = FaultInjectionQueue(FaultProfile(
        burst_start_probability=1.0,
        burst_length_messages=3,
    ))
    assert not queue.push("one", 0.0)
    assert not queue.push("two", 0.1)
    assert not queue.push("three", 0.2)
    assert queue.statistics()["burst_drops"] == 3


def test_timed_outage_drops_only_inside_window():
    queue = FaultInjectionQueue(FaultProfile(
        outage_after_s=1.0,
        outage_duration_s=0.5,
    ))
    assert queue.push("before", 0.0)
    assert not queue.push("during", 1.2)
    assert queue.push("after", 1.6)
    assert queue.statistics()["outage_drops"] == 1


def test_duplicate_and_stale_replay_are_counted():
    queue = FaultInjectionQueue(FaultProfile(
        duplicate_probability=1.0,
        stale_replay_probability=1.0,
    ))
    queue.push("first", 0.0)
    queue.pop_ready(0.1)
    queue.push("second", 0.2)
    released = queue.pop_ready(0.3)
    assert released == ["first", "first"]
    assert queue.statistics()["duplicates"] == 2
    assert queue.statistics()["stale_replays"] == 1


def test_minimum_publish_period_throttles_release():
    queue = FaultInjectionQueue(FaultProfile(minimum_publish_period_s=0.1))
    queue.push("first", 0.0)
    queue.push("second", 0.0)
    assert queue.pop_ready(0.0) == ["first"]
    assert queue.pop_ready(0.05) == []
    assert queue.pop_ready(0.1) == ["second"]
