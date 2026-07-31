import pytest

from real_vehicle_integration.contracts import ControlCommand
from real_vehicle_integration.safety_monitor import SafetyMonitor
from real_vehicle_integration.state_machine import SafetyState


def _command(stamp, sequence=1):
    return ControlCommand(stamp, sequence, 0.2, 0.1, 0.2)


def _ready_monitor(now=1.0):
    monitor = SafetyMonitor()
    monitor.update_scan(now)
    monitor.update_pose(now)
    monitor.arm(now)
    monitor.start(now)
    return monitor


def test_running_monitor_passes_valid_command():
    monitor = _ready_monitor()
    assert monitor.process(_command(1.0), 1.0).speed_mps == 0.2


def test_disarmed_monitor_returns_neutral():
    monitor = SafetyMonitor()
    monitor.update_scan(1.0)
    monitor.update_pose(1.0)
    assert monitor.process(_command(1.0), 1.0).speed_mps == 0.0


def test_watchdog_faults_on_scan_timeout():
    monitor = _ready_monitor()
    monitor.process(_command(1.0), 1.0)
    assert monitor.watchdog(1.31) == "scan_timeout"
    assert monitor.machine.state == SafetyState.FAULT


def test_emergency_stop_forces_neutral():
    monitor = _ready_monitor()
    monitor.set_emergency_stop(True)
    assert monitor.process(_command(1.0), 1.0).speed_mps == 0.0
    assert monitor.machine.fault_reason == "emergency_stop"


def test_arm_requires_fresh_sensors():
    with pytest.raises(RuntimeError, match="scan_missing"):
        SafetyMonitor().arm(1.0)


def test_non_increasing_sequence_faults():
    monitor = _ready_monitor()
    monitor.process(_command(1.0, 2), 1.0)
    safe = monitor.process(_command(1.01, 2), 1.01)
    assert safe.speed_mps == 0.0
    assert monitor.machine.fault_reason == "non_increasing_sequence"


def test_replayed_command_timestamp_faults():
    monitor = _ready_monitor()
    monitor.process(_command(1.0, 1), 1.0)
    safe = monitor.process(_command(1.0, 2), 1.01)
    assert safe.speed_mps == 0.0
    assert monitor.machine.fault_reason == "non_increasing_command_stamp"


def test_future_sensor_timestamp_prevents_arm():
    monitor = SafetyMonitor()
    monitor.update_scan(2.0)
    monitor.update_pose(1.0)
    with pytest.raises(RuntimeError, match="scan_timestamp_future"):
        monitor.arm(1.0)
