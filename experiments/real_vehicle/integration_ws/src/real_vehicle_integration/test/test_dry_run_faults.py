import pytest

from real_vehicle_integration.dry_run_faults import DryRunFaultGate


def test_inactive_gate_relays_every_stream():
    gate = DryRunFaultGate("scan_timeout")

    assert gate.should_relay("scan")
    assert gate.should_relay("pose")
    assert gate.should_relay("control")


def test_triggered_scan_timeout_drops_only_scans():
    gate = DryRunFaultGate("scan_timeout")
    gate.trigger()

    assert not gate.should_relay("scan")
    assert gate.should_relay("pose")
    assert gate.should_relay("control")
    assert gate.statistics()["scan_dropped"] == 1

    gate.clear()
    assert gate.should_relay("scan")


def test_emergency_stop_mode_does_not_drop_message_streams():
    gate = DryRunFaultGate("emergency_stop")
    gate.trigger()

    assert gate.emergency_stop_active
    assert gate.should_relay("scan")
    assert gate.should_relay("pose")
    assert gate.should_relay("control")


def test_none_mode_cannot_be_triggered():
    with pytest.raises(RuntimeError, match="fault mode is none"):
        DryRunFaultGate("none").trigger()


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match="fault mode"):
        DryRunFaultGate("unknown")
