import pytest

from real_vehicle_integration.state_machine import SafetyState, SafetyStateMachine


def test_normal_state_sequence():
    machine = SafetyStateMachine()
    machine.arm()
    assert machine.state == SafetyState.READY
    machine.start()
    assert machine.output_enabled
    machine.stop()
    machine.mark_stopped()
    assert machine.state == SafetyState.STOPPED
    machine.reset()
    assert machine.state == SafetyState.DISARMED


def test_start_requires_ready():
    with pytest.raises(RuntimeError):
        SafetyStateMachine().start()


def test_fault_records_reason_and_disables_output():
    machine = SafetyStateMachine()
    machine.arm()
    machine.start()
    machine.fault("test_failure")
    assert machine.state == SafetyState.FAULT
    assert not machine.output_enabled
    assert machine.fault_reason == "test_failure"
