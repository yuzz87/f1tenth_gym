from pathlib import Path


def test_dry_run_launch_contains_no_actuator_executable():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch/real_controller_dry_run.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")

    assert 'executable="udp_actuator_bridge"' not in source
    assert 'executable="actuator_node"' not in source
    assert 'executable="mock_vehicle_node"' not in source
    assert '"/dry_run/control_safe"' in source


def test_dry_run_start_script_does_not_send_to_actuator_udp_port():
    script_path = (
        Path(__file__).resolve().parents[3]
        / "scripts/run_real_controller_dry_run_pc.sh"
    )
    source = script_path.read_text(encoding="utf-8")

    assert "pi_udp_host" not in source
    assert "5005" not in source
