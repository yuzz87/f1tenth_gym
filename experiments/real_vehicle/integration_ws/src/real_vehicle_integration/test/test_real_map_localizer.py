import math

import numpy as np

from real_vehicle_integration.real_map_localizer import (
    GridMapLocalizer,
    OccupancyMap,
    odom_delta,
)


def write_test_map(directory):
    yaml_path = directory / "test_map.yaml"
    pgm_path = directory / "test_map.pgm"
    width = 20
    height = 20
    rows = []
    for _row in range(height):
        values = [255] * width
        values[10] = 0
        rows.append(" ".join(str(value) for value in values))
    pgm_path.write_text(
        "P2\n20 20\n255\n" + "\n".join(rows) + "\n",
        encoding="ascii",
    )
    yaml_path.write_text(
        "\n".join([
            "image: test_map.pgm",
            "resolution: 0.1",
            "origin: [0.0, 0.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
        ]) + "\n",
        encoding="ascii",
    )
    return yaml_path


def test_load_ros_pgm_and_coordinate_orientation(tmp_path):
    occupancy_map = OccupancyMap(write_test_map(tmp_path))

    assert occupancy_map.width == 20
    assert occupancy_map.height == 20
    assert occupancy_map.resolution_m == 0.1

    row, column, inside = occupancy_map.world_to_cell(1.0, 1.0)
    assert inside
    assert (row, column) == (9, 10)
    assert occupancy_map.occupied[row, column]


def test_grid_localizer_prefers_scan_endpoint_on_wall(tmp_path):
    occupancy_map = OccupancyMap(write_test_map(tmp_path))
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.2,
        search_xy_m=0.3,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        maximum_match_score_m2=0.2,
        max_scan_beams=1,
        lidar_x_m=0.0,
    )
    estimate, score = localizer.estimate({
        "angle_min": 0.0,
        "angle_increment": 1.0,
        "range_min": 0.15,
        "range_max": 12.0,
        "ranges": np.array([1.0]),
    }, [0.0, 0.0, 1.0])

    assert localizer.last_success
    assert math.isfinite(score)
    assert abs(float(estimate[1])) < 0.051


def test_odom_delta_is_in_vehicle_frame():
    delta = odom_delta(
        np.array([math.pi / 2.0, 1.0, 1.0]),
        np.array([math.pi / 2.0, 1.0, 2.0]),
    )

    assert abs(float(delta[0])) < 1e-9
    assert abs(float(delta[1]) - 1.0) < 1e-9
    assert abs(float(delta[2])) < 1e-9


def test_search_offsets_are_symmetric_and_include_zero():
    offsets = GridMapLocalizer._offsets(0.25, 0.10)

    assert 0.0 in offsets
    assert np.allclose(offsets, -offsets[::-1])
    assert np.isclose(offsets[0], -0.25)
    assert np.isclose(offsets[-1], 0.25)


class ScoreMap:
    def __init__(self, score_function):
        self.score_function = score_function

    def scan_score(self, pose, *_args, **_kwargs):
        return float(self.score_function(np.asarray(pose))), 1


def one_beam_scan():
    return {
        "angle_min": 0.0,
        "angle_increment": 1.0,
        "range_min": 0.15,
        "range_max": 12.0,
        "ranges": np.array([1.0]),
    }


def test_odometry_prior_keeps_prediction_for_small_scan_improvement():
    occupancy_map = ScoreMap(
        lambda pose: (
            0.0095
            if np.allclose(pose[1:], [0.1, 0.0])
            else 0.0100
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.20,
        minimum_scan_score_improvement_m2=0.0,
        maximum_position_correction_m=0.2,
    )

    estimate, score = localizer.estimate(one_beam_scan(), [0.0, 0.0, 0.0])

    assert localizer.last_success
    assert np.allclose(estimate, [0.0, 0.0, 0.0])
    assert score == 0.0100
    assert localizer.last_selection_reason == "prediction_best"


def test_clear_scan_improvement_is_accepted_with_prior_enabled():
    occupancy_map = ScoreMap(
        lambda pose: (
            0.001
            if np.allclose(pose[1:], [0.1, 0.0])
            else 0.020
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.20,
        minimum_scan_score_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
        correction_confirmation_scans=1,
    )

    estimate, score = localizer.estimate(one_beam_scan(), [0.0, 0.0, 0.0])

    assert localizer.last_success
    assert np.allclose(estimate, [0.0, 0.1, 0.0])
    assert score == 0.001
    assert localizer.last_selection_reason == "scan_correction"
    assert localizer.last_position_correction_m == 0.1


def test_hysteresis_rejects_insignificant_improvement_without_prior():
    occupancy_map = ScoreMap(
        lambda pose: (
            0.0095
            if np.allclose(pose[1:], [0.1, 0.0])
            else 0.0100
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
    )

    estimate, score = localizer.estimate(one_beam_scan(), [0.0, 0.0, 0.0])

    assert np.allclose(estimate, [0.0, 0.0, 0.0])
    assert score == 0.0100
    assert localizer.last_selection_reason == "hysteresis_kept_prediction"


def test_objective_hysteresis_rejects_one_grid_false_correction():
    occupancy_map = ScoreMap(
        lambda pose: (
            0.01525
            if np.allclose(pose[1:], [0.15, 0.0])
            else 0.019972222
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.15,
        theta_step_rad=0.10,
        search_xy_m=0.15,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.20,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.20,
    )

    estimate, score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert localizer.last_success
    assert np.allclose(estimate, [0.0, 0.0, 0.0])
    assert score == 0.019972222
    assert (
        localizer.last_selection_reason
        == "objective_hysteresis_kept_prediction"
    )
    assert math.isclose(localizer.last_scan_improvement_m2, 0.004722222)
    assert math.isclose(
        localizer.last_objective_improvement_m2,
        0.000222222,
    )
    assert localizer.last_correction_dx_m == 0.0
    assert localizer.last_proposed_correction_dx_m == 0.15
    assert localizer.correction_event_id == 0


def test_applied_correction_records_signed_vector_and_event_id():
    occupancy_map = ScoreMap(
        lambda pose: (
            0.001
            if np.allclose(pose, [0.05, 0.05, -0.05])
            else 0.020
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.05,
        theta_step_rad=0.05,
        search_xy_m=0.05,
        search_theta_rad=0.05,
        minimum_valid_beams=1,
        position_prior_weight=0.0,
        yaw_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.10,
        maximum_yaw_correction_rad=0.10,
        correction_confirmation_scans=1,
    )

    estimate, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(estimate, [0.05, 0.05, -0.05])
    assert localizer.last_selection_reason == "scan_correction"
    assert localizer.last_correction_dx_m == 0.05
    assert localizer.last_correction_dy_m == -0.05
    assert localizer.last_correction_dyaw_rad == 0.05
    assert localizer.correction_event_id == 1


def test_fine_search_candidate_count_is_bounded():
    localizer = GridMapLocalizer(
        ScoreMap(lambda _pose: 0.01),
        xy_step_m=0.05,
        theta_step_rad=0.05,
        search_xy_m=0.30,
        search_theta_rad=0.25,
        minimum_valid_beams=1,
        maximum_position_correction_m=0.10,
        maximum_yaw_correction_rad=0.10,
    )

    estimate, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(estimate, [0.0, 0.0, 0.0])
    assert localizer.last_candidate_count == 65


def test_consistent_correction_is_applied_after_confirmation():
    occupancy_map = ScoreMap(
        lambda pose: 0.001 if np.isclose(pose[1], 0.1) else 0.020
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
        correction_confirmation_scans=2,
        correction_cooldown_scans=0,
    )

    first, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(first, [0.0, 0.0, 0.0])
    assert localizer.last_selection_reason == "confirmation_pending"
    assert localizer.last_confirmation_count == 1
    assert localizer.correction_pending_count == 1
    assert localizer.correction_event_id == 0

    second, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(second, [0.0, 0.1, 0.0])
    assert localizer.last_selection_reason == "scan_correction"
    assert localizer.last_confirmation_count == 2
    assert localizer.correction_event_id == 1


def test_inconsistent_correction_restarts_confirmation():
    target = {"x": 0.1}
    occupancy_map = ScoreMap(
        lambda pose: (
            0.001 if np.isclose(pose[1], target["x"]) else 0.020
        )
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
        correction_confirmation_scans=2,
        correction_consistency_position_m=0.01,
    )

    first, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )
    target["x"] = -0.1
    second, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(first, [0.0, 0.0, 0.0])
    assert np.allclose(second, [0.0, 0.0, 0.0])
    assert localizer.last_selection_reason == "confirmation_pending"
    assert localizer.last_confirmation_count == 1
    assert localizer.pending_reset_count == 1

    third, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(third, [0.0, -0.1, 0.0])
    assert localizer.correction_event_id == 1


def test_cooldown_blocks_repeated_correction_proposals():
    occupancy_map = ScoreMap(
        lambda pose: 0.001 if np.isclose(pose[1], 0.1) else 0.020
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        position_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
        correction_confirmation_scans=1,
        correction_cooldown_scans=2,
    )

    applied, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )
    blocked, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert np.allclose(applied, [0.0, 0.1, 0.0])
    assert np.allclose(blocked, [0.0, 0.0, 0.0])
    assert localizer.last_selection_reason == "cooldown_kept_prediction"
    assert localizer.last_cooldown_scans_remaining == 1
    assert localizer.correction_rejection_count == 1


def test_bad_prediction_bypasses_confirmation_for_recovery():
    occupancy_map = ScoreMap(
        lambda pose: 0.001 if np.isclose(pose[1], 0.1) else 0.50
    )
    localizer = GridMapLocalizer(
        occupancy_map,
        xy_step_m=0.1,
        theta_step_rad=0.1,
        search_xy_m=0.1,
        search_theta_rad=0.0,
        minimum_valid_beams=1,
        maximum_match_score_m2=0.25,
        position_prior_weight=0.0,
        minimum_scan_score_improvement_m2=0.001,
        minimum_objective_improvement_m2=0.001,
        maximum_position_correction_m=0.2,
        correction_confirmation_scans=3,
    )

    estimate, _score = localizer.estimate(
        one_beam_scan(),
        [0.0, 0.0, 0.0],
    )

    assert localizer.last_success
    assert np.allclose(estimate, [0.0, 0.1, 0.0])
    assert localizer.last_selection_reason == "scan_correction"
    assert localizer.correction_event_id == 1
