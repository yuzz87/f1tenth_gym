"""LiDAR profiles and LaserScan-compatible geometry metadata."""

import math
from collections.abc import Mapping


# 従来のシミュレータ互換プロファイル
LEGACY_LIDAR_PROFILE = {
    "profile": "legacy",
    "model": "f110_legacy",
    "num_beams": 1080,
    "fov": 4.7,
    "range_min": 0.0,
    "range_max": 30.0,
    "scan_rate_hz": None,
    "noise_std": 0.01,
    "noise_std_per_meter": 0.0,
    "dropout_probability": 0.0,
    "scan_delay": 0.0,
    "frame_id": "laser",
    "angle_endpoint_inclusive": True,
}


# RPLiDAR A1M8-R6向けプロファイル
A1M8_R6_LIDAR_PROFILE = {
    "profile": "rplidar_a1m8_r6",
    "model": "rplidar_a1m8_r6",
    "num_beams": 360,
    "fov": 2.0 * math.pi,
    "range_min": 0.15,
    "range_max": 12.0,
    "scan_rate_hz": 5.5,
    # Step 2では理想LiDARを既定値にし、Step 3で実験条件として変更する。
    "noise_std": 0.0,
    "noise_std_per_meter": 0.0,
    "dropout_probability": 0.0,
    "scan_delay": 0.0,
    "frame_id": "laser",
    # The current simulator includes both endpoints. Step 2 will validate
    # whether the real /scan stream should use a non-duplicated full circle.
    "angle_endpoint_inclusive": True,
}


# 名前からプロファイルを検索するための一覧
LIDAR_PROFILES = {
    "legacy": LEGACY_LIDAR_PROFILE,
    "rplidar_a1m8_r6": A1M8_R6_LIDAR_PROFILE,
}


def get_lidar_extrinsics(config=None, legacy_lidar_dist=0.0, overrides=None):
    """Return the base_link -> laser 2D extrinsic parameters."""

    # lidar_distは従来設定との互換用。新しいx_offsetがあれば優先する。
    source = dict(config or {})
    source.update(overrides or {})
    return {
        "x_offset": float(source.get("x_offset", legacy_lidar_dist)),
        "y_offset": float(source.get("y_offset", 0.0)),
        "z_offset": float(source.get("z_offset", 0.0)),
        "yaw_offset": float(source.get("yaw_offset", 0.0)),
    }


def transform_pose_to_lidar(vehicle_pose, extrinsics):
    """Transform a vehicle pose into the LiDAR frame origin pose."""

    pose = [float(value) for value in vehicle_pose]
    x_offset = float(extrinsics["x_offset"])
    y_offset = float(extrinsics["y_offset"])
    yaw_offset = float(extrinsics["yaw_offset"])
    theta = pose[2]
    return (
        pose[0] + x_offset * math.cos(theta) - y_offset * math.sin(theta),
        pose[1] + x_offset * math.sin(theta) + y_offset * math.cos(theta),
        theta + yaw_offset,
    )


def resolve_lidar_config(config=None):
    """Resolve a profile name or mapping into a validated plain dictionary.

    ``None`` preserves the simulator's historical LiDAR behavior. A mapping
    can select a profile with ``profile`` and override individual fields.
    """

    # 入力形式を辞書に統一する
    if config is None:
        config = {}
    elif isinstance(config, str):
        config = {"profile": config}
    elif not isinstance(config, Mapping):
        raise TypeError("lidar configuration must be a profile name or mapping")

    config = dict(config)
    profile_name = config.get("profile", "legacy")
    try:
        # 選択したプロファイルをコピーして設定の土台にする
        resolved = dict(LIDAR_PROFILES[profile_name])
    except KeyError as exc:
        available = ", ".join(sorted(LIDAR_PROFILES))
        raise ValueError(
            f"unknown LiDAR profile {profile_name!r}; choose one of: {available}"
        ) from exc

    # 指定された値でプロファイルの設定を上書きする
    resolved.update({key: value for key, value in config.items() if key != "profile"})
    resolved["profile"] = profile_name

    # 未指定の角度情報をビーム数と視野角から計算する
    if "angle_min" not in config:
        resolved["angle_min"] = -resolved["fov"] / 2.0
    resolved["angle_increment"] = get_angle_increment(resolved)
    resolved["angle_max"] = get_angle_max(resolved)
    # 計算後の設定が正しいか検証する
    validate_lidar_config(resolved)
    return resolved


def get_angle_increment(config):
    """Return the inclusive-endpoint angle increment used by the simulator."""

    # 両端を含むため、間隔はビーム数 - 1 で割る
    return float(config["fov"]) / (int(config["num_beams"]) - 1)


def get_angle_max(config):
    """Return the final angle represented by the scan array."""

    # 最初の角度に、間隔をビーム数 - 1 回分足す
    return float(config["angle_min"]) + (
        int(config["num_beams"]) - 1
    ) * float(config["angle_increment"])


def validate_lidar_config(config):
    """Validate fields needed by the Step 1 scan interface."""

    num_beams = int(config["num_beams"])
    fov = float(config["fov"])
    range_min = float(config["range_min"])
    range_max = float(config["range_max"])

    # 各設定値が有効な範囲にあるか確認する
    if num_beams < 2:
        raise ValueError("LiDAR num_beams must be at least 2")
    if not 0.0 < fov <= 2.0 * math.pi:
        raise ValueError("LiDAR fov must be in the interval (0, 2*pi]")
    if range_min < 0.0 or range_min >= range_max:
        raise ValueError("LiDAR range_min must be smaller than range_max")
    if config["scan_rate_hz"] is not None and float(config["scan_rate_hz"]) <= 0.0:
        raise ValueError("LiDAR scan_rate_hz must be positive when specified")
    if float(config["noise_std"]) < 0.0:
        raise ValueError("LiDAR noise_std must be non-negative")
    if float(config["noise_std_per_meter"]) < 0.0:
        raise ValueError("LiDAR noise_std_per_meter must be non-negative")
    if not 0.0 <= float(config["dropout_probability"]) <= 1.0:
        raise ValueError("LiDAR dropout_probability must be in [0, 1]")
    if float(config["scan_delay"]) < 0.0:
        raise ValueError("LiDAR scan_delay must be non-negative")
    for field in ("x_offset", "y_offset", "z_offset", "yaw_offset"):
        if field in config and not math.isfinite(float(config[field])):
            raise ValueError(f"LiDAR {field} must be finite")


def get_scan_angles(config):
    """Return the scan angles in the same order as the distance array."""

    import numpy as np

    # 距離配列と同じ順番の角度配列を作る
    return np.linspace(
        float(config["angle_min"]),
        float(config["angle_max"]),
        int(config["num_beams"]),
    )
