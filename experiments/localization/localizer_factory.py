from .feature_localizer import FeatureMatchingLocalizer
from .map_localizer import BruteForceMapLocalizer


def create_localizer(conf, scan_angles):
    """Instantiate an experiment-local localizer from config."""
    localizer_conf = getattr(conf, "localizer", None)
    if localizer_conf is None:
        raise ValueError("Config must include a localizer section.")

    localizer_type = localizer_conf.get("type", "map_localizer").lower()
    # 環境と同じLiDAR設定をlocalizerの地図ray castingにも渡す。
    lidar_config = getattr(conf, "lidar", None)
    if localizer_type == "map_localizer":
        return BruteForceMapLocalizer(
            conf, localizer_conf, scan_angles, lidar_config=lidar_config
        )
    if localizer_type == "feature_localizer":
        return FeatureMatchingLocalizer(
            conf, localizer_conf, scan_angles, lidar_config=lidar_config
        )

    raise ValueError(f"Unknown localizer type: {localizer_type}")
