from .feature_localizer import FeatureMatchingLocalizer
from .map_localizer import BruteForceMapLocalizer


def create_localizer(conf, scan_angles):
    """Instantiate an experiment-local localizer from config."""
    localizer_conf = getattr(conf, "localizer", None)
    if localizer_conf is None:
        raise ValueError("Config must include a localizer section.")

    localizer_type = localizer_conf.get("type", "map_localizer").lower()
    if localizer_type == "map_localizer":
        return BruteForceMapLocalizer(conf, localizer_conf, scan_angles)
    if localizer_type == "feature_localizer":
        return FeatureMatchingLocalizer(conf, localizer_conf, scan_angles)

    raise ValueError(f"Unknown localizer type: {localizer_type}")
