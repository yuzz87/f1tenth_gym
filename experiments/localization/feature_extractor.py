import numpy as np


class BeamFeatureExtractor:
    """Extract a compact LiDAR signature with distance and shape cues."""

    def __init__(self, scan_angles, feature_angles=None):
        self.scan_angles = np.asarray(scan_angles, dtype=float)
        default_feature_angles = [-1.8, -0.9, -0.35, 0.0, 0.35, 0.9, 1.8]
        feature_angles = feature_angles if feature_angles is not None else default_feature_angles
        self.feature_angles = np.asarray(feature_angles, dtype=float)
        self.feature_indices = np.array(
            [int(np.argmin(np.abs(self.scan_angles - angle))) for angle in self.feature_angles],
            dtype=int,
        )

    def extract(self, scan):
        scan = np.asarray(scan, dtype=float)
        features = scan[self.feature_indices]
        left_mean = float(np.mean(features[:3]))
        right_mean = float(np.mean(features[-3:]))
        center = float(features[len(features) // 2])
        normalized = features / max(float(np.max(features)), 1.0)
        gradients = np.diff(features)
        curvature = features[:-2] - 2.0 * features[1:-1] + features[2:]
        symmetry_error = np.abs(features[: len(features) // 2] - features[: len(features) // 2 : -1])
        center_offsets = features - center
        signature = np.concatenate(
            (
                features,
                normalized,
                gradients,
                curvature,
                center_offsets,
                np.array(
                    [
                        left_mean - right_mean,
                        left_mean,
                        right_mean,
                        center,
                        float(np.mean(np.abs(gradients))) if gradients.size else 0.0,
                        float(np.mean(np.abs(curvature))) if curvature.size else 0.0,
                        float(np.mean(symmetry_error)) if symmetry_error.size else 0.0,
                    ],
                    dtype=float,
                ),
            )
        )
        return signature
