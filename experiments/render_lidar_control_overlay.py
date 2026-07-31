"""F1TENTH GUIへLiDARと制御情報を重ねて描画する。"""

import math

import pyglet

from f110_gym.envs.lidar_config import (
    get_lidar_extrinsics,
    transform_pose_to_lidar,
)
from pyglet.gl import GL_LINES


class LidarControlOverlay:
    """LiDARレイ、推定姿勢、操舵方向、計測値を描画する。"""

    WORLD_SCALE = 50.0

    def __init__(self, scan_angles, visual_state, range_max=12.0, beam_stride=4):
        self.scan_angles = scan_angles
        self.visual_state = visual_state
        self.range_max = float(range_max)
        self.beam_stride = max(1, int(beam_stride))
        self.scan_lines = None
        self.pose_lines = None
        self.control_lines = None
        self.info_label = None

    def render_callback(self, renderer):
        if self.info_label is None:
            self.info_label = pyglet.text.Label(
                "",
                font_size=14,
                color=(25, 35, 45, 255),
                batch=renderer.batch,
            )

        self._delete_vertex_list("scan_lines")
        self._delete_vertex_list("pose_lines")
        self._delete_vertex_list("control_lines")

        obs = self.visual_state.get("obs")
        if obs is None:
            return

        pose_x = float(obs["poses_x"][0])
        pose_y = float(obs["poses_y"][0])
        pose_theta = float(obs["poses_theta"][0])
        scan = obs.get("scans", [[]])[0]
        extrinsics = get_lidar_extrinsics(
            self.visual_state.get("lidar_config"),
            legacy_lidar_dist=float(self.visual_state.get("lidar_dist", 0.0)),
        )
        lidar_x, lidar_y, lidar_theta = transform_pose_to_lidar(
            (pose_x, pose_y, pose_theta), extrinsics
        )

        scan_vertices = []
        scan_colors = []
        for index in range(0, min(len(scan), len(self.scan_angles)), self.beam_stride):
            distance = min(max(float(scan[index]), 0.0), self.range_max)
            angle = lidar_theta + float(self.scan_angles[index])
            end_x = lidar_x + distance * math.cos(angle)
            end_y = lidar_y + distance * math.sin(angle)
            scan_vertices.extend(
                [
                    lidar_x * self.WORLD_SCALE,
                    lidar_y * self.WORLD_SCALE,
                    end_x * self.WORLD_SCALE,
                    end_y * self.WORLD_SCALE,
                ]
            )
            scan_colors.extend([70, 190, 220, 70, 190, 220])
        if scan_vertices:
            self.scan_lines = renderer.batch.add(
                len(scan_vertices) // 2,
                GL_LINES,
                None,
                ("v2f", scan_vertices),
                ("c3B", scan_colors),
            )

        est_pose = self.visual_state.get("est_pose")
        pose_vertices = []
        pose_colors = []
        if est_pose is not None:
            est_x, est_y, est_theta = [float(value) for value in est_pose]
            pose_vertices = [
                est_x * self.WORLD_SCALE,
                est_y * self.WORLD_SCALE,
                (est_x + 0.5 * math.cos(est_theta)) * self.WORLD_SCALE,
                (est_y + 0.5 * math.sin(est_theta)) * self.WORLD_SCALE,
            ]
            pose_colors = [245, 150, 50, 245, 150, 50]
            self.pose_lines = renderer.batch.add(
                2,
                GL_LINES,
                None,
                ("v2f", pose_vertices),
                ("c3B", pose_colors),
            )

        steer = float(self.visual_state.get("steer_cmd", 0.0))
        control_vertices = [
            pose_x * self.WORLD_SCALE,
            pose_y * self.WORLD_SCALE,
            (pose_x + 0.8 * math.cos(pose_theta + steer)) * self.WORLD_SCALE,
            (pose_y + 0.8 * math.sin(pose_theta + steer)) * self.WORLD_SCALE,
        ]
        self.control_lines = renderer.batch.add(
            2,
            GL_LINES,
            None,
            ("v2f", control_vertices),
            ("c3B", [245, 205, 60, 245, 205, 60]),
        )

        self.info_label.text = self._format_info(obs)
        self.info_label.x = renderer.left + 20
        self.info_label.y = renderer.top - 30

    def _format_info(self, obs):
        controller_type = self.visual_state.get("controller_type", "unknown")
        speed = float(self.visual_state.get("speed_cmd", 0.0))
        steer = float(self.visual_state.get("steer_cmd", 0.0))
        est_pose = self.visual_state.get("est_pose")
        if est_pose is None:
            xy_error = 0.0
        else:
            xy_error = math.hypot(
                float(est_pose[0]) - float(obs["poses_x"][0]),
                float(est_pose[1]) - float(obs["poses_y"][0]),
            )
        scan_age = float(obs.get("scan_ages", [0.0])[0])
        scan_updated = int(bool(obs.get("scan_updated", [False])[0]))
        return (
            f"{controller_type}  v={speed:.2f} m/s  "
            f"steer={steer:.3f} rad  est_err={xy_error:.3f} m  "
            f"scan_age={scan_age:.3f} s  updated={scan_updated}"
        )

    def _delete_vertex_list(self, attribute):
        vertex_list = getattr(self, attribute)
        if vertex_list is not None:
            vertex_list.delete()
            setattr(self, attribute, None)
