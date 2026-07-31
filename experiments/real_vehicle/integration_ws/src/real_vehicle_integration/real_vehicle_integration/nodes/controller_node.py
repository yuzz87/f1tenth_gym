"""Publish MPC or MPPI requests from the latest localized pose."""

import time

import numpy as np

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from ..configuration import declare_common_parameters, parameter_value
from ..contracts import ControlCommand
from ..repository import enable_repository_imports
from ..ros_support import (
    AckermannDriveStamped,
    contract_to_ackermann_message,
    pose_message_to_contract,
    require_ackermann_messages,
)


class ControllerNode(Node):
    def __init__(self):
        require_ackermann_messages()
        super().__init__("real_vehicle_controller")
        declare_common_parameters(self)
        root = enable_repository_imports()
        from experiments.real_vehicle.evaluation.common import load_real_config, make_controller

        self.declare_parameter(
            "vehicle_config",
            str(root / "experiments/real_vehicle/config/real_vehicle.yaml"),
        )
        self.declare_parameter("controller_type", "mppi")
        self.declare_parameter("reference_type", "straight")
        self.declare_parameter("reference_speed_mps", 0.30)
        self.declare_parameter("straight_goal_x_m", 4.0)
        self.declare_parameter("straight_stop_tolerance_m", 0.10)
        self.declare_parameter("circle_radius_m", 2.0)
        config, parameters, limits = load_real_config(
            parameter_value(self, "vehicle_config")
        )
        self.config = config
        self.parameters = parameters
        self.limits = limits
        controller_type = str(parameter_value(self, "controller_type")).lower()
        self.controller_type = controller_type
        rate_parameter = (
            "mpc_control_rate_hz" if controller_type == "mpc" else "control_rate_hz"
        )
        self.dt = 1.0 / max(float(parameter_value(self, rate_parameter)), 1e-6)
        self.controller = make_controller(
            controller_type,
            parameters,
            limits,
            self.dt,
            config["controller"],
        )
        self.pose = None
        self.steering_angle_rad = 0.0
        self.sequence_id = 0
        self.last_overrun_warning_s = -float("inf")
        self.started_monotonic = time.monotonic()
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            str(parameter_value(self, "control_request_topic")),
            1,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(parameter_value(self, "controller_status_topic")),
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(parameter_value(self, "pose_topic")),
            self._on_pose,
            10,
        )
        self.timer = self.create_timer(self.dt, self._plan)

    def _on_pose(self, message):
        self.pose = pose_message_to_contract(message, self.steering_angle_rad)

    def _reference(self, elapsed_s):
        from experiments.real_vehicle.evaluation.common import (
            circle_reference,
            straight_reference,
        )

        speed = min(
            float(parameter_value(self, "reference_speed_mps")),
            self.limits.speed_max_mps,
        )
        reference_type = str(parameter_value(self, "reference_type")).lower()
        if reference_type == "straight":
            return straight_reference(
                elapsed_s,
                self.controller.horizon,
                self.dt,
                speed,
                float(parameter_value(self, "straight_goal_x_m")),
                y=0.0,
            )
        if reference_type == "circle":
            return circle_reference(
                elapsed_s,
                self.controller.horizon,
                self.dt,
                speed,
                float(parameter_value(self, "circle_radius_m")),
                self.parameters,
            )
        raise ValueError("reference_type must be straight or circle")

    def _plan(self):
        if self.pose is None:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        state = np.array([
            self.pose.phi_rad,
            self.pose.x_m,
            self.pose.y_m,
            self.steering_angle_rad,
        ])
        goal_reached = (
            str(parameter_value(self, "reference_type")).lower() == "straight"
            and self.pose.x_m
            >= float(parameter_value(self, "straight_goal_x_m"))
            - max(float(parameter_value(self, "straight_stop_tolerance_m")), 0.0)
        )
        if goal_reached:
            control = np.array([0.0, -self.steering_angle_rad / self.dt])
            info = {"solve_time_s": 0.0}
        else:
            try:
                reference = self._reference(time.monotonic() - self.started_monotonic)
                control, info = self.controller.plan(state, reference)
            except Exception as exc:
                self.get_logger().error(f"controller failed: {exc}")
                return
        speed = float(np.clip(
            control[0], self.limits.speed_min_mps, self.limits.speed_max_mps
        ))
        steer_rate = float(np.clip(
            control[1],
            self.limits.steer_rate_min_rad_s,
            self.limits.steer_rate_max_rad_s,
        ))
        self.steering_angle_rad = float(np.clip(
            self.steering_angle_rad + steer_rate * self.dt,
            self.limits.steer_min_rad,
            self.limits.steer_max_rad,
        ))
        self.sequence_id += 1
        command = ControlCommand(
            stamp_s=now_s,
            sequence_id=self.sequence_id,
            speed_mps=speed,
            steering_angle_rad=self.steering_angle_rad,
            steering_rate_rad_s=steer_rate,
            frame_id=str(parameter_value(self, "base_frame")),
        )
        self.publisher.publish(contract_to_ackermann_message(command, now.to_msg()))
        solve_ms = 1000.0 * float(info.get("solve_time_s", 0.0))
        self._publish_status(info, solve_ms)
        monotonic_now = time.monotonic()
        if (
            solve_ms > self.dt * 1000.0
            and monotonic_now - self.last_overrun_warning_s >= 1.0
        ):
            self.get_logger().warning(
                f"controller overrun: {solve_ms:.3f} ms > {self.dt * 1000.0:.3f} ms"
            )
            self.last_overrun_warning_s = monotonic_now

    def _publish_status(self, info, solve_ms):
        overrun = solve_ms > self.dt * 1000.0
        deadline = bool(info.get("deadline_exceeded", False))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "real_vehicle_controller"
        status.hardware_id = "pc_optimizer"
        status.level = DiagnosticStatus.WARN if overrun or deadline else DiagnosticStatus.OK
        status.message = "deadline_fallback" if deadline else (
            "period_overrun" if overrun else "running"
        )
        status.values = [
            KeyValue(key="controller_type", value=self.controller_type),
            KeyValue(key="solve_time_ms", value=f"{solve_ms:.6f}"),
            KeyValue(key="period_ms", value=f"{self.dt * 1000.0:.6f}"),
            KeyValue(
                key="cost",
                value=f"{float(info.get('cost', float('nan'))):.9f}",
            ),
            KeyValue(key="overrun", value=str(overrun).lower()),
            KeyValue(key="deadline_exceeded", value=str(deadline).lower()),
            KeyValue(key="optimizer_success", value=str(bool(info.get("success"))).lower()),
            KeyValue(
                key="function_evaluations",
                value=str(int(info.get("function_evaluations", 0))),
            ),
        ]
        message.status = [status]
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
