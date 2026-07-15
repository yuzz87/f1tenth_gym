from examples.waypoint_follow import PurePursuitPlanner

from .base_controller import BaseController


class PurePursuitController(BaseController):
    """Thin wrapper that adapts the existing Pure Pursuit planner."""

    def __init__(self, conf, car_params):
        self.conf = conf
        self.controller_conf = conf.controller
        wheelbase = car_params["lf"] + car_params["lr"]
        self.planner = PurePursuitPlanner(conf, wheelbase)
        self.waypoints = self.planner.waypoints

    def plan(self, obs):
        speed, steer = self.planner.plan(
            obs["poses_x"][0],
            obs["poses_y"][0],
            obs["poses_theta"][0],
            self.controller_conf["tlad"],
            self.controller_conf["vgain"],
        )
        return float(speed), float(steer)

    def render_waypoints(self, env_renderer):
        self.planner.render_waypoints(env_renderer)

