class BaseController:
    """Common interface for experiment-local controllers."""

    def plan(self, obs):
        raise NotImplementedError

    def render_waypoints(self, env_renderer):
        del env_renderer

