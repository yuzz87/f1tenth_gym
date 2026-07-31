class BaseLocalizer:
    """Common interface for experiment-local localizers."""

    def initialize(self, initial_pose):
        raise NotImplementedError

    def update(self, obs, control=None):
        del control
        raise NotImplementedError

    def predict(self, obs, control=None):
        """Predict the pose without running scan matching."""
        del obs, control
        raise NotImplementedError

    def debug_info(self):
        return {}
