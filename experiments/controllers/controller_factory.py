from .mpc_controller import MPCController
from .mppi_controller import MPPIController
from .pure_pursuit_controller import PurePursuitController


def create_controller(conf, car_params):
    """Instantiate an experiment-local controller from config."""
    controller_type = getattr(conf, "controller_type", "pure_pursuit")
    controller_type = controller_type.lower()

    if controller_type == "pure_pursuit":
        return PurePursuitController(conf, car_params)
    if controller_type == "mpc":
        return MPCController(conf, car_params)
    if controller_type == "mppi":
        return MPPIController(conf, car_params)

    raise ValueError(f"Unknown controller_type: {controller_type}")
