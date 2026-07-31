"""Small command policies shared by actuator implementations."""


def is_neutral_stop(command, tolerance=1e-9):
    return (
        float(command.speed_mps) <= float(tolerance)
        and abs(float(command.steering_angle_rad)) <= float(tolerance)
        and abs(float(command.steering_rate_rad_s)) <= float(tolerance)
    )
