import numpy as np


def quat_to_yaw(quat: np.ndarray) -> float:
    """Convert quaternion (x, y, z, w) to yaw (rotation around z)."""
    x, y, z, w = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def si_to_unicycle(u_xy: np.ndarray, theta: float, v_max: float, w_max: float):
    """
    Map single-integrator velocity u = [u_x, u_y] in world frame
    to unicycle (v, omega) given current heading theta.

    - v aligned with heading, limited by v_max
    - omega proportional to angle between u direction and heading, limited by w_max
    """
    if np.linalg.norm(u_xy) < 1e-6:
        return 0.0, 0.0

    desired_dir = np.arctan2(u_xy[1], u_xy[0])
    angle_err = np.arctan2(
        np.sin(desired_dir - theta), np.cos(desired_dir - theta)
    )  # wrap to [-pi, pi]

    v = np.linalg.norm(u_xy) * np.cos(angle_err)
    v = float(np.clip(v, -v_max, v_max))

    omega = 2.0 * angle_err
    omega = float(np.clip(omega, -w_max, w_max))

    return v, omega
