"""Self-contained analytic coordinates for the OpenEAI v4 parallel gripper."""
import json
import math
from pathlib import Path

METADATA_PATH = Path(__file__).resolve().with_name("gripper_coupling_metadata.json")


def joint_values_from_driver(driver_rad):
    """Return the five signed URDF gripper joint positions for one crank angle."""
    driver = float(driver_rad)
    if not math.isfinite(driver):
        raise ValueError("driver_rad must be finite")
    metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    if not metadata["driver_min_rad"] <= driver <= metadata["driver_max_rad"]:
        raise ValueError("driver angle is outside the URDF joint limits")
    opening = -driver
    p = metadata["coupling"]
    theta = p["closed_theta_rad"] - opening
    y_error = p["guide_y_m"] - p["crank_radius_m"] * math.sin(theta)
    radicand = p["rod_length_m"] ** 2 - y_error ** 2
    if radicand < -1e-12:
        raise ValueError("driver angle is outside the linkage closure domain")
    follower_x = p["crank_radius_m"] * math.cos(theta) + math.sqrt(max(0.0, radicand))
    delta = follower_x - p["closed_follower_x_m"]
    alpha_positive = math.atan2(y_error, follower_x - p["crank_radius_m"] * math.cos(theta))
    alpha_negative = math.atan2(-y_error, -follower_x + p["crank_radius_m"] * math.cos(theta))
    wrap = lambda angle: math.atan2(math.sin(angle), math.cos(angle))
    values = {
        "gripper_crank_joint": driver,
        "gripper_positive_jaw_joint": delta,
        "gripper_negative_jaw_joint": -delta,
        "gripper_positive_rod_joint": wrap(alpha_positive - p["positive_rod_closed_angle_rad"]) + opening,
        "gripper_negative_rod_joint": wrap(alpha_negative - p["negative_rod_closed_angle_rad"]) + opening,
    }
    residual = abs(math.hypot(follower_x - p["crank_radius_m"] * math.cos(theta), y_error) - p["rod_length_m"])
    if residual > 1e-9 or not all(math.isfinite(value) for value in values.values()):
        raise ValueError("analytic gripper coupling failed its closure or finite-value check")
    return {name: (0.0 if abs(value) < 1e-14 else value) for name, value in values.items()}


__all__ = ["joint_values_from_driver"]
