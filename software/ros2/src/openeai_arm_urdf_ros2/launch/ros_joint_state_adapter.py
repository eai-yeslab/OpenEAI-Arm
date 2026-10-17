"""Read-only ROS 2 JointState adapter for the full OpenEAI gripper URDF.

The pure mapping functions are standard-library only. ROS imports and runtime
creation are delayed until main(), so CPU fixtures need no ROS installation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence


DEFAULT_INPUT_JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 8))
DEFAULT_INPUT_TOPIC = "/right/joint_states"
DEFAULT_OUTPUT_TOPIC = "/openeai_urdf_visual/joint_states"
WIDTH_BOUNDARY_TOLERANCE_M = 1e-7
DEFAULT_METADATA_NAME = "gripper_coupling_metadata.json"


def _finite_values(values: Sequence[float], expected_size: int, label: str) -> tuple[float, ...]:
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain {expected_size} finite values") from exc
    if len(converted) != expected_size:
        raise ValueError(f"{label} must contain {expected_size} finite values")
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} must contain {expected_size} finite values")
    return converted


def _metadata_fields(metadata: Mapping) -> tuple[tuple[str, ...], float, float, Mapping]:
    try:
        names = tuple(metadata["joint_names"])
        driver_min = float(metadata["driver_min_rad"])
        driver_max = float(metadata["driver_max_rad"])
        coupling = metadata["coupling"]
        for key in ("closed_theta_rad", "closed_follower_x_m", "crank_radius_m",
                    "guide_y_m", "rod_length_m", "positive_rod_closed_angle_rad",
                    "negative_rod_closed_angle_rad"):
            if not math.isfinite(float(coupling[key])):
                raise ValueError(f"Invalid coupling field: {key}")
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Metadata is missing valid full-gripper coupling fields") from exc
    if len(names) != 11 or len(set(names)) != 11:
        raise ValueError("Metadata must define eleven unique visual joint names")
    if names[:6] != tuple(str(i) for i in range(1, 7)):
        raise ValueError("Metadata arm joint names must be ordered as 1 through 6")
    if not (math.isfinite(driver_min) and math.isfinite(driver_max) and driver_min < driver_max):
        raise ValueError("Metadata has invalid driver limits")
    return names, driver_min, driver_max, coupling


def urdf_gripper_joint_values(driver_rad: float, metadata: Mapping) -> dict[str, float]:
    """Reproduce the installed package's analytic signed five-joint closure map."""
    _, driver_min, driver_max, p = _metadata_fields(metadata)
    driver = float(driver_rad)
    if not math.isfinite(driver):
        raise ValueError("driver_rad must be finite")
    if driver < driver_min or driver > driver_max:
        raise ValueError("driver angle is outside the URDF joint limits")

    opening = -driver
    # The URDF uses a +Z hinge axis with negative signed coordinates for opening.
    theta = float(p["closed_theta_rad"]) - opening
    radius = float(p["crank_radius_m"])
    rod_length = float(p["rod_length_m"])
    guide_y = float(p["guide_y_m"])
    y_error = guide_y - radius * math.sin(theta)
    radicand = rod_length * rod_length - y_error * y_error
    if radicand < -1e-12:
        raise ValueError("driver angle is outside the linkage closure domain")
    follower_x = radius * math.cos(theta) + math.sqrt(max(0.0, radicand))
    delta = follower_x - float(p["closed_follower_x_m"])
    alpha_positive = math.atan2(y_error, follower_x - radius * math.cos(theta))
    alpha_negative = math.atan2(-y_error, -follower_x + radius * math.cos(theta))

    def wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    values = {
        "gripper_crank_joint": driver,
        "gripper_positive_jaw_joint": delta,
        "gripper_negative_jaw_joint": -delta,
        "gripper_positive_rod_joint": wrap(alpha_positive - float(p["positive_rod_closed_angle_rad"])) + opening,
        "gripper_negative_rod_joint": wrap(alpha_negative - float(p["negative_rod_closed_angle_rad"])) + opening,
    }
    # Validate both rod closures as a guard against malformed or incompatible metadata.
    crank_x = radius * math.cos(theta)
    crank_y = radius * math.sin(theta)
    positive_residual = abs(math.hypot(follower_x - crank_x, guide_y - crank_y) - rod_length)
    negative_residual = abs(math.hypot(-follower_x + crank_x, -guide_y + crank_y) - rod_length)
    if max(positive_residual, negative_residual) > 1e-9:
        raise ValueError("analytic gripper coupling failed its rod-closure check")
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("analytic gripper coupling produced a nonfinite value")
    return {name: (0.0 if abs(value) < 1e-14 else value) for name, value in values.items()}


def _width_from_driver(driver_rad: float, driver_min: float, driver_max: float,
                       coupling: Mapping) -> float:
    """Return jaw width from the metadata linkage, zeroed at its closed pose."""
    closed_theta = float(coupling["closed_theta_rad"])
    radius = float(coupling["crank_radius_m"])
    guide_y = float(coupling["guide_y_m"])
    rod_length = float(coupling["rod_length_m"])
    closed_x = float(coupling["closed_follower_x_m"])
    theta = closed_theta + driver_rad  # signed URDF driver: range is negative when opening
    y_error = guide_y - radius * math.sin(theta)
    radicand = rod_length * rod_length - y_error * y_error
    if radicand < -1e-12:
        raise ValueError("metadata driver range leaves the linkage closure domain")
    follower_x = radius * math.cos(theta) + math.sqrt(max(0.0, radicand))
    width = 2.0 * (follower_x - closed_x)
    if not math.isfinite(width):
        raise ValueError("metadata produced a nonfinite jaw width")
    return width


def driver_from_width(width_m: float, metadata: Mapping) -> float:
    """Invert the metadata-derived monotone aperture map by bounded bisection."""
    _, driver_min, driver_max, coupling = _metadata_fields(metadata)
    width = float(width_m)
    if not math.isfinite(width):
        raise ValueError("gripper width must be finite")
    width_max = _width_from_driver(driver_min, driver_min, driver_max, coupling)
    if width < -WIDTH_BOUNDARY_TOLERANCE_M or width > width_max + WIDTH_BOUNDARY_TOLERANCE_M:
        raise ValueError(f"gripper width is outside [0, {width_max:.9g}] m")
    # Permit only the small binary-float tolerance at physical endpoints.
    width = min(max(width, 0.0), width_max)
    if width == 0.0:
        return driver_max
    if width == width_max:
        return driver_min

    lo, hi = driver_min, driver_max  # aperture decreases monotonically from lo to hi
    for _ in range(80):
        mid = (lo + hi) * 0.5
        if _width_from_driver(mid, driver_min, driver_max, coupling) > width:
            lo = mid
        else:
            hi = mid
    return (lo + hi) * 0.5


def adapt_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    metadata: Mapping,
    *,
    input_joint_names: Sequence[str] = DEFAULT_INPUT_JOINT_NAMES,
    gripper_input: str = "width",
) -> tuple[tuple[str, ...], tuple[float, ...]]:
    """Convert a named original seven-joint state into eleven visual positions.

    Arm coordinates are preserved verbatim: original OpenEAI feedback already
    subtracts reset offsets. Only the gripper position is explicitly converted.
    """
    output_names, driver_min, driver_max, _ = _metadata_fields(metadata)
    source_names = tuple(str(name) for name in names)
    if len(source_names) != len(positions):
        raise ValueError("JointState name and position arrays must have equal lengths")
    if any(not name for name in source_names):
        raise ValueError("JointState names must be nonempty")
    if len(set(source_names)) != len(source_names):
        raise ValueError("JointState contains duplicate names")
    source_positions = _finite_values(positions, len(positions), "JointState positions")
    required_names = tuple(str(name) for name in input_joint_names)
    if len(required_names) != 7 or len(set(required_names)) != 7 or any(not name for name in required_names):
        raise ValueError("input_joint_names must contain seven unique nonempty names")
    name_to_position = dict(zip(source_names, source_positions))
    missing = [name for name in required_names if name not in name_to_position]
    if missing:
        raise ValueError("JointState is missing required names: " + ", ".join(missing))
    if gripper_input not in ("width", "driver_rad"):
        raise ValueError("gripper_input must be 'width' or 'driver_rad'")

    arm_positions = {str(i): name_to_position[required_names[i - 1]] for i in range(1, 7)}
    raw_gripper = name_to_position[required_names[6]]
    if gripper_input == "width":
        driver = driver_from_width(raw_gripper, metadata)
    else:
        driver = raw_gripper
        if driver < driver_min or driver > driver_max:
            raise ValueError("driver angle is outside the URDF joint limits")
    gripper_positions = urdf_gripper_joint_values(driver, metadata)
    positions_by_name = {**arm_positions, **gripper_positions}
    if set(positions_by_name) != set(output_names):
        raise ValueError("Metadata output joints do not match the mapped arm and gripper joints")
    output_positions = tuple(float(positions_by_name[name]) for name in output_names)
    if not all(math.isfinite(value) for value in output_positions):
        raise ValueError("Converted visual joint positions are nonfinite")
    return output_names, output_positions


def resolve_metadata_path(override: str | Path | None) -> Path:
    if override is not None:
        path = Path(override).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Gripper coupling metadata not found: {path}")
        return path
    try:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory("openeai_arm_urdf_ros2")) / "urdf" / DEFAULT_METADATA_NAME
        if path.is_file():
            return path
    except ImportError:
        pass
    local_candidates = (
        Path(__file__).resolve().parent / "urdf" / DEFAULT_METADATA_NAME,
        Path(__file__).resolve().parent.parent / "urdf" / DEFAULT_METADATA_NAME,
    )
    for path in local_candidates:
        if path.is_file():
            return path
    raise FileNotFoundError("Pass --metadata or install the openeai_arm_urdf_ros2 package metadata")


def make_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC)
    parser.add_argument("--output-topic", default=DEFAULT_OUTPUT_TOPIC)
    parser.add_argument("--gripper-input", choices=("width", "driver_rad"), default="width")
    parser.add_argument("--metadata", type=Path, default=None,
                        help="Override the installed urdf/gripper_coupling_metadata.json")
    parser.add_argument("--input-joint-names", nargs=7, default=DEFAULT_INPUT_JOINT_NAMES,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = make_argument_parser()
    options, ros_args = parser.parse_known_args(argv)
    if not options.input_topic.strip() or not options.output_topic.strip():
        parser.error("input and output topics must be nonempty")
    if options.input_topic == options.output_topic:
        parser.error("input and output topics must differ")
    try:
        metadata_path = resolve_metadata_path(options.metadata)
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        _metadata_fields(metadata)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))

    # No ROS dependency is imported until a caller explicitly runs this node.
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        raise RuntimeError("ROS 2 Python packages are required to run the adapter node") from exc

    class JointStateVisualizationAdapter(Node):
        def __init__(self):
            super().__init__("openeai_joint_state_visual_adapter")
            self._metadata = metadata
            self._input_names = tuple(options.input_joint_names)
            self._gripper_input = options.gripper_input
            self._publisher = self.create_publisher(JointState, options.output_topic, 10)
            self._subscription = self.create_subscription(
                JointState, options.input_topic, self._on_joint_state, 10
            )
            self.get_logger().info(
                f"Read-only visualization adapter: {options.input_topic} -> {options.output_topic}; "
                f"gripper_input={options.gripper_input}; metadata={metadata_path}"
            )

        def _on_joint_state(self, incoming):
            try:
                output_names, output_positions = adapt_joint_state(
                    incoming.name,
                    incoming.position,
                    self._metadata,
                    input_joint_names=self._input_names,
                    gripper_input=self._gripper_input,
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(f"Dropped invalid JointState: {exc}")
                return
            outgoing = JointState()
            outgoing.header = incoming.header
            outgoing.name = list(output_names)
            outgoing.position = list(output_positions)
            # Width and motor angular velocities use different units; do not forward them.
            outgoing.velocity = []
            outgoing.effort = []
            self._publisher.publish(outgoing)

    rclpy.init(args=ros_args)
    node = JointStateVisualizationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
