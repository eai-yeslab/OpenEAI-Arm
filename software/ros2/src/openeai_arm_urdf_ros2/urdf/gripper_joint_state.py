"""Print the expanded eleven-joint OpenEAI state from seven signed inputs."""
import argparse
import json
import math
from gripper_coupling import joint_values_from_driver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-q", type=float, nargs=6, required=True, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                        help="Six arm joint positions in radians, in URDF joint order 1 through 6")
    parser.add_argument("--gripper-driver-rad", type=float, required=True,
                        help="Signed URDF crank angle in radians; zero is closed, opening is negative")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in args.arm_q):
        parser.error("Arm joint values must be finite")
    positions = {str(index): value for index, value in enumerate(args.arm_q, start=1)}
    positions.update(joint_values_from_driver(args.gripper_driver_rad))
    print(json.dumps({"joint_positions_rad": positions}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
