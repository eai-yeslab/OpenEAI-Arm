#!/usr/bin/env python3
"""Publish a deterministic smooth trajectory for controller comparison.

Publishes a continuous multi-sine trajectory as sensor_msgs/JointState at a
fixed rate (default 50 Hz) on /right/joint_targets. Centers and amplitudes are
derived from real teleop logs in control coordinates, with conservative
amplitudes so the arm stays well inside joint limits.

Usage:
    python3 uniform_trajectory.py [--topic /right/joint_targets]
                                  [--duration 60] [--frame-rate 50]
"""

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Control-space centers and amplitudes (rad), a deliberately small, safe range:
#   - amplitudes are reduced to ~40% of the original teleop-log range so the
#     end effector stays well away from the table
#   - joint_2 center is lowered toward a more upright shoulder pose to raise
#     the end effector; joint_3 center is near the horizontal elbow
# Gripper stays fixed at 0.0.
CENTER = np.array([0.345, 0.90, 1.00, -0.064, 0.668, -0.623, 0.0])
AMPLITUDE = np.array([0.2, 0.5, 0.3, 0.25, 0.15, 0.25, 0.0])

# Five out-of-phase components keep the motion teleop-like and rich, while the
# 0.25-0.70 Hz band keeps peak speeds close to the original script (J1 ~1.73
# vs ~1.67 rad/s). Peak range coeff ~1.11, inside the safe envelope.
FREQS = np.array([0.25, 0.40, 0.50, 0.60, 0.70])  # Hz
WEIGHTS = np.array([0.15, 0.35, 0.25, 0.25, 0.35])
JOINT_PHASE = np.arange(7) * (2.0 * np.pi / 7.0)
FREQ_PHASE = np.array([0.0, 1.7, 0.9, 2.3, 0.6])
PHASES = JOINT_PHASE[:, None] + FREQ_PHASE[None, :]  # (7, 5)
# Fixed per-frame step (rad) used when slowly moving to the trajectory start,
# matching arm_steps_length in config_right_inference.yml.
STEP_SIZE = np.array([0.0025] * 7)


class UniformTrajectoryNode(Node):
    def __init__(self, topic, frame_rate):
        super().__init__("uniform_trajectory")
        self.frame_rate = frame_rate
        self.pub = self.create_publisher(JointState, topic, 10)
        self.state_sub = self.create_subscription(
            JointState, "/right/joint_states", self.state_cb, 10
        )
        self.current = None
        self.last_sent = None
        # rclpy Rate.sleep() blocks forever in a non-spun node, so we spin in a
        # background thread and use real-time sleeps in the publish loop.
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self):
        rclpy.spin(self)

    def state_cb(self, msg):
        if len(msg.position) >= 7:
            self.current = np.array(msg.position[:7], dtype=float)

    def traj(self, t):
        q = CENTER.copy()
        for k in range(len(FREQS)):
            q = q + WEIGHTS[k] * AMPLITUDE * np.sin(
                2.0 * np.pi * FREQS[k] * t + PHASES[:, k]
            )
        return q

    def move_to(self, target, max_time=30.0):
        """Slowly step from the last sent pose to target.

        Follows the same per-frame stepping idea as puppet_arm_publish_continuous
        in inference: each joint moves at most STEP_SIZE per frame (0.0025 rad),
        which at 50 Hz gives 0.125 rad/s. The start is the last commanded pose
        (not the measured pose) so the command stream stays continuous.
        """
        dt = 1.0 / self.frame_rate
        if self.last_sent is None:
            if self.current is None:
                raise SystemExit("No joint-state feedback; cannot move safely.")
            self.last_sent = self.current.copy()
        current = self.last_sent.copy()
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < max_time:
            diff = target - current
            if np.all(np.abs(diff) < 1e-4):
                break
            step = np.sign(diff) * np.minimum(np.abs(diff), STEP_SIZE)
            current = current + step
            self.publish_q(current)
            time.sleep(dt)
        # Land exactly on the target.
        self.publish_q(target.copy())

    def publish_q(self, q):
        self.last_sent = np.array(q, dtype=float).copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i+1}" for i in range(7)]
        msg.position = [float(x) for x in q]
        self.pub.publish(msg)

    def run(self, duration):
        self.spin_thread.start()
        dt = 1.0 / self.frame_rate

        # Wait for the current pose so the ramp starts continuously.
        deadline = time.monotonic() + 5.0
        while self.current is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if self.current is None:
            raise SystemExit(
                "No /right/joint_states feedback within 5s; aborting to avoid "
                "a discontinuous start."
            )

        start_pose = self.current.copy()
        self.last_sent = start_pose.copy()

        # Slowly move to the first trajectory frame, so the motion starts
        # exactly where the slow move ended (fully continuous).
        self.move_to(self.traj(0))

        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < duration:
            q = self.traj(time.monotonic() - t0)
            self.publish_q(q)
            time.sleep(dt)

        # Slowly retract to the original pose after the trajectory ends.
        self.move_to(start_pose)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/right/joint_targets")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--frame-rate", type=int, default=50)
    args = parser.parse_args()

    rclpy.init()
    node = UniformTrajectoryNode(args.topic, args.frame_rate)
    try:
        node.run(args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        if node.spin_thread.is_alive():
            node.spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()

