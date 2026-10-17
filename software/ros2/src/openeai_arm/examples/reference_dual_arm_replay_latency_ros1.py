#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import h5py
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from replay_latency_metrics import (
    LatencyTrace,
    SynchronizedFeedbackRecorder,
    analyze_latency_trace,
    print_latency_report,
    save_latency_report,
)


JOINT_NAMES = [f"joint{index}" for index in range(7)]


def load_actions(path, field):
    with h5py.File(path, "r") as handle:
        if field not in handle:
            raise KeyError(f"Field '{field}' is missing. Available keys: {list(handle.keys())}")
        actions = np.asarray(handle[field], dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] < 7 or actions.shape[1] % 7 != 0:
        raise ValueError(
            "Replay actions must contain one or more complete 7-DoF arms, "
            f"got shape {actions.shape}"
        )
    return actions


class DualArmReplayNode:
    def __init__(self, args):
        self.args = args
        self.trace = LatencyTrace()
        self.feedback = SynchronizedFeedbackRecorder(["left", "right"], self.trace)
        self.latest = {}
        self.left_publisher = rospy.Publisher(args.left_command_topic, JointState, queue_size=10)
        self.right_publisher = rospy.Publisher(args.right_command_topic, JointState, queue_size=10)
        self.left_subscription = rospy.Subscriber(
            args.left_feedback_topic,
            JointState,
            self._feedback_callback,
            callback_args="left",
            queue_size=1000,
            tcp_nodelay=True,
        )
        self.right_subscription = rospy.Subscriber(
            args.right_feedback_topic,
            JointState,
            self._feedback_callback,
            callback_args="right",
            queue_size=1000,
            tcp_nodelay=True,
        )

    def _feedback_callback(self, message, arm_key):
        position = np.asarray(message.position, dtype=np.float64)
        if position.shape != (7,):
            rospy.logwarn_throttle(2.0, f"Ignoring {arm_key} feedback with shape {position.shape}")
            return
        timestamp = time.perf_counter()
        self.latest[arm_key] = position.copy()
        self.feedback.update(arm_key, position, timestamp)

    def wait_for_feedback(self, timeout_seconds=10.0):
        deadline = time.perf_counter() + timeout_seconds
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and time.perf_counter() < deadline:
            if "left" in self.latest and "right" in self.latest:
                return
            rate.sleep()
        raise RuntimeError("Timed out waiting for both arm feedback topics")

    def publish(self, action, record):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (14,):
            raise ValueError(f"Expected a 14-dimensional action, got {action.shape}")
        timestamp = time.perf_counter()
        left_message = JointState()
        left_message.header = Header(stamp=rospy.Time.now())
        left_message.name = JOINT_NAMES
        left_message.position = action[:7].tolist()
        right_message = JointState()
        right_message.header = Header(stamp=left_message.header.stamp)
        right_message.name = JOINT_NAMES
        right_message.position = action[7:].tolist()
        self.left_publisher.publish(left_message)
        self.right_publisher.publish(right_message)
        if record:
            self.trace.record_command(action, timestamp)

    def move_to_start(self, target):
        self.wait_for_feedback()
        current = np.concatenate([self.latest["left"], self.latest["right"]])
        target = np.asarray(target, dtype=np.float64)
        steps = np.tile(np.asarray(self.args.start_step_sizes, dtype=np.float64), 2)
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown():
            delta = target - current
            if np.all(np.abs(delta) <= steps):
                self.publish(target, record=False)
                return
            current += np.clip(delta, -steps, steps)
            self.publish(current, record=False)
            rate.sleep()

    def replay(self, actions):
        self.wait_for_feedback()
        source_arm_count = actions.shape[1] // 7
        if source_arm_count > 2:
            print(
                "Replay contains more than two arms; using the first two arms."
            )
            actions = actions[:, :14]
        elif source_arm_count == 1:
            stationary_right = np.asarray(
                self.latest["right"], dtype=np.float64
            )[:7]
            actions = np.concatenate(
                [
                    actions,
                    np.tile(stationary_right, (len(actions), 1)),
                ],
                axis=1,
            )
            print(
                "Single-arm replay detected; the right arm will hold its "
                "current feedback pose."
            )

        if not self.args.skip_move_to_start:
            print("Moving both arms to the first replay frame.")
            self.move_to_start(actions[0])
        else:
            self.wait_for_feedback()

        if not self.args.yes:
            input("Press Enter to start replay: ")

        frame_count = min(len(actions), self.args.max_steps)
        rate = rospy.Rate(self.args.publish_rate)
        start_time = time.perf_counter()
        for index in range(frame_count):
            if rospy.is_shutdown():
                break
            self.publish(actions[index], record=True)
            rate.sleep()

        if not self.trace.snapshot()[0].size:
            raise RuntimeError("No replay command was published")

        final_action = actions[frame_count - 1]
        settle_deadline = time.perf_counter() + self.args.settle_seconds
        while not rospy.is_shutdown() and time.perf_counter() < settle_deadline:
            self.publish(final_action, record=False)
            rate.sleep()
        print(f"Replay execution time: {time.perf_counter() - start_time:.3f}s")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Replay a dual-arm HDF5 action trajectory and measure ROS-level execution latency."
    )
    parser.add_argument("dataset_path", help="Path to an HDF5 episode")
    parser.add_argument("--field", default="action", help="HDF5 action field")
    parser.add_argument("--publish-rate", type=float, default=25.0)
    parser.add_argument("--max-steps", type=int, default=100000)
    parser.add_argument("--left-command-topic", default="/master/joint_left")
    parser.add_argument("--right-command-topic", default="/master/joint_right")
    parser.add_argument("--left-feedback-topic", default="/puppet/joint_left")
    parser.add_argument("--right-feedback-topic", default="/puppet/joint_right")
    parser.add_argument(
        "--start-step-sizes",
        type=float,
        nargs=7,
        default=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.2],
    )
    parser.add_argument("--skip-move-to-start", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Start replay without confirmation")
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--settle-tolerance", type=float, default=0.02)
    parser.add_argument("--settle-hold-seconds", type=float, default=0.2)
    parser.add_argument("--analysis-rate", type=float, default=200.0)
    parser.add_argument("--max-lag-ms", type=float, default=1000.0)
    parser.add_argument("--warmup-seconds", type=float, default=0.5)
    parser.add_argument("--report", default=None, help="Output JSON report path")
    return parser.parse_args()


def main():
    args = parse_arguments()
    actions = load_actions(args.dataset_path, args.field)
    rospy.init_node("reference_dual_arm_replay_latency", anonymous=True)
    node = DualArmReplayNode(args)
    node.replay(actions)
    report = analyze_latency_trace(
        node.trace,
        gripper_indices=[6, 13],
        analysis_rate_hz=args.analysis_rate,
        max_lag_ms=args.max_lag_ms,
        warmup_seconds=args.warmup_seconds,
        settle_tolerance=args.settle_tolerance,
        settle_hold_seconds=args.settle_hold_seconds,
    )
    report["replay"] = {
        "platform": "reference_dual_arm_ros1",
        "dataset_path": str(Path(args.dataset_path).resolve()),
        "dataset_field": args.field,
        "publish_rate_hz": float(args.publish_rate),
        "camera_subscriptions_enabled": False,
    }
    print_latency_report(report)
    report_path = args.report
    if report_path is None:
        report_path = f"{Path(args.dataset_path).stem}_reference_latency.json"
    save_latency_report(report, report_path)
    print(f"Saved latency report: {report_path}")

    if not rospy.is_shutdown():
        print("Moving both arms slowly back to the all-zero pose.")
        node.move_to_start(np.zeros(14, dtype=np.float64))


if __name__ == "__main__":
    main()

