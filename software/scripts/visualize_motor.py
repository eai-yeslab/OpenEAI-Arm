import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description="Plot motor state log data.")
    parser.add_argument("filename", nargs="?", default="motor_state_log.csv")
    parser.add_argument("--legend", action="store_true")
    args = parser.parse_args()

    data = pd.read_csv(args.filename)
    time = data["timestamp_ms"] / 1000.0
    keyframe_mask = (
        data["keyframe"] == 1
        if "keyframe" in data.columns
        else np.zeros(len(data), dtype=bool)
    )
    num_joints = 7
    figure, axes = plt.subplots(
        num_joints, 3, figsize=(14, 2.8 * num_joints), sharex=True
    )

    for joint in range(num_joints):
        position_axis = axes[joint, 0]
        command_column = f"q_cmd_{joint}"
        if command_column in data.columns:
            position_axis.plot(time, data[command_column], "b", label="Planned position")
            if keyframe_mask.any():
                position_axis.scatter(
                    time[keyframe_mask],
                    data[command_column][keyframe_mask],
                    color="b",
                    marker="o",
                    s=20,
                    label="Keyframe",
                )
        external_column = f"q_ext_{joint}"
        if external_column in data.columns:
            position_axis.plot(
                time, data[external_column], "c", label="External target"
            )
        sent_column = f"q_snd_{joint}"
        if sent_column in data.columns:
            position_axis.plot(time, data[sent_column], "y", label="Sent position")
        feedback_column = f"q_{joint}"
        if feedback_column in data.columns:
            position_axis.plot(time, data[feedback_column], "r", label="Feedback position")
        position_axis.set_ylabel("Position (rad)")
        position_axis.grid()

        if command_column in data.columns and feedback_column in data.columns:
            error = data[feedback_column] - data[command_column]
            error_axis = position_axis.twinx()
            error_axis.plot(time, error, "g", label="Position error")
            error_axis.set_ylabel("Error (rad)")
            if args.legend:
                error_axis.legend(loc="upper right")
            p01, p99 = np.nanpercentile(error, [1, 99])
            print(
                f"Joint {joint}: feedback min={data[feedback_column].min():.4f}, "
                f"max={data[feedback_column].max():.4f}, "
                f"error p01={p01:.4f}, p99={p99:.4f} rad"
            )

        velocity_axis = axes[joint, 1]
        for column, label, color in (
            (f"dq_cmd_{joint}", "Planned velocity", "b"),
            (f"dq_snd_{joint}", "Sent velocity", "y"),
            (f"dq_{joint}", "Feedback velocity", "r"),
        ):
            if column in data.columns:
                velocity_axis.plot(time, data[column], color, label=label)
        velocity_axis.set_ylabel("Velocity (rad/s)")
        velocity_axis.grid()

        torque_axis = axes[joint, 2]
        for column, label, color in (
            (f"tau_cmd_{joint}", "Command torque", "b"),
            (f"tau_{joint}", "Feedback torque", "r"),
        ):
            if column in data.columns:
                torque_axis.plot(time, data[column], color, label=label)
        torque_axis.set_ylabel("Torque (Nm)")
        torque_axis.grid()

        if args.legend:
            position_axis.legend(loc="upper left")
            velocity_axis.legend(loc="upper left")
            torque_axis.legend(loc="upper left")

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    axes[-1, 2].set_xlabel("Time (s)")
    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
