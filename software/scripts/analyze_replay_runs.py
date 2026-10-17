import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def detect_replay_segments(
    dataframe: pd.DataFrame,
    gap_seconds: float,
    min_duration_seconds: float,
) -> list[tuple[int, int]]:
    time = dataframe["timestamp_ms"].to_numpy(dtype=float) / 1000.0
    external = dataframe[[f"q_ext_{joint}" for joint in range(7)]].to_numpy()
    changed = np.r_[
        True,
        np.max(np.abs(np.diff(external, axis=0)), axis=1) > 1e-8,
    ]
    changed_indices = np.flatnonzero(changed)
    changed_times = time[changed_indices]
    boundaries = np.flatnonzero(np.diff(changed_times) > gap_seconds)
    starts = np.r_[0, boundaries + 1]
    ends = np.r_[boundaries, len(changed_indices) - 1]

    segments = []
    for start, end in zip(starts, ends):
        first_index = int(changed_indices[start])
        last_index = int(changed_indices[end])
        if time[last_index] - time[first_index] >= min_duration_seconds:
            segments.append((first_index, last_index))
    return segments


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def plot_run(
    dataframe: pd.DataFrame,
    segment: tuple[int, int],
    run_number: int,
    joints: list[int],
    output_path: Path,
) -> None:
    first_index, last_index = segment
    run = dataframe.iloc[first_index:last_index + 1].copy()
    time = run["timestamp_ms"].to_numpy(dtype=float) / 1000.0
    time -= time[0]

    figure, axes = plt.subplots(
        len(joints),
        3,
        figsize=(16, 3.2 * len(joints)),
        sharex=True,
        squeeze=False,
    )

    for row, joint in enumerate(joints):
        external = run[f"q_ext_{joint}"].to_numpy()
        sent = run[f"q_snd_{joint}"].to_numpy()
        feedback = run[f"q_{joint}"].to_numpy()
        sent_velocity = run[f"dq_snd_{joint}"].to_numpy()
        feedback_velocity = run[f"dq_{joint}"].to_numpy()

        axes[row, 0].plot(time, external, color="tab:cyan", label="External target")
        axes[row, 0].plot(time, sent, color="goldenrod", label="Sent position")
        axes[row, 0].plot(time, feedback, color="tab:red", label="Feedback position")
        axes[row, 0].set_ylabel(f"J{joint} position (rad)")

        axes[row, 1].plot(time, sent_velocity, color="goldenrod", label="Sent velocity")
        axes[row, 1].plot(time, feedback_velocity, color="tab:red", label="Feedback velocity")
        axes[row, 1].set_ylabel(f"J{joint} velocity (rad/s)")

        axes[row, 2].plot(time, external - sent, color="tab:blue", label="Target - sent")
        axes[row, 2].plot(time, feedback - sent, color="tab:red", label="Feedback - sent")
        axes[row, 2].axhline(0.0, color="black", linewidth=0.7)
        axes[row, 2].set_ylabel(f"J{joint} error (rad)")

        print(
            f"run={run_number} joint={joint} "
            f"planner_rms={rms(external - sent):.6f} "
            f"tracking_rms={rms(feedback - sent):.6f} "
            f"tracking_max={np.max(np.abs(feedback - sent)):.6f}"
        )

        for column in range(3):
            axes[row, column].grid(True, alpha=0.3)
            axes[row, column].legend(loc="best")

    for axis in axes[-1, :]:
        axis.set_xlabel("Time (s)")

    figure.suptitle(f"Replay run {run_number}")
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Detect and plot individual replay runs from a motor state log."
    )
    parser.add_argument("filename", type=Path)
    parser.add_argument("--runs", type=int, nargs="+", default=None)
    parser.add_argument("--joints", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--output-dir", type=Path, default=Path("replay_analysis"))
    parser.add_argument("--gap-seconds", type=float, default=0.1)
    parser.add_argument("--min-duration-seconds", type=float, default=5.0)
    args = parser.parse_args()

    dataframe = pd.read_csv(args.filename)
    segments = detect_replay_segments(
        dataframe,
        gap_seconds=args.gap_seconds,
        min_duration_seconds=args.min_duration_seconds,
    )
    print(f"Detected {len(segments)} replay runs")

    selected_runs = args.runs or list(range(1, len(segments) + 1))
    for run_number in selected_runs:
        if run_number < 1 or run_number > len(segments):
            raise ValueError(
                f"Run {run_number} is outside the detected range 1..{len(segments)}"
            )
        first_index, last_index = segments[run_number - 1]
        start_time = dataframe.iloc[first_index]["timestamp_ms"] / 1000.0
        end_time = dataframe.iloc[last_index]["timestamp_ms"] / 1000.0
        print(
            f"Run {run_number}: {start_time:.3f}s to {end_time:.3f}s, "
            f"duration={end_time - start_time:.3f}s"
        )
        plot_run(
            dataframe,
            segments[run_number - 1],
            run_number,
            args.joints,
            args.output_dir / f"replay_run_{run_number}.png",
        )


if __name__ == "__main__":
    main()
