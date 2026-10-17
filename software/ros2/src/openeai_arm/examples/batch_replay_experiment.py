"""Replay every HDF5 trajectory in a directory and analyze one combined log.

The script moves continuously between trajectory endpoints, records motor-log
boundaries around each actual replay, and then uses full-sequence matching to
exclude transition, preparation, terminal hold, and final return motions.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import time
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
import yaml


SCRIPT_PATH = Path(__file__).resolve()
EXAMPLES_DIR = SCRIPT_PATH.parent
SOFTWARE_ROOT = EXAMPLES_DIR.parents[3]
DEFAULT_ROS_CONFIG = EXAMPLES_DIR / "config_right_replay.yml"
DEFAULT_CONTROLLER_CONFIG = SOFTWARE_ROOT / "configs" / "default_right.yml"
DEFAULT_OUTPUT_DIR = SOFTWARE_ROOT / "interpolation_experiments"
TRAJECTORY_SUFFIXES = {".h5", ".hdf5"}
ARM_JOINTS = 6


def load_analysis_module():
    candidates = [
        EXAMPLES_DIR / "analyze_interpolation_experiment.py",
        SOFTWARE_ROOT / "scripts" / "analyze_interpolation_experiment.py",
    ]
    for path in candidates:
        if not path.is_file():
            continue
        spec = importlib.util.spec_from_file_location(
            "analyze_interpolation_experiment",
            path,
        )
        if spec is None or spec.loader is None:
            continue
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    raise FileNotFoundError(
        "analyze_interpolation_experiment.py was not found beside this script "
        "or under software/scripts."
    )


def natural_sort_key(path: Path) -> list[object]:
    parts = re.split(r"(\d+)", path.name.lower())
    return [int(part) if part.isdigit() else part for part in parts]


def discover_trajectories(directory: Path) -> list[Path]:
    directory = directory.expanduser().resolve()
    if not directory.is_dir():
        raise NotADirectoryError(f"Trajectory directory not found: {directory}")
    trajectories = sorted(
        (
            path.resolve()
            for path in directory.iterdir()
            if path.is_file() and path.suffix.lower() in TRAJECTORY_SUFFIXES
        ),
        key=natural_sort_key,
    )
    if not trajectories:
        raise FileNotFoundError(f"No HDF5 trajectories found in {directory}")
    return trajectories


def trajectory_index(path: Path, fallback_index: int) -> int:
    match = re.search(r"(\d+)(?!.*\d)", path.stem)
    return int(match.group(1)) if match else int(fallback_index)


def load_full_actions(
    path: Path,
    field: str,
    fallback_frame_rate: float,
) -> tuple[np.ndarray, float]:
    with h5py.File(path, "r") as handle:
        if field not in handle:
            raise KeyError(
                f"Dataset '{field}' not found in {path}; keys={list(handle.keys())}"
            )
        dataset = handle[field]
        actions = np.asarray(dataset, dtype=np.float64)
        frame_rate = float(
            dataset.attrs.get(
                "frame_rate",
                handle.attrs.get("frame_rate", fallback_frame_rate),
            )
        )
    if actions.ndim != 2 or actions.shape[1] < 7 or actions.shape[1] % 7 != 0:
        raise ValueError(
            f"Expected [frames, 7 * arms] actions in {path}, got {actions.shape}"
        )
    if len(actions) == 0:
        raise ValueError(f"Trajectory contains no action frames: {path}")
    if not math.isfinite(frame_rate) or frame_rate <= 0.0:
        raise ValueError(f"Invalid frame rate {frame_rate} in {path}")
    return actions, frame_rate


def resolve_log_path(explicit: Path | None) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Motor log not found: {path}")
        return path

    search_directories = [Path.cwd(), SOFTWARE_ROOT, EXAMPLES_DIR]
    candidates = {
        path.resolve()
        for directory in search_directories
        for path in directory.glob("motor_state_log*.csv")
        if path.is_file()
    }
    if not candidates:
        raise FileNotFoundError(
            "No motor_state_log*.csv was found. Use --log to provide one."
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_last_log_timestamp_ms(path: Path) -> float | None:
    """Read the newest complete CSV timestamp without loading the whole log."""
    try:
        with path.open("rb") as handle:
            handle.seek(0, 2)
            size = handle.tell()
            if size == 0:
                return None
            handle.seek(max(0, size - 65536))
            lines = handle.read().splitlines()
    except OSError:
        return None

    for line in reversed(lines):
        try:
            first_field = line.decode("utf-8").split(",", 1)[0].strip()
            return float(first_field)
        except (UnicodeDecodeError, ValueError):
            continue
    return None


def wait_for_arm_feedback(operator, arm_keys: list[str]) -> list[float]:
    rate = operator.node.create_rate(float(operator.config.get("frame_rate", 50.0)))
    while True:
        missing = [key for key in arm_keys if not operator.arm_deques[key]]
        if not missing:
            break
        operator.node.get_logger().info(
            f"Waiting for arm feedback: {', '.join(missing)}"
        )
        rate.sleep()
    pose = []
    for key in arm_keys:
        pose.extend(list(operator.arm_deques[key][-1].position[:7]))
    return pose


def adapt_actions_to_target(
    actions: np.ndarray,
    target_arm_count: int,
    stationary_pose: list[float],
) -> np.ndarray:
    source_arm_count = actions.shape[1] // 7
    if source_arm_count >= target_arm_count:
        return actions[:, : target_arm_count * 7].copy()

    missing_start = source_arm_count * 7
    stationary = np.asarray(stationary_pose[missing_start:], dtype=np.float64)
    expected = (target_arm_count - source_arm_count) * 7
    if len(stationary) != expected:
        raise ValueError(
            f"Expected {expected} stationary dimensions, got {len(stationary)}"
        )
    repeated = np.tile(stationary, (len(actions), 1))
    return np.concatenate([actions, repeated], axis=1)


def hold_action(operator, action: np.ndarray, seconds: float, frame_rate: float) -> None:
    if seconds <= 0.0:
        return
    rate = operator.node.create_rate(frame_rate)
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        operator.puppet_arm_publish("puppet", action.tolist())
        rate.sleep()


def publish_command_transition(
    operator,
    start_action: np.ndarray,
    target_action: np.ndarray,
) -> None:
    """Move from the previous command, preserving command-level continuity."""
    current = np.asarray(start_action, dtype=np.float64).copy()
    target = np.asarray(target_action, dtype=np.float64)
    if current.shape != target.shape:
        raise ValueError(
            f"Transition shape mismatch: {current.shape} != {target.shape}"
        )
    per_arm_steps = np.asarray(
        operator.config["arm_steps_length"],
        dtype=np.float64,
    )
    if per_arm_steps.shape != (7,) or np.any(per_arm_steps <= 0.0):
        raise ValueError("arm_steps_length must contain seven positive values")
    steps = np.tile(per_arm_steps, len(current) // 7)
    rate = operator.node.create_rate(
        float(operator.config.get("frame_rate", 50.0))
    )
    while True:
        difference = target - current
        if np.all(np.abs(difference) <= steps):
            operator.puppet_arm_publish("puppet", target.tolist())
            rate.sleep()
            return
        current += np.clip(difference, -steps, steps)
        operator.puppet_arm_publish("puppet", current.tolist())
        rate.sleep()


def run_batch(args, trajectories: list[Path], log_path: Path) -> dict:
    import rclpy

    from OpenEAIArmROS2Operator import OpenEAIArmROS2Operator

    operator = OpenEAIArmROS2Operator(str(args.ros_config))
    puppet_keys = [key for key in operator.arm_keys if "puppet" in key]
    if not puppet_keys:
        raise RuntimeError("No puppet arm is configured")

    fallback_rate = float(operator.config.get("frame_rate", args.frame_rate))
    loaded = [
        (path, *load_full_actions(path, args.field, fallback_rate))
        for path in trajectories
    ]
    manifest = {
        "schema_version": 1,
        "trajectory_directory": str(args.trajectory_directory.resolve()),
        "ros_config": str(args.ros_config.resolve()),
        "controller_config": str(args.controller_config.resolve()),
        "motor_log": str(log_path.resolve()),
        "batch_log_start_ms": read_last_log_timestamp_ms(log_path),
        "trajectories": [],
    }

    print("Replay order:")
    for sequence, (path, actions, frame_rate) in enumerate(loaded):
        print(
            f"  {sequence}: {path.name}, frames={len(actions)}, "
            f"rate={frame_rate:g} Hz"
        )
    if not args.yes:
        input("Press Enter to start the complete batch replay: ")

    completed_normally = False
    last_command = None
    try:
        for sequence, (path, source_actions, frame_rate) in enumerate(loaded):
            current_pose = wait_for_arm_feedback(operator, puppet_keys)
            actions = adapt_actions_to_target(
                source_actions,
                len(puppet_keys),
                current_pose,
            )

            print(
                f"[{sequence + 1}/{len(loaded)}] Moving continuously to "
                f"the first frame of {path.name}."
            )
            if last_command is None:
                operator.puppet_arm_publish_continuous(
                    "puppet",
                    actions[0].tolist(),
                )
            else:
                publish_command_transition(
                    operator,
                    last_command,
                    actions[0],
                )
            hold_action(
                operator,
                actions[0],
                args.transition_settle_seconds,
                frame_rate,
            )
            start_log_ms = read_last_log_timestamp_ms(log_path)

            print(
                f"[{sequence + 1}/{len(loaded)}] Replaying {path.name} "
                f"at {frame_rate:g} Hz."
            )
            rate = operator.node.create_rate(frame_rate)
            start_wall_time = time.time()
            for action in actions:
                if not rclpy.ok():
                    raise KeyboardInterrupt
                operator.puppet_arm_publish("puppet", action.tolist())
                rate.sleep()
            end_wall_time = time.time()

            hold_action(
                operator,
                actions[-1],
                args.terminal_hold_seconds,
                frame_rate,
            )
            end_log_ms = read_last_log_timestamp_ms(log_path)
            manifest["trajectories"].append(
                {
                    "sequence": int(sequence),
                    "trajectory_index": trajectory_index(path, sequence),
                    "path": str(path),
                    "frame_count": int(len(source_actions)),
                    "action_dimension": int(source_actions.shape[1]),
                    "frame_rate_hz": float(frame_rate),
                    "start_log_timestamp_ms": start_log_ms,
                    "end_log_timestamp_ms": end_log_ms,
                    "start_wall_time_s": float(start_wall_time),
                    "end_wall_time_s": float(end_wall_time),
                }
            )
            last_command = actions[-1].copy()
        completed_normally = True
    finally:
        manifest["batch_log_end_ms"] = read_last_log_timestamp_ms(log_path)
        if completed_normally and not args.keep_final_pose and rclpy.ok():
            print("Moving continuously back to the all-zero control pose.")
            publish_command_transition(
                operator,
                last_command,
                np.zeros(len(puppet_keys) * 7, dtype=np.float64),
            )
        if operator.node is not None:
            operator.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return manifest


def locate_replay_in_window(
    dataframe: pd.DataFrame,
    physical_actions: np.ndarray,
    frame_rate: float,
    candidate_tolerance: float,
    minimum_start_s: float,
    maximum_end_s: float | None,
) -> dict[str, float | int]:
    time_axis = dataframe["timestamp_ms"].to_numpy(dtype=np.float64) / 1000.0
    external = dataframe[
        [f"q_ext_{joint}" for joint in range(ARM_JOINTS)]
    ].to_numpy(dtype=np.float64)
    duration_s = len(physical_actions) / frame_rate

    allowed = time_axis >= minimum_start_s
    if maximum_end_s is not None:
        allowed &= time_axis + duration_s <= maximum_end_s
    allowed_indices = np.flatnonzero(allowed)
    if len(allowed_indices) == 0:
        raise RuntimeError("No log samples remain inside the trajectory window")

    first_error = np.max(
        np.abs(external[allowed_indices] - physical_actions[0]),
        axis=1,
    )
    candidates = allowed_indices[first_error <= candidate_tolerance]
    if len(candidates) == 0:
        order = np.argsort(first_error)[: min(500, len(first_error))]
        candidates = allowed_indices[order]
    elif len(candidates) > 5000:
        stride = int(math.ceil(len(candidates) / 5000.0))
        candidates = candidates[::stride]

    offsets = np.arange(len(physical_actions), dtype=np.float64) / frame_rate
    best = None
    for candidate_index in candidates:
        sample_times = time_axis[candidate_index] + offsets
        if sample_times[-1] > time_axis[-1]:
            continue
        sampled_indices = np.searchsorted(time_axis, sample_times, side="right") - 1
        difference = external[sampled_indices] - physical_actions
        per_frame = np.sqrt(np.mean(np.square(difference), axis=1))
        score = float(np.sqrt(np.mean(np.square(difference))))
        p95 = float(np.percentile(per_frame, 95))
        result = {
            "score": score,
            "p95": p95,
            "candidate_index": int(candidate_index),
            "start_time_s": float(time_axis[candidate_index]),
            "end_time_s": float(time_axis[candidate_index] + duration_s),
        }
        if best is None or (score, p95) < (best["score"], best["p95"]):
            best = result
    if best is None:
        raise RuntimeError("No complete trajectory fits inside its log window")
    return best


def save_batch_manifest(manifest: dict, destination: Path, analysis) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as handle:
        json.dump(
            analysis.json_compatible(manifest),
            handle,
            indent=2,
            allow_nan=False,
        )


def analyze_batch(args, manifest: dict, log_path: Path, analysis) -> None:
    with args.controller_config.open("r", encoding="utf-8") as handle:
        controller_config = yaml.safe_load(handle) or {}
    variant = analysis.nested_variant(controller_config)
    dataframe = pd.read_csv(log_path)
    analysis.required_log_columns(dataframe)

    variant_dir = args.output_dir / variant["variant_slug"]
    variant_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = variant_dir / "batch_manifest.json"
    manifest["variant"] = variant
    manifest["analysis_motor_log"] = str(log_path)
    save_batch_manifest(manifest, manifest_path, analysis)

    next_minimum_start_s = float(manifest.get("batch_log_start_ms") or 0.0) / 1000.0
    used_indices = set()
    for entry in manifest["trajectories"]:
        path = Path(entry["path"])
        index = int(entry["trajectory_index"])
        if index in used_indices:
            raise ValueError(
                f"Duplicate trajectory index {index}; rename files so their final "
                "numeric identifiers are unique."
            )
        used_indices.add(index)

        actions, frame_rate = analysis.load_actions(
            path,
            args.field,
            args.arm_index,
            args.frame_rate,
        )
        physical_actions = analysis.actions_to_physical(
            actions,
            controller_config,
        )
        start_boundary = entry.get("start_log_timestamp_ms")
        end_boundary = entry.get("end_log_timestamp_ms")
        minimum_start_s = next_minimum_start_s
        if start_boundary is not None:
            minimum_start_s = max(
                minimum_start_s,
                float(start_boundary) / 1000.0,
            )
        maximum_end_s = None
        if end_boundary is not None:
            maximum_end_s = (
                float(end_boundary) / 1000.0 + args.boundary_slack_seconds
            )

        alignment = locate_replay_in_window(
            dataframe,
            physical_actions,
            frame_rate,
            args.candidate_tolerance,
            minimum_start_s,
            maximum_end_s,
        )
        if alignment["score"] > args.max_alignment_rms and not args.allow_poor_alignment:
            raise RuntimeError(
                f"Trajectory {index} alignment RMS {alignment['score']:.6f} rad "
                f"exceeds limit {args.max_alignment_rms:.6f}."
            )
        next_minimum_start_s = alignment["end_time_s"]

        cropped = analysis.crop_log(dataframe, alignment)
        entry["alignment"] = alignment
        entry["cropped_row_count"] = int(len(cropped))
        if start_boundary is not None:
            entry["excluded_before_replay_s"] = max(
                0.0,
                alignment["start_time_s"] - float(start_boundary) / 1000.0,
            )
        if end_boundary is not None:
            entry["excluded_after_replay_s"] = max(
                0.0,
                float(end_boundary) / 1000.0 - alignment["end_time_s"],
            )
        metrics = analysis.joint_metrics(
            cropped,
            variant,
            index,
            args.max_lag_seconds,
        )
        for row in metrics:
            row["alignment_rms_rad"] = float(alignment["score"])
            row["alignment_p95_rad"] = float(alignment["p95"])
            row["command_frame_rate_hz"] = float(frame_rate)

        run_dir = variant_dir / f"trajectory_{index:03d}"
        run_dir.mkdir(parents=True, exist_ok=True)
        cropped.to_csv(run_dir / "motor_state_log.csv", index=False)
        pd.DataFrame(metrics).to_csv(run_dir / "joint_metrics.csv", index=False)
        analysis.copy_if_different(path, run_dir / "trajectory.hdf5")
        analysis.copy_if_different(
            args.controller_config,
            run_dir / "config.yml",
        )

        summary = {
            "schema_version": 1,
            "variant": variant,
            "trajectory_index": index,
            "batch_sequence": int(entry["sequence"]),
            "source": {
                "motor_log": str(log_path),
                "trajectory": str(path),
                "config": str(args.controller_config),
                "field": args.field,
                "arm_index": int(args.arm_index),
                "batch_manifest": str(manifest_path),
            },
            "trajectory": {
                "frame_count": int(len(actions)),
                "dimension": int(actions.shape[1]),
                "frame_rate_hz": float(frame_rate),
            },
            "alignment": alignment,
            "crop": {
                "row_count": int(len(cropped)),
                "duration_s": float(
                    cropped["trajectory_time_s"].iloc[-1]
                    - cropped["trajectory_time_s"].iloc[0]
                ),
            },
            "aggregate_metrics": analysis.mean_numeric(metrics),
            "joint_metrics": metrics,
        }
        with (run_dir / "run_summary.json").open("w", encoding="utf-8") as handle:
            json.dump(
                analysis.json_compatible(summary),
                handle,
                indent=2,
                allow_nan=False,
            )
        print(
            f"Analyzed trajectory {index}: {alignment['start_time_s']:.3f}s "
            f"to {alignment['end_time_s']:.3f}s, "
            f"RMS={alignment['score']:.6f} rad."
        )

    save_batch_manifest(manifest, manifest_path, analysis)
    analysis.summarize_results(args.output_dir, None)
    print(f"Saved batch manifest: {manifest_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay all trajectories in one directory, split one combined motor "
            "log, and produce per-trajectory and aggregate experiment results."
        )
    )
    parser.add_argument("trajectory_directory", type=Path)
    parser.add_argument("--ros-config", type=Path, default=DEFAULT_ROS_CONFIG)
    parser.add_argument(
        "--controller-config",
        type=Path,
        default=DEFAULT_CONTROLLER_CONFIG,
    )
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--field", default="action")
    parser.add_argument("--arm-index", type=int, default=0)
    parser.add_argument("--frame-rate", type=float, default=50.0)
    parser.add_argument("--transition-settle-seconds", type=float, default=0.5)
    parser.add_argument("--terminal-hold-seconds", type=float, default=0.5)
    parser.add_argument("--boundary-slack-seconds", type=float, default=0.1)
    parser.add_argument("--candidate-tolerance", type=float, default=0.01)
    parser.add_argument("--max-alignment-rms", type=float, default=0.05)
    parser.add_argument("--allow-poor-alignment", action="store_true")
    parser.add_argument("--max-lag-seconds", type=float, default=0.8)
    parser.add_argument("--keep-final-pose", action="store_true")
    parser.add_argument("--yes", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.trajectory_directory = args.trajectory_directory.expanduser().resolve()
    args.ros_config = args.ros_config.expanduser().resolve()
    args.controller_config = args.controller_config.expanduser().resolve()
    args.output_dir = args.output_dir.expanduser().resolve()
    if not args.ros_config.is_file():
        raise FileNotFoundError(f"ROS replay config not found: {args.ros_config}")
    if not args.controller_config.is_file():
        raise FileNotFoundError(
            f"Controller config not found: {args.controller_config}"
        )

    trajectories = discover_trajectories(args.trajectory_directory)
    log_path = resolve_log_path(args.log)
    analysis = load_analysis_module()
    manifest = run_batch(args, trajectories, log_path)
    time.sleep(0.5)
    analyze_batch(args, manifest, log_path, analysis)


if __name__ == "__main__":
    main()

