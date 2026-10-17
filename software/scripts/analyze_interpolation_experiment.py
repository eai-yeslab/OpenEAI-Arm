"""Archive and summarize one interpolation replay experiment.

Typical usage from the software directory:

    python scripts/analyze_interpolation_experiment.py 0
    python scripts/analyze_interpolation_experiment.py --summary

The saved command trajectory is used to align the replay precisely against
q_ext in the motor log. Preparation motion and terminal holding are excluded.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
import yaml


SCRIPT_PATH = Path(__file__).resolve()
SOFTWARE_ROOT = SCRIPT_PATH.parent.parent
DEFAULT_CONFIG = SOFTWARE_ROOT / "configs" / "default_right.yml"
DEFAULT_TRAJECTORY_DIR = SOFTWARE_ROOT / "trajectories"
DEFAULT_OUTPUT_DIR = SOFTWARE_ROOT / "interpolation_experiments"
ARM_JOINTS = 6


def sanitize_name(value: object) -> str:
    text = str(value).strip().lower()
    text = re.sub(r"[^a-z0-9._-]+", "-", text)
    return text.strip("-._") or "unknown"


def nested_variant(config: dict) -> dict[str, str]:
    controller = config.get("controller", {}) or {}
    interpolation = controller.get("interpolation", {}) or {}
    if not isinstance(interpolation, dict):
        interpolation = {}

    method = interpolation.get("method", controller.get("method", "unknown"))

    def variant_value(name: str) -> str:
        value = interpolation.get(name, controller.get(name, {}))
        if isinstance(value, dict):
            value = value.get("variant", "unknown")
        return str(value if value is not None else "unknown")

    tame = variant_value("tame")
    swift = variant_value("swift")
    method_slug = sanitize_name(method)
    if method_slug == "tame_swift":
        slug = (
            f"{method_slug}__tame-{sanitize_name(tame)}"
            f"__swift-{sanitize_name(swift)}"
        )
    else:
        slug = method_slug
    return {
        "method": str(method),
        "tame_variant": tame,
        "swift_variant": swift,
        "variant_slug": slug,
    }


def resolve_log_path(explicit: Path | None) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Motor log not found: {path}")
        return path

    preferred = [
        Path("motor_state_log_right.csv"),
        Path("motor_state_log.csv"),
        SOFTWARE_ROOT / "motor_state_log_right.csv",
        SOFTWARE_ROOT / "motor_state_log.csv",
    ]
    for path in preferred:
        if path.is_file():
            return path.resolve()

    candidates = list(Path.cwd().glob("motor_state_log*.csv"))
    candidates.extend(SOFTWARE_ROOT.glob("motor_state_log*.csv"))
    unique = {path.resolve() for path in candidates if path.is_file()}
    if not unique:
        raise FileNotFoundError(
            "No motor_state_log*.csv found. Use --log to provide one."
        )
    return max(unique, key=lambda path: path.stat().st_mtime)


def resolve_trajectory_path(
    explicit: Path | None,
    trajectory_dir: Path,
    index: int,
) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Trajectory not found: {path}")
        return path

    candidates = [
        trajectory_dir / f"trajectory_{index}.hdf5",
        trajectory_dir / f"trajectory_{index:03d}.hdf5",
        trajectory_dir / f"trajectory_{index}.h5",
        trajectory_dir / f"trajectory_{index:03d}.h5",
    ]
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError(
        "Trajectory not found. Expected one of: "
        + ", ".join(str(path) for path in candidates)
        + ". Use --trajectory to override."
    )


def load_actions(
    path: Path,
    field: str,
    arm_index: int,
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
        raise ValueError(f"Expected [frames, 7 * arms] actions, got {actions.shape}")
    arm_count = actions.shape[1] // 7
    if arm_index < 0 or arm_index >= arm_count:
        raise ValueError(f"arm_index={arm_index} outside available range 0..{arm_count - 1}")
    if not math.isfinite(frame_rate) or frame_rate <= 0.0:
        raise ValueError(f"Invalid trajectory frame rate: {frame_rate}")
    actions = actions[:, arm_index * 7:(arm_index + 1) * 7]
    return actions, frame_rate


def actions_to_physical(actions: np.ndarray, config: dict) -> np.ndarray:
    motors = config.get("motors", [])
    if len(motors) < ARM_JOINTS:
        raise ValueError("Config must contain at least six motors")
    reset_pose = np.asarray(
        [float(motors[joint]["reset_pose"]) for joint in range(ARM_JOINTS)],
        dtype=np.float64,
    )
    return actions[:, :ARM_JOINTS] + reset_pose[None, :]


def required_log_columns(dataframe: pd.DataFrame) -> None:
    required = {"timestamp_ms"}
    for joint in range(ARM_JOINTS):
        required.update(
            {
                f"q_ext_{joint}",
                f"q_snd_{joint}",
                f"dq_snd_{joint}",
                f"q_{joint}",
                f"dq_{joint}",
            }
        )
    missing = sorted(required.difference(dataframe.columns))
    if missing:
        raise ValueError(f"Motor log is missing columns: {missing}")


def locate_replay(
    dataframe: pd.DataFrame,
    physical_actions: np.ndarray,
    frame_rate: float,
    candidate_tolerance: float,
) -> dict[str, float | int]:
    time = dataframe["timestamp_ms"].to_numpy(dtype=np.float64) / 1000.0
    external = dataframe[
        [f"q_ext_{joint}" for joint in range(ARM_JOINTS)]
    ].to_numpy(dtype=np.float64)
    if len(time) < 2 or np.any(np.diff(time) < 0.0):
        raise ValueError("Motor log timestamps must be ordered")

    first_error = np.max(np.abs(external - physical_actions[0]), axis=1)
    candidates = np.flatnonzero(first_error <= candidate_tolerance)
    if len(candidates) == 0:
        candidates = np.argsort(first_error)[: min(500, len(first_error))]
    elif len(candidates) > 5000:
        stride = int(math.ceil(len(candidates) / 5000.0))
        candidates = candidates[::stride]

    offsets = np.arange(len(physical_actions), dtype=np.float64) / frame_rate
    best = None
    for candidate_index in candidates:
        sample_times = time[candidate_index] + offsets
        if sample_times[-1] > time[-1]:
            continue
        sampled_indices = np.searchsorted(time, sample_times, side="right") - 1
        if sampled_indices[0] < 0:
            continue
        difference = external[sampled_indices] - physical_actions
        per_frame = np.sqrt(np.mean(np.square(difference), axis=1))
        score = float(np.sqrt(np.mean(np.square(difference))))
        p95 = float(np.percentile(per_frame, 95))
        result = {
            "score": score,
            "p95": p95,
            "candidate_index": int(candidate_index),
            "start_time_s": float(time[candidate_index]),
            "end_time_s": float(time[candidate_index] + len(physical_actions) / frame_rate),
        }
        if best is None or (score, p95) < (best["score"], best["p95"]):
            best = result

    if best is None:
        raise RuntimeError("No complete trajectory interval fits inside the motor log")
    return best


def crop_log(dataframe: pd.DataFrame, alignment: dict[str, float | int]) -> pd.DataFrame:
    time = dataframe["timestamp_ms"].to_numpy(dtype=np.float64) / 1000.0
    mask = (time >= alignment["start_time_s"]) & (time < alignment["end_time_s"])
    cropped = dataframe.loc[mask].copy()
    if cropped.empty:
        raise RuntimeError("Aligned replay interval produced an empty crop")
    cropped["trajectory_time_s"] = (
        cropped["timestamp_ms"] - cropped["timestamp_ms"].iloc[0]
    ) / 1000.0
    return cropped


def rms(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(np.square(finite)))) if len(finite) else math.nan


def percentile_abs(values: np.ndarray, percentile: float) -> float:
    finite = np.abs(values[np.isfinite(values)])
    return float(np.percentile(finite, percentile)) if len(finite) else math.nan


def best_lag_ms(
    source: np.ndarray,
    target: np.ndarray,
    sample_period_s: float,
    max_lag_s: float,
) -> float:
    if len(source) < 8 or len(target) != len(source):
        return math.nan
    source_signal = np.gradient(source, sample_period_s)
    target_signal = np.gradient(target, sample_period_s)
    source_signal -= np.mean(source_signal)
    target_signal -= np.mean(target_signal)
    if np.std(source_signal) < 1e-9 or np.std(target_signal) < 1e-9:
        return math.nan

    max_lag = min(int(round(max_lag_s / sample_period_s)), len(source) // 3)
    best_correlation = -math.inf
    best_lag = 0
    for lag in range(max_lag + 1):
        if lag == 0:
            left, right = source_signal, target_signal
        else:
            left, right = source_signal[:-lag], target_signal[lag:]
        if len(left) < 5:
            continue
        denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
        if denominator <= 1e-12:
            continue
        correlation = float(np.dot(left, right) / denominator)
        if correlation > best_correlation:
            best_correlation = correlation
            best_lag = lag
    return float(best_lag * sample_period_s * 1000.0)


def joint_metrics(
    dataframe: pd.DataFrame,
    variant: dict[str, str],
    trajectory_index: int,
    max_lag_s: float,
) -> list[dict[str, object]]:
    time = dataframe["timestamp_ms"].to_numpy(dtype=np.float64) / 1000.0
    dt = float(np.median(np.diff(time)))
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("Invalid motor log sample period")

    metrics = []
    for joint in range(ARM_JOINTS):
        external = dataframe[f"q_ext_{joint}"].to_numpy(dtype=np.float64)
        sent = dataframe[f"q_snd_{joint}"].to_numpy(dtype=np.float64)
        feedback = dataframe[f"q_{joint}"].to_numpy(dtype=np.float64)
        sent_velocity = dataframe[f"dq_snd_{joint}"].to_numpy(dtype=np.float64)
        feedback_velocity = dataframe[f"dq_{joint}"].to_numpy(dtype=np.float64)
        sent_acceleration = np.gradient(sent_velocity, dt)
        sent_jerk = np.gradient(sent_acceleration, dt)
        tracking_error = feedback - sent
        total_error = feedback - external
        planner_error = sent - external

        active = np.abs(sent_velocity) > 0.02
        active_duration = max(float(np.sum(active) * dt), dt)
        significant_error = tracking_error.copy()
        significant_error[np.abs(significant_error) < 0.001] = 0.0
        signs = np.sign(significant_error)
        nonzero_signs = signs[signs != 0.0]
        zero_crossings = int(np.sum(nonzero_signs[1:] != nonzero_signs[:-1]))

        row = {
            **variant,
            "trajectory_index": int(trajectory_index),
            "joint": int(joint),
            "duration_s": float(time[-1] - time[0]),
            "sample_count": int(len(dataframe)),
            "sample_rate_hz": float(1.0 / dt),
            "external_to_sent_lag_ms": best_lag_ms(
                external, sent, dt, max_lag_s
            ),
            "sent_to_feedback_lag_ms": best_lag_ms(
                sent, feedback, dt, max_lag_s
            ),
            "planner_error_rms_rad": rms(planner_error),
            "planner_error_p95_rad": percentile_abs(planner_error, 95),
            "tracking_error_rms_rad": rms(tracking_error),
            "tracking_error_mae_rad": float(np.mean(np.abs(tracking_error))),
            "tracking_error_p95_rad": percentile_abs(tracking_error, 95),
            "tracking_error_max_rad": percentile_abs(tracking_error, 100),
            "total_error_rms_rad": rms(total_error),
            "sent_velocity_rms_rad_s": rms(sent_velocity),
            "sent_velocity_p95_rad_s": percentile_abs(sent_velocity, 95),
            "sent_velocity_max_rad_s": percentile_abs(sent_velocity, 100),
            "feedback_velocity_rms_rad_s": rms(feedback_velocity),
            "sent_acceleration_rms_rad_s2": rms(sent_acceleration),
            "sent_acceleration_p95_rad_s2": percentile_abs(sent_acceleration, 95),
            "sent_acceleration_max_rad_s2": percentile_abs(sent_acceleration, 100),
            "sent_jerk_rms_rad_s3": rms(sent_jerk),
            "sent_jerk_p95_rad_s3": percentile_abs(sent_jerk, 95),
            "sent_jerk_max_rad_s3": percentile_abs(sent_jerk, 100),
            "tracking_error_zero_crossings_hz": float(
                zero_crossings / active_duration
            ),
        }
        metrics.append(row)
    return metrics


def mean_numeric(rows: list[dict[str, object]]) -> dict[str, float]:
    result = {}
    if not rows:
        return result
    excluded = {
        "trajectory_index",
        "joint",
        "sample_count",
        "method",
        "tame_variant",
        "swift_variant",
        "variant_slug",
    }
    for key in rows[0]:
        if key in excluded:
            continue
        values = np.asarray([row.get(key, math.nan) for row in rows], dtype=float)
        finite = values[np.isfinite(values)]
        if len(finite):
            result[key] = float(np.mean(finite))
    return result


def copy_if_different(source: Path, destination: Path) -> None:
    source = source.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and source.samefile(destination):
        return
    shutil.copy2(source, destination)


def json_compatible(value):
    if isinstance(value, dict):
        return {key: json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def process_run(args: argparse.Namespace) -> None:
    if args.trajectory_index is None:
        raise ValueError("trajectory_index is required unless --summary is used")
    if args.trajectory_index < 0:
        raise ValueError("trajectory_index must be non-negative")

    config_path = args.config.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Config not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    variant = nested_variant(config)

    trajectory_path = resolve_trajectory_path(
        args.trajectory,
        args.trajectory_dir,
        args.trajectory_index,
    )
    log_path = resolve_log_path(args.log)
    actions, frame_rate = load_actions(
        trajectory_path,
        args.field,
        args.arm_index,
        args.frame_rate,
    )
    physical_actions = actions_to_physical(actions, config)

    dataframe = pd.read_csv(log_path)
    required_log_columns(dataframe)
    alignment = locate_replay(
        dataframe,
        physical_actions,
        frame_rate,
        args.candidate_tolerance,
    )
    if alignment["score"] > args.max_alignment_rms and not args.allow_poor_alignment:
        raise RuntimeError(
            f"Trajectory alignment RMS {alignment['score']:.6f} rad exceeds "
            f"limit {args.max_alignment_rms:.6f}. Check the trajectory, log, "
            "arm index, config reset poses, or use --allow-poor-alignment."
        )

    cropped = crop_log(dataframe, alignment)
    metrics = joint_metrics(
        cropped,
        variant,
        args.trajectory_index,
        args.max_lag_seconds,
    )
    for row in metrics:
        row["alignment_rms_rad"] = float(alignment["score"])
        row["alignment_p95_rad"] = float(alignment["p95"])
        row["command_frame_rate_hz"] = float(frame_rate)

    run_dir = (
        args.output_dir
        / variant["variant_slug"]
        / f"trajectory_{args.trajectory_index:03d}"
    ).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    cropped.to_csv(run_dir / "motor_state_log.csv", index=False)
    pd.DataFrame(metrics).to_csv(run_dir / "joint_metrics.csv", index=False)
    copy_if_different(trajectory_path, run_dir / "trajectory.hdf5")
    copy_if_different(config_path, run_dir / "config.yml")

    summary = {
        "schema_version": 1,
        "variant": variant,
        "trajectory_index": int(args.trajectory_index),
        "source": {
            "motor_log": str(log_path),
            "trajectory": str(trajectory_path),
            "config": str(config_path),
            "field": args.field,
            "arm_index": int(args.arm_index),
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
        "aggregate_metrics": mean_numeric(metrics),
        "joint_metrics": metrics,
    }
    with (run_dir / "run_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(json_compatible(summary), handle, indent=2, allow_nan=False)

    print(f"Variant: {variant['variant_slug']}")
    print(f"Trajectory: {trajectory_path}")
    print(f"Motor log: {log_path}")
    print(
        f"Aligned {alignment['start_time_s']:.3f}s to "
        f"{alignment['end_time_s']:.3f}s, RMS={alignment['score']:.6f} rad, "
        f"rows={len(cropped)}"
    )
    print(f"Saved run archive: {run_dir}")


def summarize_results(output_dir: Path, summary_output: Path | None) -> None:
    summaries = []
    for path in sorted(output_dir.glob("*/trajectory_*/run_summary.json")):
        with path.open("r", encoding="utf-8") as handle:
            summaries.append(json.load(handle))
    if not summaries:
        raise FileNotFoundError(f"No run_summary.json files found under {output_dir}")

    detail_rows = []
    for summary in summaries:
        detail_rows.extend(summary.get("joint_metrics", []))
    detail = pd.DataFrame(detail_rows)
    if detail.empty:
        raise RuntimeError("Run summaries contain no joint metrics")

    identity = [
        "variant_slug",
        "method",
        "tame_variant",
        "swift_variant",
        "joint",
    ]
    excluded_numeric = {"trajectory_index", "sample_count"}
    numeric = [
        column
        for column in detail.select_dtypes(include=[np.number]).columns
        if column not in excluded_numeric and column not in identity
    ]

    rows = []
    for group_key, group in detail.groupby(identity, dropna=False, sort=True):
        row = dict(zip(identity, group_key))
        row["trajectory_count"] = int(group["trajectory_index"].nunique())
        for column in numeric:
            values = group[column].to_numpy(dtype=float)
            values = values[np.isfinite(values)]
            if len(values) == 0:
                row[f"{column}_mean"] = math.nan
                row[f"{column}_std"] = math.nan
                row[f"{column}_median"] = math.nan
                row[f"{column}_p95"] = math.nan
                continue
            row[f"{column}_mean"] = float(np.mean(values))
            row[f"{column}_std"] = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
            row[f"{column}_median"] = float(np.median(values))
            row[f"{column}_p95"] = float(np.percentile(values, 95))
        rows.append(row)

    result = pd.DataFrame(rows)
    destination = summary_output or (output_dir / "experiment_summary.csv")
    destination = destination.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    result.to_csv(destination, index=False)
    print(
        f"Saved {len(result)} variant/joint summary rows from "
        f"{len(summaries)} runs to {destination}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Precisely crop and summarize one interpolation replay, or aggregate "
            "all archived experiment variants."
        )
    )
    parser.add_argument("trajectory_index", nargs="?", type=int)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--trajectory", type=Path, default=None)
    parser.add_argument("--trajectory-dir", type=Path, default=DEFAULT_TRAJECTORY_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--summary-output", type=Path, default=None)
    parser.add_argument("--field", default="action")
    parser.add_argument("--arm-index", type=int, default=0)
    parser.add_argument(
        "--frame-rate",
        type=float,
        default=50.0,
        help="Fallback only when the HDF5 action dataset has no frame_rate attribute.",
    )
    parser.add_argument("--candidate-tolerance", type=float, default=0.01)
    parser.add_argument("--max-alignment-rms", type=float, default=0.05)
    parser.add_argument("--allow-poor-alignment", action="store_true")
    parser.add_argument("--max-lag-seconds", type=float, default=0.8)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    if args.summary:
        summarize_results(args.output_dir, args.summary_output)
    else:
        process_run(args)


if __name__ == "__main__":
    main()
