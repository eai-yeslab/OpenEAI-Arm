import json
import threading
import time
from pathlib import Path

import numpy as np


class LatencyTrace:
    """Thread-safe command and feedback recorder using one monotonic clock."""

    def __init__(self):
        self._command_times = []
        self._command_positions = []
        self._feedback_times = []
        self._feedback_positions = []
        self._lock = threading.Lock()

    def record_command(self, position, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else float(timestamp)
        with self._lock:
            self._command_times.append(timestamp)
            self._command_positions.append(np.asarray(position, dtype=np.float64).copy())

    def record_feedback(self, position, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else float(timestamp)
        with self._lock:
            self._feedback_times.append(timestamp)
            self._feedback_positions.append(np.asarray(position, dtype=np.float64).copy())

    def feedback_count(self):
        with self._lock:
            return len(self._feedback_times)

    def snapshot(self):
        with self._lock:
            return (
                np.asarray(self._command_times, dtype=np.float64),
                np.asarray(self._command_positions, dtype=np.float64),
                np.asarray(self._feedback_times, dtype=np.float64),
                np.asarray(self._feedback_positions, dtype=np.float64),
            )


class SynchronizedFeedbackRecorder:
    """Combine one or more arm feedback callbacks into a fixed joint order."""

    def __init__(self, arm_keys, trace):
        self._arm_keys = list(arm_keys)
        self._trace = trace
        self._latest = {}
        self._updated = set()
        self._lock = threading.Lock()

    def update(self, arm_key, position, timestamp=None):
        timestamp = time.perf_counter() if timestamp is None else float(timestamp)
        combined = None
        with self._lock:
            self._latest[arm_key] = np.asarray(position, dtype=np.float64).copy()
            self._updated.add(arm_key)
            if all(key in self._latest for key in self._arm_keys) and all(
                key in self._updated for key in self._arm_keys
            ):
                combined = np.concatenate([self._latest[key] for key in self._arm_keys])
                self._updated.clear()
        if combined is not None:
            self._trace.record_feedback(combined, timestamp)


def _percentile(values, q):
    values = np.asarray(values, dtype=np.float64)
    return float(np.percentile(values, q)) if values.size else None


def _rms(values):
    values = np.asarray(values, dtype=np.float64)
    return float(np.sqrt(np.mean(values * values))) if values.size else None


def _timing_summary(times):
    intervals_ms = np.diff(np.asarray(times, dtype=np.float64)) * 1000.0
    return {
        "sample_count": int(len(times)),
        "interval_ms_median": _percentile(intervals_ms, 50),
        "interval_ms_p95": _percentile(intervals_ms, 95),
        "interval_ms_p99": _percentile(intervals_ms, 99),
        "interval_ms_max": _percentile(intervals_ms, 100),
    }


def _moving_average(values, width):
    width = max(1, int(width))
    if width % 2 == 0:
        width += 1
    if width <= 1:
        return values.copy()
    kernel = np.ones(width, dtype=np.float64) / width
    padding = width // 2
    padded = np.pad(values, (padding, padding), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _normalized_correlation(left, right, active_mask):
    left = np.asarray(left, dtype=np.float64)[active_mask]
    right = np.asarray(right, dtype=np.float64)[active_mask]
    if left.size < 10:
        return None
    left = left - np.mean(left)
    right = right - np.mean(right)
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else None


def _estimate_joint_lag(command_velocity, feedback_velocity, sample_rate_hz, max_lag_ms):
    maximum_lag = min(
        int(round(max_lag_ms * sample_rate_hz / 1000.0)),
        len(command_velocity) // 3,
    )
    motion_threshold = max(0.02, 0.15 * _percentile(np.abs(command_velocity), 95))
    best_lag = None
    best_score = None
    for lag in range(maximum_lag + 1):
        if lag == 0:
            command = command_velocity
            feedback = feedback_velocity
        else:
            command = command_velocity[:-lag]
            feedback = feedback_velocity[lag:]
        active = np.abs(command) >= motion_threshold
        score = _normalized_correlation(command, feedback, active)
        if score is not None and (best_score is None or score > best_score):
            best_lag = lag
            best_score = score
    if best_lag is None:
        return None, None
    return best_lag, best_score


def _find_settle_latency(
    feedback_times,
    feedback_positions,
    final_command_time,
    final_target,
    active_joint_indices,
    tolerance,
    hold_seconds,
):
    after = np.flatnonzero(feedback_times >= final_command_time)
    if not after.size:
        return None
    for start_offset, start_index in enumerate(after):
        start_time = feedback_times[start_index]
        end_time = start_time + hold_seconds
        end_offset = int(np.searchsorted(feedback_times[after], end_time, side="left"))
        if end_offset >= len(after):
            break
        window_indices = after[start_offset : end_offset + 1]
        error = np.abs(
            feedback_positions[window_indices][:, active_joint_indices]
            - final_target[active_joint_indices]
        )
        if np.all(error <= tolerance):
            return float((start_time - final_command_time) * 1000.0)
    return None


def analyze_latency_trace(
    trace,
    gripper_indices,
    analysis_rate_hz=200.0,
    max_lag_ms=1000.0,
    warmup_seconds=0.5,
    settle_tolerance=0.02,
    settle_hold_seconds=0.2,
):
    command_times, command_positions, feedback_times, feedback_positions = trace.snapshot()
    if len(command_times) < 3:
        raise ValueError("At least three recorded commands are required")
    if len(feedback_times) < 3:
        raise ValueError("At least three feedback samples are required")
    if command_positions.ndim != 2 or feedback_positions.ndim != 2:
        raise ValueError("Command and feedback positions must be two-dimensional")
    if command_positions.shape[1] != feedback_positions.shape[1]:
        raise ValueError(
            "Command and feedback dimensions differ: "
            f"{command_positions.shape[1]} versus {feedback_positions.shape[1]}"
        )

    dimension = command_positions.shape[1]
    gripper_indices = {int(index) for index in gripper_indices}
    candidate_joints = [index for index in range(dimension) if index not in gripper_indices]
    analysis_start = max(command_times[0] + warmup_seconds, feedback_times[0])
    analysis_stop = min(command_times[-1], feedback_times[-1])
    if analysis_stop <= analysis_start:
        raise ValueError("Command and feedback timelines do not overlap after warmup")

    sample_period = 1.0 / float(analysis_rate_hz)
    grid = np.arange(analysis_start, analysis_stop, sample_period)
    if len(grid) < 20:
        raise ValueError("The replay interval is too short for latency analysis")

    command_grid = np.column_stack(
        [np.interp(grid, command_times, command_positions[:, joint]) for joint in range(dimension)]
    )
    feedback_grid = np.column_stack(
        [np.interp(grid, feedback_times, feedback_positions[:, joint]) for joint in range(dimension)]
    )
    smoothing_width = max(3, int(round(0.025 * analysis_rate_hz)))
    command_velocity = np.gradient(command_grid, sample_period, axis=0)
    feedback_velocity = np.gradient(feedback_grid, sample_period, axis=0)
    for joint in range(dimension):
        command_velocity[:, joint] = _moving_average(command_velocity[:, joint], smoothing_width)
        feedback_velocity[:, joint] = _moving_average(feedback_velocity[:, joint], smoothing_width)

    joint_reports = []
    active_joint_indices = []
    for joint in candidate_joints:
        travel = float(np.ptp(command_grid[:, joint]))
        command_velocity_rms = _rms(command_velocity[:, joint])
        is_active = travel >= 0.02 and command_velocity_rms >= 0.02
        report = {
            "joint_index": int(joint),
            "active": bool(is_active),
            "command_travel": travel,
            "command_velocity_rms": command_velocity_rms,
        }
        if not is_active:
            report.update(
                {
                    "trajectory_lag_ms": None,
                    "velocity_correlation": None,
                    "tracking_error_rms": _rms(feedback_grid[:, joint] - command_grid[:, joint]),
                    "tracking_error_p95": _percentile(
                        np.abs(feedback_grid[:, joint] - command_grid[:, joint]), 95
                    ),
                    "lag_compensated_error_rms": None,
                }
            )
            joint_reports.append(report)
            continue

        active_joint_indices.append(joint)
        lag_frames, correlation = _estimate_joint_lag(
            command_velocity[:, joint],
            feedback_velocity[:, joint],
            analysis_rate_hz,
            max_lag_ms,
        )
        raw_error = feedback_grid[:, joint] - command_grid[:, joint]
        if lag_frames is None:
            lag_ms = None
            aligned_error_rms = None
        elif lag_frames == 0:
            lag_ms = 0.0
            aligned_error_rms = _rms(raw_error)
        else:
            lag_ms = float(lag_frames / analysis_rate_hz * 1000.0)
            aligned_error_rms = _rms(
                feedback_grid[lag_frames:, joint] - command_grid[:-lag_frames, joint]
            )
        report.update(
            {
                "trajectory_lag_ms": lag_ms,
                "velocity_correlation": correlation,
                "tracking_error_rms": _rms(raw_error),
                "tracking_error_p95": _percentile(np.abs(raw_error), 95),
                "lag_compensated_error_rms": aligned_error_rms,
            }
        )
        joint_reports.append(report)

    if not active_joint_indices:
        raise ValueError("No non-gripper joint moved enough for latency analysis")

    valid_lags = [
        report["trajectory_lag_ms"]
        for report in joint_reports
        if report["active"] and report["trajectory_lag_ms"] is not None
    ]
    settle_latency_ms = _find_settle_latency(
        feedback_times,
        feedback_positions,
        command_times[-1],
        command_positions[-1],
        active_joint_indices,
        settle_tolerance,
        settle_hold_seconds,
    )
    return {
        "measurement_scope": (
            "Local monotonic time from application publish call to locally received ROS joint feedback"
        ),
        "analysis_rate_hz": float(analysis_rate_hz),
        "max_lag_ms": float(max_lag_ms),
        "warmup_seconds": float(warmup_seconds),
        "settle_tolerance": float(settle_tolerance),
        "settle_hold_seconds": float(settle_hold_seconds),
        "command_timing": _timing_summary(command_times),
        "feedback_timing": _timing_summary(feedback_times),
        "aggregate": {
            "active_joint_count": int(len(active_joint_indices)),
            "trajectory_lag_ms_median": _percentile(valid_lags, 50),
            "trajectory_lag_ms_p95": _percentile(valid_lags, 95),
            "terminal_settle_latency_ms": settle_latency_ms,
        },
        "joints": joint_reports,
    }


def save_latency_report(report, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)


def print_latency_report(report):
    command = report["command_timing"]
    feedback = report["feedback_timing"]
    aggregate = report["aggregate"]
    print("Latency measurement scope:", report["measurement_scope"])
    print(
        "Command timing: "
        f"samples={command['sample_count']} "
        f"median={command['interval_ms_median']:.2f}ms "
        f"p95={command['interval_ms_p95']:.2f}ms "
        f"max={command['interval_ms_max']:.2f}ms"
    )
    print(
        "Feedback timing: "
        f"samples={feedback['sample_count']} "
        f"median={feedback['interval_ms_median']:.2f}ms "
        f"p95={feedback['interval_ms_p95']:.2f}ms "
        f"max={feedback['interval_ms_max']:.2f}ms"
    )
    for joint in report["joints"]:
        if not joint["active"]:
            continue
        print(
            f"J{joint['joint_index']}: "
            f"lag={joint['trajectory_lag_ms']:.1f}ms "
            f"corr={joint['velocity_correlation']:.3f} "
            f"error_rms={joint['tracking_error_rms']:.5f} "
            f"aligned_error_rms={joint['lag_compensated_error_rms']:.5f}"
        )
    print(
        "Aggregate: "
        f"active_joints={aggregate['active_joint_count']} "
        f"median_lag={aggregate['trajectory_lag_ms_median']:.1f}ms "
        f"p95_lag={aggregate['trajectory_lag_ms_p95']:.1f}ms "
        f"terminal_settle={aggregate['terminal_settle_latency_ms']}ms"
    )
