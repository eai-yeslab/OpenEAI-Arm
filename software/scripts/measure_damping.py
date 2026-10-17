#!/usr/bin/env python3
"""Measure joint damping with a free-decay step test.

The MIT post-interpolation motion filter would smooth a position step, so this
script writes a temporary config with motion_filter.enabled=false, applies a
small position step to one joint, records the decaying oscillation, and
estimates the damping ratio from the logarithmic decrement.

Safety:
  - one joint at a time, small step amplitude
  - the step target is checked against the configured joint limits
  - the arm is never moved far: the test runs around the current pose
  - the starting pose is restored after each trial
  - motors are disabled on exit
"""

import argparse
import os
import sys
import tempfile
import time

import numpy as np

# Allow importing the built pybind package when running from the source tree.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python"))
import OpenEAIArm  # noqa: E402

try:
    import yaml
except ImportError:
    yaml = None


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/default_right.yml")
    p.add_argument("--joint", type=int, default=0, help="joint index 0..5 to test")
    p.add_argument("--all", action="store_true", help="test all six arm joints")
    p.add_argument("--step", type=float, default=0.02, help="step amplitude in rad")
    p.add_argument("--record-time", type=float, default=3.0, help="seconds to record")
    p.add_argument("--sample-dt", type=float, default=0.005, help="target sampling interval")
    p.add_argument("--settle-time", type=float, default=1.0, help="seconds to settle before/after")
    p.add_argument("--output", default=None, help="optional CSV output path")
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    return p.parse_args()


def load_config_disable_smoothing(src):
    """Return (temporary config path, per-joint lower/upper limits).

    A step must reach the motor unchanged. Disable the post-interpolation
    motion filter and set filter_alpha to zero so that the fallback
    exponential blend does not turn the step into a first-order response.
    """
    if yaml is None:
        raise SystemExit("PyYAML is required to patch the config (pip install pyyaml)")
    with open(src, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    ctl = cfg.get("controller")
    if not ctl or "motion_filter" not in ctl:
        raise SystemExit("config has no controller.motion_filter section")
    ctl["motion_filter"]["enabled"] = False
    ctl["filter_alpha"] = 0.0

    limits = []
    for motor in cfg.get("motors", []):
        limits.append((float(motor["min_position"]), float(motor["max_position"])))
    if len(limits) < 6:
        raise SystemExit("config does not define at least 6 arm joints")

    fd, path = tempfile.mkstemp(suffix=".yml", prefix="damping_config_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True)
    return path, limits


def local_extrema_indices(x):
    idx = []
    for i in range(1, len(x) - 1):
        if (x[i] - x[i - 1]) * (x[i + 1] - x[i]) < 0:
            idx.append(i)
    return np.asarray(idx, dtype=int)


def estimate_zeta(ts, q, steady, min_amp=1e-4):
    """Estimate damping ratio from logarithmic decrement of the oscillation."""
    x = q - steady
    ext = local_extrema_indices(x)
    if len(ext) < 3:
        return None, None, None

    # Same-sign peaks are one full period apart; prefer the dominant sign.
    pos = ext[(x[ext] > min_amp)]
    if len(pos) < 2:
        pos = ext[(x[ext] < -min_amp)]
    if len(pos) < 2:
        return None, None, None

    amps = np.abs(x[pos])
    deltas = np.log(amps[:-1] / amps[1:])
    deltas = deltas[deltas > 0.0]
    if len(deltas) == 0:
        return None, None, None

    delta = float(np.mean(deltas))
    zeta = delta / np.sqrt(4.0 * np.pi ** 2 + delta ** 2)

    periods = np.diff(ts[pos])
    T = float(np.mean(periods)) if len(periods) else 0.0
    wd = (2.0 * np.pi / T) if T > 1e-6 else 0.0
    wn = (wd / np.sqrt(1.0 - zeta ** 2)) if (wd > 1e-6 and zeta < 1.0) else 0.0
    return zeta, wd, wn


def run_trial(arm, joint, step, record_time, sample_dt, settle_time, limits, output):
    q0 = np.asarray(arm.get_joint_positions(), dtype=float)
    lo, hi = limits[joint]
    target = q0.copy()
    target[joint] += step
    if target[joint] < lo or target[joint] > hi:
        raise RuntimeError(f"step target {target[joint]:.4f} outside limits [{lo}, {hi}]")

    time.sleep(settle_time)
    arm.set_joint_targets(list(target), 0.0, interpolate=False)

    t0 = time.monotonic()
    ts = []
    qs = []
    while time.monotonic() - t0 < record_time:
        ts.append(time.monotonic() - t0)
        qs.append(arm.get_joint_positions()[joint])
        time.sleep(sample_dt)

    ts = np.asarray(ts, dtype=float)
    qs = np.asarray(qs, dtype=float)
    steady = float(np.mean(qs[-max(20, len(qs) // 10):]))
    zeta, wd, wn = estimate_zeta(ts, qs, steady)

    arm.set_joint_targets(list(q0), 1.0, interpolate=True)
    time.sleep(settle_time)

    if output:
        with open(output, "w", encoding="utf-8") as f:
            f.write("t_s,q_rad\n")
            for t, q in zip(ts, qs):
                f.write(f"{t:.6f},{q:.6f}\n")

    return zeta, wd, wn


def main():
    args = parse_args()
    if not (0 <= args.joint < 6):
        raise SystemExit("joint must be in 0..5 (gripper is not an MIT impedance joint)")

    cfg_path, limits = load_config_disable_smoothing(args.config)
    arm = OpenEAIArm.OpenEAIArm(cfg_path)

    joints = list(range(6)) if args.all else [args.joint]
    try:
        print("Current pose:", [round(v, 4) for v in arm.get_joint_positions()])
        print(f"Step {args.step:.4f} rad on joint(s) {joints}, record {args.record_time:.2f} s each")
        if not args.yes:
            input("Press Enter to run the step test (Ctrl-C to abort)... ")

        for j in joints:
            out = None
            if args.output:
                out = args.output if not args.all else f"{args.output}.joint{j}"
            elif args.all:
                out = f"damping_joint_{j}.csv"

            print(f"--- joint {j} ---")
            zeta, wd, wn = run_trial(
                arm, j, args.step, args.record_time, args.sample_dt,
                args.settle_time, limits, out,
            )
            if zeta is None:
                print("  no clear decaying oscillation detected (possibly overdamped)")
            else:
                print(f"  zeta={zeta:.4f}  wd={wd:.3f} rad/s  wn={wn:.3f} rad/s")
    finally:
        try:
            arm.disable_all()
        finally:
            os.remove(cfg_path)
    print("done")


if __name__ == "__main__":
    main()
