#!/usr/bin/env python3
"""Estimate electrical input power of the OpenEAI arm from a motor_state_log.csv.

daMiao DM motors do not report bus voltage/current over CAN, so this script
estimates the electrical input power from the feedback torque using the rated
operating point of each motor type:

    I_est  = |tau_fb| / rated_torque * rated_current
    P_elec = bus_voltage * I_est + overhead_w
    P_mech = tau_fb * dq_fb

The estimate is a conservative upper bound. It is intended for an initial power
budget only; a hardware power meter (or daMiao KT/Rs readback) is required for
precise electrical power.
"""

import argparse
import csv
import os

import numpy as np


NUM_JOINTS = 7

# Rated operating point per motor type. Matches the C++ ElectricalSpec table in
# dm.hpp. The first three joints are DM4340 and the last four are DM4310 for the
# current arm configuration.
ELECTRICAL_SPEC = {
    "DM4310": {"bus_voltage": 24.0, "rated_torque": 3.0, "rated_current": 2.5, "overhead_w": 1.0},
    "DM4310_48V": {"bus_voltage": 48.0, "rated_torque": 3.0, "rated_current": 1.0, "overhead_w": 1.0},
    "DM4340": {"bus_voltage": 24.0, "rated_torque": 9.0, "rated_current": 2.5, "overhead_w": 1.0},
    "DM4340_48V": {"bus_voltage": 48.0, "rated_torque": 9.0, "rated_current": 1.0, "overhead_w": 1.0},
}

DEFAULT_TYPES = ["DM4340", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM4310"]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default="motor_state_log.csv", help="Path to motor_state_log.csv")
    parser.add_argument(
        "--motor-types",
        default=",".join(DEFAULT_TYPES),
        help="Comma-separated motor type for joints 0..6 (default: %(default)s)",
    )
    parser.add_argument(
        "--bus-voltage-override",
        type=float,
        default=None,
        help="Override bus voltage (V) for all joints",
    )
    parser.add_argument(
        "--static-threshold",
        type=float,
        default=0.05,
        help="|dq| below this (rad/s) on all joints is treated as static hold",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional CSV path to write per-sample estimated power",
    )
    return parser.parse_args()


def load_columns(path):
    cols = {}
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                cols.setdefault(key, []).append(float(value) if value not in ("", None) else 0.0)
    return cols


def summarize(name, values):
    if len(values) == 0:
        print(f"{name:<24} no samples")
        return
    a = np.asarray(values, dtype=float)
    print(
        f"{name:<24} n={len(a):8d}  "
        f"median={np.median(a):7.3f} W  mean={a.mean():7.3f} W  "
        f"p90={np.percentile(a, 90):7.3f} W  p99={np.percentile(a, 99):7.3f} W  "
        f"max={a.max():7.3f} W"
    )


def main():
    args = parse_args()
    if not os.path.isfile(args.csv):
        raise SystemExit(f"CSV not found: {args.csv}")

    motor_types = [t.strip() for t in args.motor_types.split(",")]
    if len(motor_types) != NUM_JOINTS:
        raise SystemExit(f"Expected {NUM_JOINTS} motor types, got {len(motor_types)}")
    for t in motor_types:
        if t not in ELECTRICAL_SPEC:
            raise SystemExit(f"Unknown motor type '{t}'. Known: {list(ELECTRICAL_SPEC)}")

    cols = load_columns(args.csv)
    tau = np.array([cols[f"tau_{i}"] for i in range(NUM_JOINTS)], dtype=float)
    dq = np.array([cols[f"dq_{i}"] for i in range(NUM_JOINTS)], dtype=float)

    p_mech = tau * dq
    p_elec = np.zeros_like(p_mech)
    for i, mtype in enumerate(motor_types):
        spec = dict(ELECTRICAL_SPEC[mtype])
        if args.bus_voltage_override is not None:
            spec["bus_voltage"] = args.bus_voltage_override
        est_current = np.abs(tau[i]) / spec["rated_torque"] * spec["rated_current"]
        p_elec[i] = spec["bus_voltage"] * est_current + spec["overhead_w"]

    total_mech = p_mech.sum(axis=0)
    total_elec = p_elec.sum(axis=0)

    static = np.all(np.abs(dq) < args.static_threshold, axis=0)
    moving = ~static

    print("Motor types: " + ", ".join(motor_types))
    print(f"Static threshold: |dq| < {args.static_threshold} rad/s on all joints\n")

    print("== Total estimated electrical input power (arm-level) ==")
    summarize("all samples", total_elec)
    summarize("static hold", total_elec[static])
    summarize("in motion", total_elec[moving])

    print("\n== Total mechanical output power (reference) ==")
    summarize("all samples", total_mech)
    summarize("static hold", total_mech[static])
    summarize("in motion", total_mech[moving])

    print("\n== Per-joint estimated electrical power (all samples) ==")
    print(f"{'joint':<7}{'type':<12}{'mean':>9}{'p90':>9}{'p99':>9}{'max':>9}  (W)")
    for i, mtype in enumerate(motor_types):
        a = p_elec[i]
        print(
            f"J{i:<6}{mtype:<12}{a.mean():9.3f}{np.percentile(a, 90):9.3f}"
            f"{np.percentile(a, 99):9.3f}{a.max():9.3f}"
        )

    print(
        "\nNote: this is a conservative estimate from rated torque/current. "
        "Precise electrical power requires bus voltage/current measurement."
    )

    if args.output:
        with open(args.output, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            header = ["timestamp_ms"]
            header += [f"p_mech_{i}" for i in range(NUM_JOINTS)]
            header += [f"p_elec_{i}" for i in range(NUM_JOINTS)]
            writer.writerow(header)
            ts = cols.get("timestamp_ms", [0] * p_mech.shape[1])
            for k in range(p_mech.shape[1]):
                writer.writerow([ts[k]] + list(p_mech[:, k]) + list(p_elec[:, k]))
        print(f"Per-sample power written to: {args.output}")


if __name__ == "__main__":
    main()
