#!/usr/bin/env python3
"""
Extract key flight data from PX4 ULG log files.

Usage:
    python3 extract_ulg.py <ulg_file> [--output-dir <dir>]

Outputs CSV files:
    - position.csv:  timestamp_s, x, y, z (NED, meters)
    - velocity.csv:  timestamp_s, vx, vy, vz (NED, m/s)
    - attitude.csv:  timestamp_s, roll_deg, pitch_deg, yaw_deg
    - angular_velocity.csv: timestamp_s, rollspeed, pitchspeed, yawspeed (rad/s)
    - actuator.csv:  timestamp_s, output[0-3] (normalized motor outputs)
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
from pyulog import ULog


def quat_to_euler(q0, q1, q2, q3):
    roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1**2 + q2**2))
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi / 2, sinp), np.arcsin(sinp))
    yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2**2 + q3**2))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def get_dataset(ulog: ULog, topic: str):
    matches = [d for d in ulog.data_list if d.name == topic]
    if not matches:
        print(f"Warning: topic '{topic}' not found in log, skipping.")
        return None
    return matches[0]


def timestamps_to_seconds(ts_us, t0_us):
    return (ts_us - t0_us) / 1e6


def extract(ulg_path: str, output_dir: str):
    ulog = ULog(ulg_path)
    os.makedirs(output_dir, exist_ok=True)

    all_ts = []
    for d in ulog.data_list:
        if len(d.data["timestamp"]) > 0:
            all_ts.append(d.data["timestamp"][0])
    t0 = min(all_ts) if all_ts else 0

    # position & velocity
    ds = get_dataset(ulog, "vehicle_local_position")
    if ds is not None:
        t = timestamps_to_seconds(ds.data["timestamp"], t0)
        pd.DataFrame({
            "timestamp_s": t, "x": ds.data["x"], "y": ds.data["y"], "z": ds.data["z"],
        }).to_csv(os.path.join(output_dir, "position.csv"), index=False)

        pd.DataFrame({
            "timestamp_s": t, "vx": ds.data["vx"], "vy": ds.data["vy"], "vz": ds.data["vz"],
        }).to_csv(os.path.join(output_dir, "velocity.csv"), index=False)
        print(f"  position.csv / velocity.csv  ({len(t)} rows)")

    # attitude
    ds = get_dataset(ulog, "vehicle_attitude")
    if ds is not None:
        t = timestamps_to_seconds(ds.data["timestamp"], t0)
        q0, q1, q2, q3 = ds.data["q[0]"], ds.data["q[1]"], ds.data["q[2]"], ds.data["q[3]"]
        roll, pitch, yaw = quat_to_euler(q0, q1, q2, q3)
        pd.DataFrame({
            "timestamp_s": t, "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
        }).to_csv(os.path.join(output_dir, "attitude.csv"), index=False)
        print(f"  attitude.csv                 ({len(t)} rows)")

    # angular velocity
    ds = get_dataset(ulog, "vehicle_angular_velocity")
    if ds is not None:
        t = timestamps_to_seconds(ds.data["timestamp"], t0)
        pd.DataFrame({
            "timestamp_s": t,
            "rollspeed": ds.data["xyz[0]"],
            "pitchspeed": ds.data["xyz[1]"],
            "yawspeed": ds.data["xyz[2]"],
        }).to_csv(os.path.join(output_dir, "angular_velocity.csv"), index=False)
        print(f"  angular_velocity.csv         ({len(t)} rows)")

    # actuator outputs
    ds = get_dataset(ulog, "actuator_outputs")
    if ds is not None:
        t = timestamps_to_seconds(ds.data["timestamp"], t0)
        data = {"timestamp_s": t}
        for i in range(4):
            key = f"output[{i}]"
            data[key] = ds.data[key]
        pd.DataFrame(data).to_csv(os.path.join(output_dir, "actuator.csv"), index=False)
        print(f"  actuator.csv                 ({len(t)} rows)")

    print(f"Done. Output saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Extract flight data from PX4 ULG log.")
    parser.add_argument("ulg_file", help="Path to ULG file")
    parser.add_argument("--output-dir", default=None, help="Output directory (default: same name as ulg file)")
    args = parser.parse_args()

    if not os.path.isfile(args.ulg_file):
        print(f"Error: file not found: {args.ulg_file}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.splitext(args.ulg_file)[0]

    print(f"Extracting: {args.ulg_file}")
    extract(args.ulg_file, output_dir)


if __name__ == "__main__":
    main()
