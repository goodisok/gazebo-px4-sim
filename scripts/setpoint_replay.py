#!/usr/bin/env python3
"""
Setpoint replay for Sim-to-Real dynamics validation.

Extracts trajectory setpoints from a PX4 ULG log (the "truth" flight),
and replays them to a different drone's PX4 via MAVSDK offboard mode.
Compares how the sim's controller + dynamics tracks the same setpoints.

Usage:
    # 1. Start PX4+Gazebo interceptor sim
    # 2. Run this script:
    python3 setpoint_replay.py \
        --ulg data/flight_logs/x500_truth.ulg \
        --output-dir results/setpoint_replay_round4
"""

import asyncio
import argparse
import json
import sys
import time
import numpy as np
import pandas as pd
from pathlib import Path
from pyulog import ULog
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)


def extract_setpoints(ulg_path):
    """Extract trajectory_setpoint and truth states from ULG."""
    ulog = ULog(ulg_path)
    data = {}
    for d in ulog.data_list:
        data[d.name] = d.data

    t0 = data["trajectory_setpoint"]["timestamp"][0]

    traj = data["trajectory_setpoint"]
    ts = (traj["timestamp"] - t0) / 1e6

    setpoints = pd.DataFrame({
        "t": ts,
        "px": traj["position[0]"],
        "py": traj["position[1]"],
        "pz": traj["position[2]"],
        "vx": traj["velocity[0]"],
        "vy": traj["velocity[1]"],
        "vz": traj["velocity[2]"],
        "yaw": traj["yaw"],
    })

    truth = {}
    if "vehicle_local_position" in data:
        pos = data["vehicle_local_position"]
        pos_ts = (pos["timestamp"] - t0) / 1e6
        truth["pos"] = pd.DataFrame({
            "t": pos_ts, "x": pos["x"], "y": pos["y"], "z": pos["z"],
            "vx": pos["vx"], "vy": pos["vy"], "vz": pos["vz"],
        })
    if "vehicle_attitude" in data:
        att = data["vehicle_attitude"]
        att_ts = (att["timestamp"] - t0) / 1e6
        q0, q1, q2, q3 = att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"]
        truth["att"] = pd.DataFrame({
            "t": att_ts,
            "roll": np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))),
            "pitch": np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1))),
            "yaw": np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))),
        })

    return setpoints, truth


def find_flight_window(setpoints):
    """Find the time range with valid setpoints (skip NaN startup)."""
    valid = setpoints.dropna(subset=["px", "py", "pz"])
    if len(valid) == 0:
        raise ValueError("No valid position setpoints found")
    return valid.index[0], valid.index[-1]


async def replay_setpoints(setpoints, output_dir):
    """Connect to PX4, arm, takeoff, replay setpoints via offboard."""

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("  Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected!")
            break

    print("  Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  GPS OK!")
            break

    first_valid_idx, last_valid_idx = find_flight_window(setpoints)
    first_sp = setpoints.iloc[first_valid_idx]
    print(f"  First valid setpoint at t={first_sp['t']:.1f}s: "
          f"pos=[{first_sp['px']:.2f}, {first_sp['py']:.2f}, {first_sp['pz']:.2f}]")

    # Start sending setpoints before switching to offboard
    initial_sp = PositionNedYaw(0.0, 0.0, -2.0, 0.0)
    await drone.offboard.set_position_ned(initial_sp)

    print("  Arming...")
    await drone.action.arm()
    await asyncio.sleep(1)

    print("  Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("  Switching to offboard mode...")
    await drone.offboard.set_position_ned(initial_sp)
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"  Offboard start failed: {e}")
        return None

    print("  Offboard active. Replaying setpoints...")

    replay_start = time.monotonic()
    sp_start_time = setpoints.iloc[first_valid_idx]["t"]
    n_sent = 0
    last_mode = None

    for idx in range(first_valid_idx, last_valid_idx + 1):
        sp = setpoints.iloc[idx]
        target_wall_time = replay_start + (sp["t"] - sp_start_time)

        now = time.monotonic()
        if target_wall_time > now:
            await asyncio.sleep(target_wall_time - now)

        has_pos = not (np.isnan(sp["px"]) or np.isnan(sp["py"]) or np.isnan(sp["pz"]))
        has_vel = not (np.isnan(sp["vx"]) or np.isnan(sp["vy"]) or np.isnan(sp["vz"]))
        yaw_val = sp["yaw"] if not np.isnan(sp["yaw"]) else 0.0
        yaw_deg = np.degrees(yaw_val)

        if has_pos:
            await drone.offboard.set_position_ned(
                PositionNedYaw(float(sp["px"]), float(sp["py"]),
                               float(sp["pz"]), float(yaw_deg)))
            mode = "POS"
        elif has_vel:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(float(sp["vx"]), float(sp["vy"]),
                               float(sp["vz"]), 0.0))
            mode = "VEL"
        else:
            continue

        n_sent += 1
        if mode != last_mode:
            elapsed = time.monotonic() - replay_start
            print(f"    [{elapsed:.1f}s] Mode → {mode}, "
                  f"sp=[{sp['px']:.1f},{sp['py']:.1f},{sp['pz']:.1f}]")
            last_mode = mode

        if n_sent % 50 == 0:
            elapsed = time.monotonic() - replay_start
            print(f"    [{elapsed:.1f}s] Sent {n_sent}/{last_valid_idx - first_valid_idx + 1}")

    replay_duration = time.monotonic() - replay_start
    print(f"  Replay done: {n_sent} setpoints in {replay_duration:.1f}s")

    print("  Returning home and landing...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()
    await asyncio.sleep(10)

    print("  Disarming...")
    await drone.action.disarm()

    return replay_duration


def main():
    parser = argparse.ArgumentParser(description="Setpoint replay for dynamics validation")
    parser.add_argument("--ulg", required=True, help="Path to truth ULG file")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Setpoint Replay - Dynamics Validation")
    print("=" * 60)

    print("\n[1/3] Extracting setpoints from truth ULG...")
    setpoints, truth = extract_setpoints(args.ulg)
    first_idx, last_idx = find_flight_window(setpoints)
    print(f"  Total setpoints: {len(setpoints)}")
    print(f"  Valid range: samples {first_idx}-{last_idx} "
          f"(t={setpoints.iloc[first_idx]['t']:.1f}s - {setpoints.iloc[last_idx]['t']:.1f}s)")

    n_pos = setpoints[["px", "py", "pz"]].dropna().shape[0]
    n_vel = setpoints[["vx", "vy", "vz"]].dropna().shape[0]
    print(f"  Position setpoints: {n_pos}, Velocity setpoints: {n_vel}")

    # Save extracted setpoints for reference
    setpoints.to_csv(output_dir / "truth_setpoints.csv", index=False)

    print("\n[2/3] Connecting to PX4 and replaying setpoints...")
    print("  (Make sure PX4+Gazebo interceptor is running)")
    loop = asyncio.get_event_loop()
    duration = loop.run_until_complete(replay_setpoints(setpoints, output_dir))

    if duration is None:
        print("  Replay failed!")
        return 1

    print(f"\n[3/3] Replay complete! Duration: {duration:.1f}s")
    print(f"  Now extract the interceptor's ULG log and compare with truth.")
    print(f"  Use scripts/compare.py for the comparison.")
    print(f"  Results saved to: {output_dir}/")

    return 0


if __name__ == "__main__":
    sys.exit(main())
