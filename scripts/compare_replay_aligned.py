#!/usr/bin/env python3
"""
Aligned comparison of truth vs replay for setpoint replay validation.

Aligns flights by:
1. Finding takeoff time in both
2. Comparing from relative t=0 (takeoff)
3. Focusing on velocity/acceleration which are independent of absolute position
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def extract(ulg_path):
    ulog = ULog(str(ulg_path))
    data = {d.name: d.data for d in ulog.data_list}
    t0 = min(d.data["timestamp"][0] for d in ulog.data_list
             if len(d.data["timestamp"]) > 0)
    result = {}
    if "vehicle_local_position" in data:
        d = data["vehicle_local_position"]
        t = (d["timestamp"] - t0) / 1e6
        result["pos"] = {"t": t, "x": d["x"], "y": d["y"], "z": d["z"],
                         "vx": d["vx"], "vy": d["vy"], "vz": d["vz"]}
    if "vehicle_attitude" in data:
        d = data["vehicle_attitude"]
        t = (d["timestamp"] - t0) / 1e6
        q0, q1, q2, q3 = d["q[0]"], d["q[1]"], d["q[2]"], d["q[3]"]
        roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3),
                                     1 - 2 * (q1 ** 2 + q2 ** 2)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
        yaw = np.degrees(np.arctan2(2 * (q0 * q3 + q1 * q2),
                                    1 - 2 * (q2 ** 2 + q3 ** 2)))
        result["att"] = {"t": t, "roll": roll, "pitch": pitch, "yaw": yaw}
    if "vehicle_acceleration" in data:
        d = data["vehicle_acceleration"]
        t = (d["timestamp"] - t0) / 1e6
        result["accel"] = {"t": t, "ax": d["xyz[0]"], "ay": d["xyz[1]"],
                           "az": d["xyz[2]"]}
    return result


def find_takeoff(pos, z_thr=-1.5):
    """Find when altitude first exceeds threshold."""
    z = np.array(pos["z"])
    t = np.array(pos["t"])
    flying = z < z_thr
    if not np.any(flying):
        return float(t[0])
    return float(t[np.argmax(flying)])


def compute_metrics(truth_vals, replay_vals):
    diff = truth_vals - replay_vals
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    maxerr = float(np.max(np.abs(diff)))
    ss_res = np.sum(diff ** 2)
    ss_tot = np.sum((truth_vals - np.mean(truth_vals)) ** 2)
    r2 = float(1 - ss_res / ss_tot) if ss_tot > 1e-10 else 0.0
    return {"RMSE": rmse, "MaxErr": maxerr, "R2": r2}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("truth_ulg")
    parser.add_argument("replay_ulg")
    parser.add_argument("--output-dir", default="results/replay_comparison")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Truth: {args.truth_ulg}")
    print(f"Replay: {args.replay_ulg}")

    truth = extract(args.truth_ulg)
    replay = extract(args.replay_ulg)

    # Align by takeoff
    t0_truth = find_takeoff(truth["pos"])
    t0_replay = find_takeoff(replay["pos"])
    print(f"Truth takeoff: {t0_truth:.1f}s, Replay takeoff: {t0_replay:.1f}s")

    # Common relative time range (from takeoff)
    truth_dur = truth["pos"]["t"][-1] - t0_truth
    replay_dur = replay["pos"]["t"][-1] - t0_replay
    max_dur = min(truth_dur, replay_dur) - 5
    print(f"Duration: truth={truth_dur:.1f}s, replay={replay_dur:.1f}s, "
          f"compare={max_dur:.1f}s")

    dt = 0.05
    t_rel = np.arange(2, max_dur, dt)

    # Interpolate to common relative time
    def interp(data, key, field, t0_abs):
        return np.interp(t_rel + t0_abs, data[key]["t"], data[key][field])

    # Velocity comparison (most meaningful for dynamics validation)
    channels = {
        "vx": ("pos", "vx"), "vy": ("pos", "vy"), "vz": ("pos", "vz"),
    }

    all_metrics = {}
    print(f"\n{'='*60}")
    print(f"  ALIGNED SETPOINT REPLAY COMPARISON")
    print(f"{'='*60}")

    fig_vel, axes_vel = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
    fig_vel.suptitle("Velocity Comparison (Aligned by Takeoff)", fontsize=14)

    truth_vx = interp(truth, "pos", "vx", t0_truth)
    truth_vy = interp(truth, "pos", "vy", t0_truth)
    truth_vz = interp(truth, "pos", "vz", t0_truth)
    replay_vx = interp(replay, "pos", "vx", t0_replay)
    replay_vy = interp(replay, "pos", "vy", t0_replay)
    replay_vz = interp(replay, "pos", "vz", t0_replay)

    truth_speed = np.sqrt(truth_vx ** 2 + truth_vy ** 2)
    replay_speed = np.sqrt(replay_vx ** 2 + replay_vy ** 2)

    vel_data = [
        ("Vx (North)", truth_vx, replay_vx, "m/s"),
        ("Vy (East)", truth_vy, replay_vy, "m/s"),
        ("Vz (Down)", truth_vz, replay_vz, "m/s"),
        ("Horizontal Speed", truth_speed, replay_speed, "m/s"),
    ]

    for i, (name, tv, rv, unit) in enumerate(vel_data):
        m = compute_metrics(tv, rv)
        key = name.lower().replace(" ", "_").replace("(", "").replace(")", "")
        all_metrics[key] = m
        print(f"  {name:>20s}: RMSE={m['RMSE']:.3f}{unit}, "
              f"R²={m['R2']:.4f}")

        axes_vel[i].plot(t_rel, tv, 'b-', linewidth=0.5, label='Truth',
                         alpha=0.8)
        axes_vel[i].plot(t_rel, rv, 'r--', linewidth=0.5, label='Replay',
                         alpha=0.8)
        axes_vel[i].set_ylabel(f"{name} ({unit})")
        axes_vel[i].set_title(
            f"{name}: RMSE={m['RMSE']:.2f}{unit}, R²={m['R2']:.3f}")
        axes_vel[i].legend(fontsize=8)
        axes_vel[i].grid(True, alpha=0.3)

    axes_vel[-1].set_xlabel("Relative Time from Takeoff (s)")
    plt.tight_layout()
    plt.savefig(out_dir / "velocity_comparison.png", dpi=150)
    plt.close()

    # Relative position comparison
    truth_dx = interp(truth, "pos", "x", t0_truth)
    truth_dy = interp(truth, "pos", "y", t0_truth)
    truth_dz = interp(truth, "pos", "z", t0_truth)
    replay_dx = interp(replay, "pos", "x", t0_replay)
    replay_dy = interp(replay, "pos", "y", t0_replay)
    replay_dz = interp(replay, "pos", "z", t0_replay)

    # Normalize to relative positions (from start)
    truth_dx -= truth_dx[0]
    truth_dy -= truth_dy[0]
    truth_dz -= truth_dz[0]
    replay_dx -= replay_dx[0]
    replay_dy -= replay_dy[0]
    replay_dz -= replay_dz[0]

    fig_pos, axes_pos = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig_pos.suptitle("Relative Position (Aligned, Origin at Takeoff)",
                     fontsize=14)

    pos_data = [
        ("dX (North)", truth_dx, replay_dx, "m"),
        ("dY (East)", truth_dy, replay_dy, "m"),
        ("dZ (Down)", truth_dz, replay_dz, "m"),
    ]

    for i, (name, tv, rv, unit) in enumerate(pos_data):
        m = compute_metrics(tv, rv)
        key = f"rel_{name.split()[0].lower()}"
        all_metrics[key] = m
        print(f"  {name:>20s}: RMSE={m['RMSE']:.2f}{unit}, "
              f"R²={m['R2']:.4f}")

        axes_pos[i].plot(t_rel, tv, 'b-', linewidth=0.5, label='Truth')
        axes_pos[i].plot(t_rel, rv, 'r--', linewidth=0.5, label='Replay')
        axes_pos[i].set_ylabel(f"{name} ({unit})")
        axes_pos[i].set_title(
            f"{name}: RMSE={m['RMSE']:.2f}{unit}, R²={m['R2']:.3f}")
        axes_pos[i].legend(fontsize=8)
        axes_pos[i].grid(True, alpha=0.3)

    axes_pos[-1].set_xlabel("Relative Time from Takeoff (s)")
    plt.tight_layout()
    plt.savefig(out_dir / "position_comparison.png", dpi=150)
    plt.close()

    # Attitude comparison
    if "att" in truth and "att" in replay:
        fig_att, axes_att = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
        fig_att.suptitle("Attitude Comparison", fontsize=14)

        for i, ch in enumerate(["roll", "pitch", "yaw"]):
            tv = interp(truth, "att", ch, t0_truth)
            rv = interp(replay, "att", ch, t0_replay)
            m = compute_metrics(tv, rv)
            all_metrics[ch] = m
            print(f"  {ch:>20s}: RMSE={m['RMSE']:.2f}°, R²={m['R2']:.4f}")

            axes_att[i].plot(t_rel, tv, 'b-', linewidth=0.5, label='Truth')
            axes_att[i].plot(t_rel, rv, 'r--', linewidth=0.5, label='Replay')
            axes_att[i].set_ylabel(f"{ch.capitalize()} (°)")
            axes_att[i].set_title(
                f"{ch.capitalize()}: RMSE={m['RMSE']:.2f}°, R²={m['R2']:.3f}")
            axes_att[i].legend(fontsize=8)
            axes_att[i].grid(True, alpha=0.3)

        axes_att[-1].set_xlabel("Relative Time (s)")
        plt.tight_layout()
        plt.savefig(out_dir / "attitude_comparison.png", dpi=150)
        plt.close()

    # Speed-binned error analysis
    print(f"\n  Speed-dependent RMSE:")
    speed_bins = [(0, 5), (5, 15), (15, 30), (30, 50), (0, 50)]
    speed_rmse = {}
    for v_lo, v_hi in speed_bins:
        mask = (truth_speed >= v_lo) & (truth_speed < v_hi)
        if np.sum(mask) > 10:
            vx_rmse = float(np.sqrt(np.mean(
                (truth_vx[mask] - replay_vx[mask]) ** 2)))
            vy_rmse = float(np.sqrt(np.mean(
                (truth_vy[mask] - replay_vy[mask]) ** 2)))
            spd_rmse = float(np.sqrt(np.mean(
                (truth_speed[mask] - replay_speed[mask]) ** 2)))
            print(f"    {v_lo}-{v_hi} m/s: speed_RMSE={spd_rmse:.2f}, "
                  f"vx_RMSE={vx_rmse:.2f}, vy_RMSE={vy_rmse:.2f}, "
                  f"n={np.sum(mask)}")
            speed_rmse[f"{v_lo}-{v_hi}"] = {
                "speed_rmse": spd_rmse,
                "vx_rmse": vx_rmse,
                "vy_rmse": vy_rmse,
                "n_samples": int(np.sum(mask)),
            }

    all_metrics["speed_binned_rmse"] = speed_rmse

    # Save metrics
    json_metrics = {}
    for k, v in all_metrics.items():
        if isinstance(v, dict):
            json_metrics[k] = {kk: vv for kk, vv in v.items()
                               if kk not in ("t", "truth", "replay")}
        else:
            json_metrics[k] = v

    with open(out_dir / "metrics.json", "w") as f:
        json.dump(json_metrics, f, indent=2)

    print(f"\nResults saved to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
