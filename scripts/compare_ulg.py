#!/usr/bin/env python3
"""
Compare two ULG files (truth vs replay) for dynamics validation.

Usage:
    python3 compare_ulg.py <truth.ulg> <replay.ulg> --output-dir results/comparison
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


def extract_from_ulg(ulg_path):
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


def detect_flight_window(pos_data, z_threshold=-1.0):
    z = pos_data["z"]
    t = pos_data["t"]
    flying = z < z_threshold
    if not np.any(flying):
        return t[0], t[-1]
    start_idx = np.argmax(flying)
    end_idx = len(flying) - 1 - np.argmax(flying[::-1])
    return float(t[start_idx]), float(t[end_idx])


def align_and_compare(truth, replay, channels, t_start, t_end, dt=0.05):
    t = np.arange(t_start + 2, min(t_end, t_start + 200), dt)
    metrics = {}

    for label, t_key, ch_truth, ch_replay in channels:
        t_t = truth[t_key]["t"]
        t_r = replay[t_key]["t"]

        truth_vals = np.interp(t, t_t, truth[t_key][ch_truth])
        replay_vals = np.interp(t, t_r, replay[t_key][ch_replay])

        diff = truth_vals - replay_vals
        rmse = float(np.sqrt(np.mean(diff ** 2)))
        maxerr = float(np.max(np.abs(diff)))

        ss_res = np.sum(diff ** 2)
        ss_tot = np.sum((truth_vals - np.mean(truth_vals)) ** 2)
        r2 = float(1 - ss_res / ss_tot) if ss_tot > 0 else 0.0

        metrics[label] = {
            "RMSE": rmse,
            "MaxErr": maxerr,
            "R2": r2,
            "t": t.tolist(),
            "truth": truth_vals.tolist(),
            "replay": replay_vals.tolist(),
        }

    return metrics


def generate_comparison_plots(metrics, out_dir, truth_label, replay_label):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Position comparison
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Position Comparison: {truth_label} vs {replay_label}",
                 fontsize=14)

    for i, ch in enumerate(["X (North)", "Y (East)", "Z (Down)"]):
        key = ch.split()[0].lower()
        full_key = f"pos_{key}"
        if full_key in metrics:
            m = metrics[full_key]
            axes[i].plot(m["t"], m["truth"], 'b-', linewidth=0.5,
                         label=truth_label)
            axes[i].plot(m["t"], m["replay"], 'r--', linewidth=0.5,
                         label=replay_label)
            axes[i].set_ylabel(f"{ch} (m)")
            axes[i].legend(fontsize=8, loc='upper right')
            axes[i].set_title(
                f"{ch}: RMSE={m['RMSE']:.2f}m, R²={m['R2']:.3f}")
            axes[i].grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(out_dir / "comparison_position.png", dpi=150)
    plt.close()

    # Velocity comparison
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Velocity Comparison: {truth_label} vs {replay_label}",
                 fontsize=14)

    for i, ch in enumerate(["Vx (North)", "Vy (East)", "Vz (Down)"]):
        key = "v" + ch[1].lower()
        full_key = f"vel_{key}"
        if full_key in metrics:
            m = metrics[full_key]
            axes[i].plot(m["t"], m["truth"], 'b-', linewidth=0.5,
                         label=truth_label)
            axes[i].plot(m["t"], m["replay"], 'r--', linewidth=0.5,
                         label=replay_label)
            axes[i].set_ylabel(f"{ch} (m/s)")
            axes[i].legend(fontsize=8, loc='upper right')
            axes[i].set_title(
                f"{ch}: RMSE={m['RMSE']:.2f}m/s, R²={m['R2']:.3f}")
            axes[i].grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(out_dir / "comparison_velocity.png", dpi=150)
    plt.close()

    # Speed comparison
    if "speed" in metrics:
        fig, ax = plt.subplots(1, 1, figsize=(14, 5))
        m = metrics["speed"]
        ax.plot(m["t"], m["truth"], 'b-', linewidth=1, label=truth_label)
        ax.plot(m["t"], m["replay"], 'r--', linewidth=1, label=replay_label)
        ax.set_ylabel("Horizontal Speed (m/s)")
        ax.set_xlabel("Time (s)")
        ax.set_title(f"Speed: RMSE={m['RMSE']:.2f}m/s, R²={m['R2']:.3f}")
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(out_dir / "comparison_speed.png", dpi=150)
        plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("truth_ulg", help="Truth ULG file")
    parser.add_argument("replay_ulg", help="Replay ULG file")
    parser.add_argument("--output-dir", default="results/comparison")
    parser.add_argument("--truth-label", default="Truth")
    parser.add_argument("--replay-label", default="Replay")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading truth: {args.truth_ulg}")
    truth = extract_from_ulg(args.truth_ulg)
    print(f"Loading replay: {args.replay_ulg}")
    replay = extract_from_ulg(args.replay_ulg)

    # Detect flight windows
    t_start_t, t_end_t = detect_flight_window(truth["pos"])
    t_start_r, t_end_r = detect_flight_window(replay["pos"])
    print(f"Truth flight: {t_start_t:.1f} - {t_end_t:.1f}s")
    print(f"Replay flight: {t_start_r:.1f} - {t_end_r:.1f}s")

    # Use replay window
    t_start = t_start_r
    t_end = min(t_end_t, t_end_r)

    channels = [
        ("pos_x", "pos", "x", "x"),
        ("pos_y", "pos", "y", "y"),
        ("pos_z", "pos", "z", "z"),
        ("vel_vx", "pos", "vx", "vx"),
        ("vel_vy", "pos", "vy", "vy"),
        ("vel_vz", "pos", "vz", "vz"),
    ]

    metrics = align_and_compare(truth, replay, channels, t_start, t_end)

    # Add horizontal speed
    t = np.arange(t_start + 2, min(t_end, t_start + 200), 0.05)
    truth_vx = np.interp(t, truth["pos"]["t"], truth["pos"]["vx"])
    truth_vy = np.interp(t, truth["pos"]["t"], truth["pos"]["vy"])
    truth_speed = np.sqrt(truth_vx ** 2 + truth_vy ** 2)

    replay_vx = np.interp(t, replay["pos"]["t"], replay["pos"]["vx"])
    replay_vy = np.interp(t, replay["pos"]["t"], replay["pos"]["vy"])
    replay_speed = np.sqrt(replay_vx ** 2 + replay_vy ** 2)

    diff = truth_speed - replay_speed
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    maxerr = float(np.max(np.abs(diff)))
    ss_res = np.sum(diff ** 2)
    ss_tot = np.sum((truth_speed - np.mean(truth_speed)) ** 2)
    r2 = float(1 - ss_res / ss_tot) if ss_tot > 0 else 0.0

    metrics["speed"] = {
        "RMSE": rmse, "MaxErr": maxerr, "R2": r2,
        "t": t.tolist(), "truth": truth_speed.tolist(),
        "replay": replay_speed.tolist(),
    }

    # Print results
    print(f"\n{'='*60}")
    print(f"  SETPOINT REPLAY COMPARISON RESULTS")
    print(f"{'='*60}")
    for ch, m in metrics.items():
        unit = "m" if "pos" in ch else "m/s" if "vel" in ch or ch == "speed" else ""
        print(f"  {ch:>10s}: RMSE={m['RMSE']:.3f}{unit}, "
              f"MaxErr={m['MaxErr']:.3f}{unit}, R²={m['R2']:.4f}")

    generate_comparison_plots(metrics, out_dir,
                              args.truth_label, args.replay_label)

    # Save metrics (without time series)
    json_metrics = {}
    for ch, m in metrics.items():
        json_metrics[ch] = {k: v for k, v in m.items()
                            if k not in ("t", "truth", "replay")}
    with open(out_dir / "metrics.json", "w") as f:
        json.dump(json_metrics, f, indent=2)

    print(f"\nPlots and metrics saved to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
