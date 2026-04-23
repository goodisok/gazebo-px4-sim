#!/usr/bin/env python3
"""
Compare two sets of flight data (ground truth vs simulation).

Usage:
    python3 compare.py <truth_dir> <sim_dir> [--output-dir <dir>] [--label-truth <str>] [--label-sim <str>]

Outputs:
    - metrics.json: RMSE, MaxErr, R² for each channel
    - comparison_position.png: 3-panel position comparison (X, Y, Z)
    - comparison_velocity.png: 3-panel velocity comparison
    - comparison_attitude.png: 3-panel attitude comparison (Roll, Pitch, Yaw)
    - comparison_angular_velocity.png: 3-panel angular velocity comparison
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def detect_takeoff_time(pos_df: pd.DataFrame, threshold: float = 0.3) -> float:
    if pos_df is None or pos_df.empty:
        return 0.0
    x, y, z = pos_df["x"].values, pos_df["y"].values, pos_df["z"].values
    disp = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)
    idx = np.argmax(disp > threshold)
    if disp[idx] <= threshold:
        return 0.0
    return pos_df["timestamp_s"].iloc[idx]


def align_data(truth_df: pd.DataFrame, sim_df: pd.DataFrame,
               t0_truth: float, t0_sim: float, cols: list[str]):
    truth = truth_df.copy()
    sim = sim_df.copy()
    truth["t"] = truth["timestamp_s"] - t0_truth
    sim["t"] = sim["timestamp_s"] - t0_sim

    truth = truth[truth["t"] >= 0].reset_index(drop=True)
    sim = sim[sim["t"] >= 0].reset_index(drop=True)

    t_start = max(truth["t"].iloc[0], sim["t"].iloc[0])
    t_end = min(truth["t"].iloc[-1], sim["t"].iloc[-1])
    if t_end <= t_start:
        return None

    n_points = min(2000, max(len(truth), len(sim)))
    t_common = np.linspace(t_start, t_end, n_points)

    result = {"t": t_common}
    for col in cols:
        result[f"truth_{col}"] = np.interp(t_common, truth["t"].values, truth[col].values)
        result[f"sim_{col}"] = np.interp(t_common, sim["t"].values, sim[col].values)

    return pd.DataFrame(result)


def compute_metrics(truth: np.ndarray, sim: np.ndarray) -> dict:
    err = truth - sim
    rmse = float(np.sqrt(np.mean(err**2)))
    max_err = float(np.max(np.abs(err)))
    ss_res = np.sum(err**2)
    ss_tot = np.sum((truth - np.mean(truth))**2)
    r2 = float(1 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")
    return {"RMSE": round(rmse, 4), "MaxErr": round(max_err, 4), "R2": round(r2, 4)}


def plot_comparison(aligned: pd.DataFrame, cols: list[str], labels: list[str],
                    ylabel: str, title: str, output_path: str,
                    label_truth: str, label_sim: str, metrics: dict):
    fig, axes = plt.subplots(len(cols), 1, figsize=(12, 3.2 * len(cols)), sharex=True)
    if len(cols) == 1:
        axes = [axes]

    for ax, col, label in zip(axes, cols, labels):
        t = aligned["t"].values
        tr = aligned[f"truth_{col}"].values
        sm = aligned[f"sim_{col}"].values
        ax.plot(t, tr, "b-", linewidth=1.0, label=label_truth)
        ax.plot(t, sm, "r--", linewidth=1.0, label=label_sim)
        ax.set_ylabel(label)
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)

        m = metrics.get(col)
        if m:
            ax.text(0.02, 0.92, f"RMSE={m['RMSE']:.4f}",
                    transform=ax.transAxes, fontsize=9,
                    verticalalignment="top",
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="wheat", alpha=0.5))

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def load_csv(directory: str, name: str) -> pd.DataFrame | None:
    path = os.path.join(directory, name)
    if not os.path.isfile(path):
        print(f"  Warning: {path} not found, skipping.")
        return None
    return pd.read_csv(path)


DATASETS = [
    {
        "csv": "position.csv",
        "cols": ["x", "y", "z"],
        "labels": ["X (m)", "Y (m)", "Z (m)"],
        "ylabel": "Position",
        "title": "Position Comparison (NED)",
        "fig_name": "comparison_position.png",
    },
    {
        "csv": "velocity.csv",
        "cols": ["vx", "vy", "vz"],
        "labels": ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"],
        "ylabel": "Velocity",
        "title": "Velocity Comparison (NED)",
        "fig_name": "comparison_velocity.png",
    },
    {
        "csv": "attitude.csv",
        "cols": ["roll_deg", "pitch_deg", "yaw_deg"],
        "labels": ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"],
        "ylabel": "Attitude",
        "title": "Attitude Comparison",
        "fig_name": "comparison_attitude.png",
    },
    {
        "csv": "angular_velocity.csv",
        "cols": ["rollspeed", "pitchspeed", "yawspeed"],
        "labels": ["Roll Rate (rad/s)", "Pitch Rate (rad/s)", "Yaw Rate (rad/s)"],
        "ylabel": "Angular Velocity",
        "title": "Angular Velocity Comparison",
        "fig_name": "comparison_angular_velocity.png",
    },
]


def main():
    parser = argparse.ArgumentParser(description="Compare ground truth vs simulation flight data.")
    parser.add_argument("truth_dir", help="Directory with truth CSV files")
    parser.add_argument("sim_dir", help="Directory with simulation CSV files")
    parser.add_argument("--output-dir", default=None, help="Output directory (default: current dir)")
    parser.add_argument("--label-truth", default="Truth", help="Label for truth data")
    parser.add_argument("--label-sim", default="Simulation", help="Label for simulation data")
    args = parser.parse_args()

    for d in [args.truth_dir, args.sim_dir]:
        if not os.path.isdir(d):
            print(f"Error: directory not found: {d}", file=sys.stderr)
            sys.exit(1)

    output_dir = args.output_dir or "."
    os.makedirs(output_dir, exist_ok=True)

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
        "axes.unicode_minus": False,
    })

    truth_pos = load_csv(args.truth_dir, "position.csv")
    sim_pos = load_csv(args.sim_dir, "position.csv")
    t0_truth = detect_takeoff_time(truth_pos)
    t0_sim = detect_takeoff_time(sim_pos)
    print(f"Takeoff detected - Truth: {t0_truth:.2f}s, Sim: {t0_sim:.2f}s")

    all_metrics = {}

    for ds in DATASETS:
        truth_df = load_csv(args.truth_dir, ds["csv"])
        sim_df = load_csv(args.sim_dir, ds["csv"])
        if truth_df is None or sim_df is None:
            continue

        aligned = align_data(truth_df, sim_df, t0_truth, t0_sim, ds["cols"])
        if aligned is None:
            print(f"  Warning: no overlapping time range for {ds['csv']}, skipping.")
            continue

        ch_metrics = {}
        for col in ds["cols"]:
            m = compute_metrics(aligned[f"truth_{col}"].values, aligned[f"sim_{col}"].values)
            ch_metrics[col] = m
            all_metrics[col] = m

        plot_comparison(aligned, ds["cols"], ds["labels"], ds["ylabel"], ds["title"],
                        os.path.join(output_dir, ds["fig_name"]),
                        args.label_truth, args.label_sim, ch_metrics)

    metrics_path = os.path.join(output_dir, "metrics.json")
    with open(metrics_path, "w") as f:
        json.dump(all_metrics, f, indent=2)
    print(f"  Saved: {metrics_path}")

    print("\n" + "=" * 70)
    print(f"{'Channel':<18} {'RMSE':>10} {'MaxErr':>10} {'R²':>10}")
    print("-" * 70)
    for ch, m in all_metrics.items():
        print(f"{ch:<18} {m['RMSE']:>10.4f} {m['MaxErr']:>10.4f} {m['R2']:>10.4f}")
    print("=" * 70)


if __name__ == "__main__":
    main()
