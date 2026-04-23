#!/usr/bin/env python3
"""
Generate comprehensive convergence analysis for setpoint-replay 4-round experiment.
Produces multi-panel convergence charts covering position, velocity, attitude, angular rate.
"""

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")

ROUNDS = [1, 2, 3, 4]
ROUND_LABELS = [
    "R1: Initial\n(all deviated)",
    "R2: Fix mass",
    "R3: Fix motor\nConstant",
    "R4: Fix Ixx/Iyy\n+ timeConst",
]

VELOCITY_CHANNELS = {"vx": "Vx (m/s)", "vy": "Vy (m/s)", "vz": "Vz (m/s)"}
ATTITUDE_CHANNELS = {"roll_deg": "Roll (deg)", "pitch_deg": "Pitch (deg)", "yaw_deg": "Yaw (deg)"}
ANGVEL_CHANNELS = {"rollspeed": "Roll rate", "pitchspeed": "Pitch rate", "yawspeed": "Yaw rate"}
POSITION_CHANNELS = {"x": "X (m)", "y": "Y (m)", "z": "Z (m)"}


def load_metrics():
    all_metrics = {}
    for r in ROUNDS:
        mpath = os.path.join(RESULTS_DIR, f"sp_round{r}", "metrics.json")
        with open(mpath) as f:
            all_metrics[r] = json.load(f)
    return all_metrics


def plot_group(ax, all_metrics, channels, title, colors):
    for ch, label in channels.items():
        vals = [all_metrics[r][ch]["RMSE"] for r in ROUNDS]
        ax.plot(ROUNDS, vals, "o-", label=label, linewidth=2, markersize=7,
                color=colors[ch])
        for i, v in enumerate(vals):
            ax.annotate(f"{v:.3f}", (ROUNDS[i], v), textcoords="offset points",
                        xytext=(0, 8), fontsize=7, ha="center")
    ax.set_title(title, fontsize=11, fontweight="bold")
    ax.set_xticks(ROUNDS)
    ax.set_xticklabels(ROUND_LABELS, fontsize=7)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.set_ylabel("RMSE")
    ax.set_xlim(0.5, 4.5)


def main():
    all_metrics = load_metrics()

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans", "Arial"],
        "axes.unicode_minus": False,
    })

    vel_colors = {"vx": "#2196F3", "vy": "#4CAF50", "vz": "#FF9800"}
    att_colors = {"roll_deg": "#E91E63", "pitch_deg": "#9C27B0", "yaw_deg": "#607D8B"}
    ang_colors = {"rollspeed": "#00BCD4", "pitchspeed": "#795548", "yawspeed": "#FF5722"}
    pos_colors = {"x": "#3F51B5", "y": "#8BC34A", "z": "#F44336"}

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    plot_group(axes[0, 0], all_metrics, VELOCITY_CHANNELS,
              "Velocity RMSE Convergence", vel_colors)
    plot_group(axes[0, 1], all_metrics, ATTITUDE_CHANNELS,
              "Attitude RMSE Convergence", att_colors)
    plot_group(axes[1, 0], all_metrics, ANGVEL_CHANNELS,
              "Angular Velocity RMSE Convergence", ang_colors)
    plot_group(axes[1, 1], all_metrics, POSITION_CHANNELS,
              "Position RMSE Convergence (cumulative drift)", pos_colors)

    fig.suptitle("Setpoint Replay: 4-Round Parameter Tuning Convergence",
                 fontsize=14, fontweight="bold", y=1.01)
    fig.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "sp_convergence_full.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path}")

    # Also generate a summary table
    print("\n" + "=" * 90)
    print(f"{'Metric':<15}", end="")
    for rl in ["R1 (all dev)", "R2 (fix mass)", "R3 (fix motor)", "R4 (fix all)"]:
        print(f" {rl:>15}", end="")
    print(f" {'R1→R4 change':>14}")
    print("-" * 90)

    for group_name, channels in [("Position", POSITION_CHANNELS),
                                  ("Velocity", VELOCITY_CHANNELS),
                                  ("Attitude", ATTITUDE_CHANNELS),
                                  ("Ang.Vel.", ANGVEL_CHANNELS)]:
        for ch, label in channels.items():
            vals = [all_metrics[r][ch]["RMSE"] for r in ROUNDS]
            delta_pct = (vals[-1] - vals[0]) / vals[0] * 100
            print(f"{ch:<15}", end="")
            for v in vals:
                print(f" {v:>15.4f}", end="")
            print(f" {delta_pct:>+13.1f}%")
        print()
    print("=" * 90)

    print("\nKey findings:")
    for ch in ["vz", "pitch_deg", "yawspeed"]:
        r1 = all_metrics[1][ch]["RMSE"]
        r4 = all_metrics[4][ch]["RMSE"]
        pct = (r4 - r1) / r1 * 100
        print(f"  {ch}: {r1:.4f} → {r4:.4f} ({pct:+.1f}%)")


if __name__ == "__main__":
    main()
