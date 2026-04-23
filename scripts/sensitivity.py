#!/usr/bin/env python3
"""
Analyze parameter sensitivity from multiple experiment rounds.

Usage:
    python3 sensitivity.py <experiments_json>

Input format (experiments.json):
[
    {
        "round": 1,
        "label": "Initial guess (+15% mass, -18% thrust)",
        "params": {"mass": 2.3, "Ixx": 0.025, "Izz": 0.035, "motorConstant": 7.0e-06, "timeConstantUp": 0.020},
        "metrics_file": "results/round1/metrics.json"
    },
    {
        "round": 2,
        "label": "Fix mass → 2.0",
        "params": {"mass": 2.0, "Ixx": 0.025, "Izz": 0.035, "motorConstant": 7.0e-06, "timeConstantUp": 0.020},
        "metrics_file": "results/round2/metrics.json"
    }
]

Outputs:
    - convergence.png: RMSE convergence plot across rounds
    - sensitivity_table.txt: Which parameter fix reduced RMSE the most
"""

import argparse
import json
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

KEY_CHANNELS = {
    "z":         "Position Z",
    "roll_deg":  "Roll",
    "pitch_deg": "Pitch",
    "yaw_deg":   "Yaw",
}

COLORS = {
    "z":         "#2196F3",
    "roll_deg":  "#E91E63",
    "pitch_deg": "#FF9800",
    "yaw_deg":   "#4CAF50",
}


def load_experiments(json_path: str) -> list[dict]:
    base_dir = os.path.dirname(os.path.abspath(json_path))
    with open(json_path) as f:
        experiments = json.load(f)

    experiments.sort(key=lambda e: e["round"])

    for exp in experiments:
        mf = exp["metrics_file"]
        if not os.path.isabs(mf):
            mf = os.path.join(base_dir, mf)
        if not os.path.isfile(mf):
            print(f"Error: metrics file not found: {mf}", file=sys.stderr)
            sys.exit(1)
        with open(mf) as f:
            exp["metrics"] = json.load(f)

    return experiments


def extract_rmse_series(experiments: list[dict]) -> dict[str, list[float]]:
    series = {ch: [] for ch in KEY_CHANNELS}
    for exp in experiments:
        metrics = exp["metrics"]
        for ch in KEY_CHANNELS:
            if ch in metrics and "RMSE" in metrics[ch]:
                series[ch].append(metrics[ch]["RMSE"])
            else:
                series[ch].append(float("nan"))
    return series


def detect_changed_params(experiments: list[dict]) -> list[str]:
    """For each round (after the first), find which params changed vs previous round."""
    annotations = [""]
    for i in range(1, len(experiments)):
        prev_params = experiments[i - 1].get("params", {})
        curr_params = experiments[i].get("params", {})
        changed = []
        for key in set(prev_params) | set(curr_params):
            v_prev = prev_params.get(key)
            v_curr = curr_params.get(key)
            if v_prev != v_curr:
                changed.append(key)
        annotations.append(", ".join(changed) if changed else "?")
    return annotations


def plot_convergence(experiments: list[dict], rmse_series: dict[str, list[float]],
                     annotations: list[str], output_path: str):
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
        "axes.unicode_minus": False,
    })

    fig, ax = plt.subplots(figsize=(11, 6))
    rounds = [e["round"] for e in experiments]

    for ch, display in KEY_CHANNELS.items():
        values = rmse_series[ch]
        ax.plot(rounds, values, "o-", color=COLORS[ch], linewidth=2,
                markersize=7, label=display, zorder=3)

    y_max = 0.0
    for values in rmse_series.values():
        valid = [v for v in values if np.isfinite(v)]
        if valid:
            y_max = max(y_max, max(valid))

    for i, ann in enumerate(annotations):
        if i == 0 or not ann:
            continue
        composite = max(
            rmse_series[ch][i] for ch in KEY_CHANNELS if np.isfinite(rmse_series[ch][i])
        )
        ax.annotate(
            f"fix: {ann}",
            xy=(rounds[i], composite),
            xytext=(0, 18), textcoords="offset points",
            fontsize=8, ha="center", color="#555555",
            arrowprops=dict(arrowstyle="-", color="#AAAAAA", lw=0.8),
            bbox=dict(boxstyle="round,pad=0.25", fc="#FFFFDD", ec="#CCCCAA", alpha=0.85),
        )

    ax.set_xlabel("Experiment Round", fontsize=12)
    ax.set_ylabel("RMSE", fontsize=12)
    ax.set_title("Parameter Tuning Convergence", fontsize=14, fontweight="bold")
    ax.set_xticks(rounds)
    ax.set_xticklabels(
        [f"R{r}\n{experiments[i]['label'][:28]}" for i, r in enumerate(rounds)],
        fontsize=8,
    )
    ax.legend(loc="upper right", fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.set_ylim(bottom=0, top=y_max * 1.35 if y_max > 0 else 1.0)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def build_sensitivity_table(experiments: list[dict], rmse_series: dict[str, list[float]],
                            annotations: list[str]) -> list[dict]:
    rows = []
    for i in range(1, len(experiments)):
        param_changed = annotations[i]
        for ch, display in KEY_CHANNELS.items():
            prev = rmse_series[ch][i - 1]
            curr = rmse_series[ch][i]
            if not np.isfinite(prev) or not np.isfinite(curr):
                continue
            delta = curr - prev
            pct = (delta / prev * 100) if abs(prev) > 1e-12 else float("nan")
            rows.append({
                "round": experiments[i]["round"],
                "label": experiments[i]["label"],
                "param_changed": param_changed,
                "channel": display,
                "rmse_before": prev,
                "rmse_after": curr,
                "delta": delta,
                "pct_change": pct,
            })

    rows.sort(key=lambda r: r["delta"])
    return rows


def write_sensitivity_table(rows: list[dict], output_path: str):
    with open(output_path, "w") as f:
        header = (
            f"{'Round':>5}  {'Param Changed':<22}  {'Channel':<12}  "
            f"{'Before':>8}  {'After':>8}  {'Delta':>8}  {'%Change':>8}  Label"
        )
        sep = "=" * len(header)
        f.write("Parameter Sensitivity Analysis\n")
        f.write(f"(sorted by RMSE delta, negative = improvement)\n\n")
        f.write(f"{sep}\n{header}\n{sep}\n")

        for r in rows:
            line = (
                f"{r['round']:>5}  {r['param_changed']:<22}  {r['channel']:<12}  "
                f"{r['rmse_before']:>8.4f}  {r['rmse_after']:>8.4f}  {r['delta']:>+8.4f}  "
                f"{r['pct_change']:>+7.1f}%  {r['label']}"
            )
            f.write(f"{line}\n")

        f.write(f"{sep}\n")

        f.write("\nSummary (most impactful parameter fixes):\n")
        f.write("-" * 50 + "\n")
        seen = set()
        rank = 0
        for r in rows:
            if r["delta"] >= 0:
                break
            key = (r["round"], r["param_changed"])
            if key in seen:
                continue
            seen.add(key)
            rank += 1
            f.write(
                f"  {rank}. Round {r['round']}: fix {r['param_changed']:<18} "
                f"→ {r['channel']} RMSE {r['delta']:+.4f} ({r['pct_change']:+.1f}%)\n"
            )
        if rank == 0:
            f.write("  (no improvements detected)\n")

    print(f"  Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze parameter sensitivity from multi-round experiments."
    )
    parser.add_argument("experiments_json", help="Path to experiments.json")
    parser.add_argument(
        "--output-dir", default=None,
        help="Output directory (default: same dir as experiments.json)"
    )
    args = parser.parse_args()

    if not os.path.isfile(args.experiments_json):
        print(f"Error: file not found: {args.experiments_json}", file=sys.stderr)
        sys.exit(1)

    output_dir = args.output_dir or os.path.dirname(os.path.abspath(args.experiments_json))
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading experiments from: {args.experiments_json}")
    experiments = load_experiments(args.experiments_json)
    print(f"  Found {len(experiments)} rounds")

    rmse_series = extract_rmse_series(experiments)
    annotations = detect_changed_params(experiments)

    print("\nRMSE per round:")
    print(f"  {'Round':<6}", end="")
    for display in KEY_CHANNELS.values():
        print(f"  {display:>12}", end="")
    print()
    for i, exp in enumerate(experiments):
        print(f"  R{exp['round']:<5}", end="")
        for ch in KEY_CHANNELS:
            val = rmse_series[ch][i]
            print(f"  {val:>12.4f}" if np.isfinite(val) else f"  {'N/A':>12}", end="")
        print(f"  {exp['label']}")

    conv_path = os.path.join(output_dir, "convergence.png")
    plot_convergence(experiments, rmse_series, annotations, conv_path)

    rows = build_sensitivity_table(experiments, rmse_series, annotations)
    table_path = os.path.join(output_dir, "sensitivity_table.txt")
    write_sensitivity_table(rows, table_path)

    print("\nDone.")


if __name__ == "__main__":
    main()
