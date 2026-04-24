#!/usr/bin/env python3
"""
Overlay horizontal speed from two ULGs (e.g. Gobi v1 linear damp vs v2 v² drag).
"""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

plt.rcParams["font.size"] = 10


def speed_series(ulg_path):
    ulog = ULog(ulg_path)
    for d in ulog.data_list:
        if d.name == "vehicle_local_position":
            lp = d.data
            break
    else:
        return None, None, None
    t0 = lp["timestamp"][0]
    t = (lp["timestamp"] - t0) / 1e6
    vx = np.array(lp.get("vx", 0 * t))
    vy = np.array(lp.get("vy", 0 * t))
    vh = np.sqrt(vx**2 + vy**2)
    return t, vh, np.max(vh)


def main():
    if len(sys.argv) < 3:
        print("Usage: compare_gobi_v1_v2.py v1.ulg v2.ulg [out.png]")
        sys.exit(1)
    p1, p2 = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else None
    t1, v1, m1 = speed_series(p1)
    t2, v2, m2 = speed_series(p2)
    if t1 is None or t2 is None:
        print("Missing vehicle_local_position")
        sys.exit(1)

    out_dir = os.path.join(
        os.path.dirname(__file__), "..", "results", "gobi")
    os.makedirs(out_dir, exist_ok=True)
    if out is None:
        out = os.path.join(out_dir, "gobi_v1_vs_v2_speed.png")

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t1, v1, "b-", lw=0.8, alpha=0.9, label=f"v1 (velocity_decay)  max={m1:.1f} m/s")
    ax.plot(t2, v2, "r-", lw=0.8, alpha=0.9, label=f"v2 (QuadraticDrag) max={m2:.1f} m/s")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Horizontal speed (m/s)")
    ax.set_title("Gobi: linear damping vs v² body drag (same T/W, same mission)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")
    print(f"Max speed v1={m1:.2f} m/s  v2={m2:.2f} m/s  (delta {m1-m2:.2f} m/s)")


if __name__ == "__main__":
    main()
