#!/usr/bin/env python3
"""
Comprehensive analysis of system identification results.

Generates:
1. k_f identification accuracy at different conditions
2. Speed-dependent drag analysis
3. Comparison charts
4. Summary table

Usage:
    python3 analyze_sysid_results.py <ulg_file> [--output-dir results/sysid_analysis]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt
from scipy.optimize import curve_fit
from pyulog import ULog
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def extract_data(ulg_path):
    ulog = ULog(str(ulg_path))
    data = {d.name: d.data for d in ulog.data_list}
    t0 = min(d.data["timestamp"][0] for d in ulog.data_list if len(d.data["timestamp"]) > 0)

    result = {}

    for topic, fields in [
        ("vehicle_acceleration", ["xyz[0]", "xyz[1]", "xyz[2]"]),
        ("vehicle_angular_velocity", ["xyz[0]", "xyz[1]", "xyz[2]"]),
        ("actuator_motors", [f"control[{i}]" for i in range(4)]),
        ("vehicle_local_position", ["x", "y", "z", "vx", "vy", "vz"]),
        ("vehicle_attitude", ["q[0]", "q[1]", "q[2]", "q[3]"]),
    ]:
        if topic in data:
            d = data[topic]
            t = (d["timestamp"] - t0) / 1e6
            vals = {f: d[f] for f in fields if f in d}
            result[topic] = {"t": t, **vals}

    return result


def analyze_speed_segments(data, speed_thresholds=[2, 5, 10, 15, 20]):
    """Analyze acceleration at different speed ranges."""
    vd = data.get("vehicle_local_position")
    ad = data.get("vehicle_acceleration")
    md = data.get("actuator_motors")
    if vd is None or ad is None or md is None:
        return {}

    # Interpolate to common time
    dt = 0.01
    t_min = max(vd["t"][0], ad["t"][0], md["t"][0])
    t_max = min(vd["t"][-1], ad["t"][-1], md["t"][-1])
    t = np.arange(t_min + 1, t_max - 1, dt)

    vx = np.interp(t, vd["t"], vd["vx"])
    vy = np.interp(t, vd["t"], vd["vy"])
    vz = np.interp(t, vd["t"], vd["vz"])
    v_hor = np.sqrt(vx**2 + vy**2)

    az = np.interp(t, ad["t"], ad["xyz[2]"])
    motors = np.column_stack([
        np.interp(t, md["t"], md[f"control[{i}]"]) for i in range(4)
    ])
    motor_mean = np.mean(motors, axis=1)

    results = {}
    prev = 0
    for thr in speed_thresholds:
        mask = (v_hor >= prev) & (v_hor < thr) & (motor_mean > 0.3)
        if np.sum(mask) > 50:
            results[f"{prev}-{thr}"] = {
                "v_mean": float(np.mean(v_hor[mask])),
                "v_max": float(np.max(v_hor[mask])),
                "az_mean": float(np.mean(az[mask])),
                "motor_mean": float(np.mean(motor_mean[mask])),
                "motor_std": float(np.std(motor_mean[mask])),
                "n_samples": int(np.sum(mask)),
            }
        prev = thr

    # Also check beyond max threshold
    mask = (v_hor >= speed_thresholds[-1]) & (motor_mean > 0.3)
    if np.sum(mask) > 50:
        results[f">{speed_thresholds[-1]}"] = {
            "v_mean": float(np.mean(v_hor[mask])),
            "v_max": float(np.max(v_hor[mask])),
            "az_mean": float(np.mean(az[mask])),
            "motor_mean": float(np.mean(motor_mean[mask])),
            "motor_std": float(np.std(motor_mean[mask])),
            "n_samples": int(np.sum(mask)),
        }

    return {"speed_segments": results, "max_speed": float(np.max(v_hor))}


def identify_kf_multispeed(data, omega_max=1000.0, mass=2.0):
    """Identify k_f at different speed levels."""
    vd = data.get("vehicle_local_position")
    ad = data.get("vehicle_acceleration")
    md = data.get("actuator_motors")
    if vd is None or ad is None or md is None:
        return {}

    dt = 0.004
    t_min = max(vd["t"][0], ad["t"][0], md["t"][0])
    t_max = min(vd["t"][-1], ad["t"][-1], md["t"][-1])
    t = np.arange(t_min + 1, t_max - 1, dt)
    fs = 1.0 / dt

    vx = np.interp(t, vd["t"], vd["vx"])
    vy = np.interp(t, vd["t"], vd["vy"])
    v_hor = np.sqrt(vx**2 + vy**2)

    az = np.interp(t, ad["t"], ad["xyz[2]"])
    motors = np.column_stack([
        np.interp(t, md["t"], md[f"control[{i}]"]) for i in range(4)
    ])
    motor_mean = np.mean(motors, axis=1)

    b, a = butter(4, 30/(fs/2), 'low')
    az_f = filtfilt(b, a, az)

    omega_sq_sum = np.sum((motors * omega_max)**2, axis=1)

    results = {}
    for v_lo, v_hi in [(0, 2), (2, 5), (5, 10), (10, 15), (15, 25), (25, 50)]:
        mask = (v_hor >= v_lo) & (v_hor < v_hi) & (motor_mean > 0.3)
        if np.sum(mask) > 100:
            kf = -mass * np.mean(az_f[mask]) / np.mean(omega_sq_sum[mask])
            rmse = np.sqrt(np.mean(
                (-kf * omega_sq_sum[mask] / mass - az_f[mask])**2))
            results[f"{v_lo}-{v_hi}m/s"] = {
                "k_f": float(kf),
                "rmse": float(rmse),
                "n_samples": int(np.sum(mask)),
                "v_mean": float(np.mean(v_hor[mask])),
            }

    # Overall
    mask = motor_mean > 0.3
    kf_all = -mass * np.mean(az_f[mask]) / np.mean(omega_sq_sum[mask])
    results["overall"] = {
        "k_f": float(kf_all),
        "n_samples": int(np.sum(mask)),
    }

    return results


def estimate_drag_coefficient(data, k_f=8.54858e-6, omega_max=1000.0, mass=2.0):
    """
    Estimate body drag coefficient from the difference between
    predicted z-accel (from motors) and measured z-accel at speed.

    At speed, extra motor thrust is needed to overcome drag:
    F_drag = F_thrust_horizontal = F_total * sin(tilt)
    We can estimate drag as the excess thrust beyond hover.
    """
    vd = data.get("vehicle_local_position")
    ad = data.get("vehicle_acceleration")
    md = data.get("actuator_motors")
    if vd is None or ad is None or md is None:
        return {}

    dt = 0.01
    t_min = max(vd["t"][0], ad["t"][0], md["t"][0])
    t_max = min(vd["t"][-1], ad["t"][-1], md["t"][-1])
    t = np.arange(t_min + 1, t_max - 1, dt)

    vx = np.interp(t, vd["t"], vd["vx"])
    vy = np.interp(t, vd["t"], vd["vy"])
    v_hor = np.sqrt(vx**2 + vy**2)

    motors = np.column_stack([
        np.interp(t, md["t"], md[f"control[{i}]"]) for i in range(4)
    ])
    motor_mean = np.mean(motors, axis=1)

    # Total thrust
    omega = motors * omega_max
    F_total = np.sum(k_f * omega**2, axis=1)

    # At hover, F_total = mg. Extra thrust goes to overcoming drag.
    F_excess = F_total - mass * 9.81

    # Drag model: F_drag = Cd * v^2 (quadratic) or Cd_lin * v (linear)
    mask = (v_hor > 1.0) & (motor_mean > 0.3) & (F_excess > 0)
    if np.sum(mask) < 50:
        return {"error": "Not enough high-speed data"}

    v = v_hor[mask]
    F = F_excess[mask]

    # Quadratic fit: F = a * v^2 + b
    try:
        def drag_model(v, Cd, offset):
            return Cd * v**2 + offset
        popt, _ = curve_fit(drag_model, v, F, p0=[0.1, 0])
        Cd_quad = popt[0]
    except Exception:
        Cd_quad = 0.0

    # Linear fit: F = c * v + d
    try:
        coeffs = np.polyfit(v, F, 1)
        Cd_lin = coeffs[0]
    except Exception:
        Cd_lin = 0.0

    return {
        "Cd_quadratic": float(Cd_quad),
        "Cd_linear": float(Cd_lin),
        "max_speed": float(np.max(v)),
        "n_points": int(np.sum(mask)),
    }


def generate_plots(data, out_dir, ulg_name):
    """Generate analysis plots."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    vd = data.get("vehicle_local_position")
    ad = data.get("vehicle_acceleration")
    md = data.get("actuator_motors")
    gd = data.get("vehicle_angular_velocity")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"System Identification Analysis: {ulg_name}", fontsize=14)

    # 1. Speed profile
    if vd:
        t = vd["t"]
        vx, vy = vd["vx"], vd["vy"]
        v_hor = np.sqrt(vx**2 + vy**2)
        axes[0, 0].plot(t, v_hor, 'b-', linewidth=0.5)
        axes[0, 0].set_title("Horizontal Speed")
        axes[0, 0].set_ylabel("Speed (m/s)")
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].grid(True, alpha=0.3)

    # 2. Motor commands
    if md:
        t = md["t"]
        for i in range(4):
            axes[0, 1].plot(t, md[f"control[{i}]"], linewidth=0.3, label=f"M{i}")
        axes[0, 1].set_title("Motor Commands")
        axes[0, 1].set_ylabel("Command [0-1]")
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].legend(fontsize=8)
        axes[0, 1].grid(True, alpha=0.3)

    # 3. Z-acceleration
    if ad:
        t = ad["t"]
        axes[1, 0].plot(t, ad["xyz[2]"], 'r-', linewidth=0.3)
        axes[1, 0].axhline(y=-9.81, color='k', linestyle='--', alpha=0.5, label='g')
        axes[1, 0].set_title("Z-Acceleration (Specific Force)")
        axes[1, 0].set_ylabel("az (m/s²)")
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

    # 4. Angular rates
    if gd:
        t = gd["t"]
        for i, name in enumerate(["p (roll)", "q (pitch)", "r (yaw)"]):
            axes[1, 1].plot(t, np.degrees(gd[f"xyz[{i}]"]),
                           linewidth=0.3, label=name)
        axes[1, 1].set_title("Angular Rates")
        axes[1, 1].set_ylabel("Rate (deg/s)")
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].legend(fontsize=8)
        axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_dir / "sysid_overview.png", dpi=150)
    plt.close()

    # k_f vs speed plot
    if vd and ad and md:
        fig2, ax2 = plt.subplots(1, 1, figsize=(8, 5))
        kf_results = identify_kf_multispeed(data)
        speeds = []
        kfs = []
        truth_kf = 8.54858e-6
        for key, val in kf_results.items():
            if key != "overall" and val["n_samples"] > 50:
                speeds.append(val["v_mean"])
                kfs.append(val["k_f"])

        if speeds:
            ax2.scatter(speeds, [k*1e6 for k in kfs], s=60, zorder=5)
            ax2.axhline(y=truth_kf*1e6, color='r', linestyle='--',
                       label=f'Truth: {truth_kf*1e6:.3f}')
            ax2.set_xlabel("Mean Speed (m/s)")
            ax2.set_ylabel("Identified k_f (×10⁻⁶)")
            ax2.set_title("k_f Identification vs Speed")
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig(out_dir / "kf_vs_speed.png", dpi=150)
        plt.close()

    print(f"Plots saved to {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Sysid analysis")
    parser.add_argument("ulg_file", help="PX4 ULG file")
    parser.add_argument("--output-dir", default="results/sysid_analysis")
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--omega-max", type=float, default=1000.0)
    args = parser.parse_args()

    ulg_path = Path(args.ulg_file)
    if not ulg_path.is_file():
        print(f"Error: {ulg_path} not found", file=sys.stderr)
        return 1

    print(f"Analyzing: {ulg_path}")
    data = extract_data(ulg_path)

    print("\n═══ Speed Segment Analysis ═══")
    speed_analysis = analyze_speed_segments(data)
    if "speed_segments" in speed_analysis:
        print(f"Max speed reached: {speed_analysis['max_speed']:.1f} m/s")
        for seg, vals in speed_analysis["speed_segments"].items():
            print(f"  {seg:>8s} m/s: v_mean={vals['v_mean']:.1f}, "
                  f"motor={vals['motor_mean']:.3f}±{vals['motor_std']:.3f}, "
                  f"az={vals['az_mean']:.2f}, n={vals['n_samples']}")

    print("\n═══ k_f vs Speed ═══")
    kf_results = identify_kf_multispeed(data, args.omega_max, args.mass)
    truth_kf = 8.54858e-6
    for seg, vals in kf_results.items():
        kf = vals["k_f"]
        err = (kf - truth_kf) / truth_kf * 100
        print(f"  {seg:>12s}: k_f={kf:.6e} (err: {err:+.1f}%), n={vals['n_samples']}")

    print("\n═══ Drag Estimation ═══")
    drag = estimate_drag_coefficient(data, truth_kf, args.omega_max, args.mass)
    if "error" not in drag:
        print(f"  Quadratic Cd: {drag['Cd_quadratic']:.4f}")
        print(f"  Linear Cd: {drag['Cd_linear']:.4f}")
        print(f"  Max speed: {drag['max_speed']:.1f} m/s")
    else:
        print(f"  {drag['error']}")

    print("\n═══ Generating Plots ═══")
    generate_plots(data, args.output_dir, ulg_path.stem)

    # Save summary
    summary = {
        "ulg": str(ulg_path),
        "speed_analysis": speed_analysis,
        "kf_results": kf_results,
        "drag": drag,
    }
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "sysid_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Summary saved to {out_dir / 'sysid_summary.json'}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
