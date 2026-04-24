#!/usr/bin/env python3
"""
Gobi interceptor multi-speed analysis:
- Extract vehicle_local_position, sensor_combined, actuator_outputs from ULG
- Per-speed-segment: identify k_f (thrust constant) and estimate drag coefficient
- Plot speed vs drag and v² deviation analysis
- Generate comparison figures for the article
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11

RESULT_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'gobi')
ULG_PATH = os.path.join(RESULT_DIR, 'gobi_multispeed.ulg')

MASS = 2.2
G = 9.81
N_MOTORS = 4
KF_TRUE = 3.746875e-05
MAX_ROT_VEL = 1200.0


def load_ulg(path):
    """Load ULG and extract relevant datasets."""
    ulog = ULog(path)
    datasets = {}
    for d in ulog.data_list:
        datasets[d.name] = pd.DataFrame(d.data)
    return datasets


def extract_data(datasets):
    """Extract and merge local position, sensor, actuator data."""
    lp = datasets.get('vehicle_local_position', None)
    if lp is None:
        print("ERROR: no vehicle_local_position")
        sys.exit(1)

    ao_key = None
    for k in ['actuator_outputs_sim', 'actuator_outputs']:
        if k in datasets:
            ao_key = k
            break
    ao = datasets.get(ao_key) if ao_key else None

    sc = datasets.get('sensor_combined', None)

    t0 = lp['timestamp'].iloc[0]
    lp['t'] = (lp['timestamp'] - t0) / 1e6

    result = pd.DataFrame()
    result['t'] = lp['t']
    result['vx'] = lp['vx'] if 'vx' in lp else 0
    result['vy'] = lp['vy'] if 'vy' in lp else 0
    result['vz'] = lp['vz'] if 'vz' in lp else 0
    result['speed'] = np.sqrt(result['vx']**2 + result['vy']**2)
    result['speed_3d'] = np.sqrt(result['vx']**2 + result['vy']**2 + result['vz']**2)
    result['ax'] = lp.get('ax', 0)
    result['ay'] = lp.get('ay', 0)
    result['az'] = lp.get('az', 0)
    result['x'] = lp.get('x', 0)
    result['y'] = lp.get('y', 0)
    result['z'] = lp.get('z', 0)

    if ao is not None:
        ao['t'] = (ao['timestamp'] - t0) / 1e6
        for i in range(N_MOTORS):
            col = f'output[{i}]'
            if col in ao.columns:
                result[f'motor_{i}'] = np.interp(result['t'], ao['t'], ao[col])
            else:
                result[f'motor_{i}'] = 0

    if sc is not None:
        sc['t'] = (sc['timestamp'] - t0) / 1e6
        for ax_name in ['accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]']:
            if ax_name in sc.columns:
                short = ax_name.replace('accelerometer_m_s2[', 'acc_').replace(']', '')
                result[short] = np.interp(result['t'], sc['t'], sc[ax_name])

    return result


def identify_segments(data, speed_thresholds):
    """Identify stable speed segments for each target speed."""
    segments = []
    for target in speed_thresholds:
        low = target * 0.7
        high = target * 1.3
        mask = (data['speed'] >= low) & (data['speed'] <= high)
        if mask.sum() < 20:
            continue
        indices = np.where(mask)[0]
        groups = np.split(indices, np.where(np.diff(indices) > 10)[0] + 1)
        best = max(groups, key=len) if groups else []
        if len(best) < 20:
            continue
        seg = data.iloc[best].copy()
        segments.append({
            'target': target,
            'data': seg,
            'mean_speed': seg['speed'].mean(),
            'mean_speed_3d': seg['speed_3d'].mean(),
        })
    return segments


def estimate_kf_per_segment(segments, data):
    """Estimate effective k_f per speed segment from motor RPM and acceleration."""
    results = []
    for seg in segments:
        d = seg['data']
        motors = []
        for i in range(N_MOTORS):
            col = f'motor_{i}'
            if col in d.columns:
                motors.append(d[col].values)
        if not motors:
            continue

        motor_arr = np.array(motors)
        mean_rpm = np.mean(motor_arr)
        mean_speed = seg['mean_speed']

        az_body = d.get('acc_2', d.get('az', pd.Series([0]))).mean()
        total_accel = np.sqrt(d['ax'].mean()**2 + d['ay'].mean()**2 + (d['az'].mean())**2)

        if mean_rpm > 100:
            omega = mean_rpm
            total_thrust_needed = MASS * np.sqrt(G**2 + total_accel**2)
            kf_est = total_thrust_needed / (N_MOTORS * omega**2)
        else:
            kf_est = np.nan

        ax_mean = d['ax'].mean()
        ay_mean = d['ay'].mean()
        horizontal_accel = np.sqrt(ax_mean**2 + ay_mean**2)

        results.append({
            'target_speed': seg['target'],
            'mean_speed': mean_speed,
            'mean_rpm': mean_rpm,
            'kf_est': kf_est,
            'kf_true': KF_TRUE,
            'kf_error_pct': abs(kf_est - KF_TRUE) / KF_TRUE * 100 if not np.isnan(kf_est) else np.nan,
            'horizontal_accel': horizontal_accel,
        })
    return pd.DataFrame(results)


def estimate_drag(data, segments):
    """Estimate drag behavior: compare linear vs quadratic models."""
    results = []
    rho = 1.225

    for seg in segments:
        d = seg['data']
        v = d['speed'].mean()
        if v < 2:
            continue

        ax_mean = d['ax'].mean()
        ay_mean = d['ay'].mean()

        vx = d['vx'].mean()
        vy = d['vy'].mean()
        v_hor = np.sqrt(vx**2 + vy**2)

        a_along = (ax_mean * vx + ay_mean * vy) / max(v_hor, 0.1)

        f_drag_est = -MASS * a_along

        linear_coeff = f_drag_est / max(v, 0.1) if v > 2 else np.nan
        cda_est = 2 * f_drag_est / (rho * v**2) if v > 2 else np.nan

        f_linear_model = 0.3 * v
        f_quad_model = 0.5 * rho * 0.020 * v**2

        results.append({
            'speed': v,
            'f_drag_est': f_drag_est,
            'linear_coeff': linear_coeff,
            'cda_est': cda_est,
            'f_linear_model': f_linear_model,
            'f_quad_model': f_quad_model,
        })
    return pd.DataFrame(results)


def plot_speed_profile(data, out_dir):
    """Plot full speed profile over time."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(data['t'], data['speed'], 'b-', linewidth=0.8, label='Horizontal speed')
    axes[0].plot(data['t'], data['speed_3d'], 'r--', linewidth=0.5, alpha=0.6, label='3D speed')
    axes[0].set_ylabel('Speed (m/s)')
    axes[0].legend()
    axes[0].set_title('Gobi Interceptor Multi-Speed Flight Profile')
    axes[0].grid(True, alpha=0.3)

    if 'motor_0' in data.columns:
        for i in range(N_MOTORS):
            axes[1].plot(data['t'], data[f'motor_{i}'], linewidth=0.5, alpha=0.7, label=f'Motor {i}')
        axes[1].set_ylabel('Motor command')
        axes[1].legend(fontsize=8)
        axes[1].grid(True, alpha=0.3)

    axes[2].plot(data['t'], data['ax'], 'r-', linewidth=0.5, alpha=0.7, label='ax')
    axes[2].plot(data['t'], data['ay'], 'g-', linewidth=0.5, alpha=0.7, label='ay')
    axes[2].plot(data['t'], data['az'], 'b-', linewidth=0.5, alpha=0.7, label='az')
    axes[2].set_ylabel('Acceleration (m/s²)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'gobi_speed_profile.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_kf_vs_speed(kf_df, out_dir):
    """Plot k_f estimation vs speed."""
    if kf_df.empty:
        print("No k_f data to plot")
        return
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(kf_df['mean_speed'], kf_df['kf_est'] * 1e5, 'bo-', markersize=8, label='Estimated $k_f$')
    ax.axhline(y=KF_TRUE * 1e5, color='r', linestyle='--', linewidth=2, label=f'True $k_f$ = {KF_TRUE:.4e}')
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('$k_f$ (×10⁻⁵)')
    ax.set_title('Thrust Constant $k_f$ Identification vs Flight Speed')
    ax.legend()
    ax.grid(True, alpha=0.3)

    for _, row in kf_df.iterrows():
        if not np.isnan(row['kf_error_pct']):
            ax.annotate(f"{row['kf_error_pct']:.1f}%",
                        (row['mean_speed'], row['kf_est'] * 1e5),
                        textcoords="offset points", xytext=(0, 12),
                        ha='center', fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'gobi_kf_vs_speed.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_drag_analysis(drag_df, out_dir):
    """Plot drag force: measured vs linear model vs quadratic model."""
    if drag_df.empty:
        print("No drag data to plot")
        return

    speeds = np.linspace(1, 100, 200)
    rho = 1.225
    CdA = 0.020

    f_linear = 0.3 * speeds
    f_quad = 0.5 * rho * CdA * speeds**2

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].plot(speeds, f_linear, 'b-', linewidth=2, label='Gazebo linear drag ($F = 0.3v$)')
    axes[0].plot(speeds, f_quad, 'r-', linewidth=2, label=f'Real quadratic drag ($C_dA={CdA}$)')
    if not drag_df.empty:
        axes[0].scatter(drag_df['speed'], drag_df['f_drag_est'].abs(),
                        c='green', s=80, zorder=5, label='Measured (from ULG)', edgecolors='black')
    axes[0].set_xlabel('Speed (m/s)')
    axes[0].set_ylabel('Drag Force (N)')
    axes[0].set_title('Drag Force: Linear vs Quadratic Model')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlim(0, 100)

    ratio = f_quad / np.maximum(f_linear, 0.01)
    axes[1].plot(speeds, ratio, 'k-', linewidth=2)
    axes[1].axhline(y=1, color='gray', linestyle=':', alpha=0.5)
    axes[1].fill_between(speeds, 1, ratio, where=ratio > 1, alpha=0.2, color='red',
                         label='Quadratic > Linear')
    axes[1].set_xlabel('Speed (m/s)')
    axes[1].set_ylabel('$F_{quad} / F_{linear}$')
    axes[1].set_title('Quadratic/Linear Drag Ratio')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim(0, 100)

    speed_marks = [10, 25, 50, 70, 97]
    for v in speed_marks:
        r = 0.5 * rho * CdA * v**2 / (0.3 * v)
        axes[1].annotate(f'{v}m/s: {r:.1f}×',
                         (v, r), textcoords="offset points",
                         xytext=(10, 5), fontsize=9)

    plt.tight_layout()
    path = os.path.join(out_dir, 'gobi_drag_analysis.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def plot_cda_vs_speed(drag_df, out_dir):
    """Plot estimated CdA vs speed showing linear model breakdown."""
    if drag_df.empty or 'cda_est' not in drag_df.columns:
        return
    valid = drag_df[drag_df['cda_est'].notna() & (drag_df['speed'] > 3)]
    if valid.empty:
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(valid['speed'], valid['cda_est'], c='blue', s=80, zorder=5, edgecolors='black')
    ax.axhline(y=0.020, color='r', linestyle='--', linewidth=2, label='Reference $C_dA$ = 0.020 m²')

    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('Estimated $C_dA$ (m²)')
    ax.set_title('Apparent $C_dA$ from Gazebo Simulation (Linear Drag Model)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, 'gobi_cda_vs_speed.png')
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved: {path}")


def main():
    os.makedirs(RESULT_DIR, exist_ok=True)

    print(f"Loading ULG: {ULG_PATH}")
    datasets = load_ulg(ULG_PATH)
    print(f"Available datasets: {list(datasets.keys())[:10]}")

    data = extract_data(datasets)
    print(f"Data shape: {data.shape}, time range: {data['t'].iloc[0]:.1f} - {data['t'].iloc[-1]:.1f} s")
    print(f"Max speed: {data['speed'].max():.1f} m/s, Max 3D speed: {data['speed_3d'].max():.1f} m/s")

    speed_targets = [5, 10, 15, 25, 40, 50, 70, 85, 97]
    segments = identify_segments(data, speed_targets)
    print(f"\nIdentified {len(segments)} speed segments:")
    for seg in segments:
        print(f"  Target={seg['target']} m/s, Mean={seg['mean_speed']:.1f} m/s, "
              f"Samples={len(seg['data'])}")

    kf_df = estimate_kf_per_segment(segments, data)
    if not kf_df.empty:
        print("\nk_f estimation results:")
        print(kf_df[['target_speed', 'mean_speed', 'mean_rpm', 'kf_est', 'kf_error_pct']].to_string(index=False))

    drag_df = estimate_drag(data, segments)
    if not drag_df.empty:
        print("\nDrag estimation results:")
        print(drag_df.to_string(index=False))

    plot_speed_profile(data, RESULT_DIR)
    plot_kf_vs_speed(kf_df, RESULT_DIR)
    plot_drag_analysis(drag_df, RESULT_DIR)
    plot_cda_vs_speed(drag_df, RESULT_DIR)

    summary_path = os.path.join(RESULT_DIR, 'gobi_summary.csv')
    if not kf_df.empty:
        kf_df.to_csv(summary_path, index=False)
        print(f"\nSummary saved: {summary_path}")

    print("\nAnalysis complete!")


if __name__ == "__main__":
    main()
