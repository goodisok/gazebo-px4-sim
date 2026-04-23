#!/usr/bin/env python3
"""
IMU-based System Identification for Quadrotor Dynamics (Two-Stage).

Stage 1: Identify k_f (thrust coefficient) from z-axis acceleration.
Stage 2: Identify Ixx, Iyy, Izz, k_m, tau from angular dynamics.

Works from a single PX4 ULG log using only onboard sensor data.

Usage:
    python3 imu_sysid.py <ulg_file> [--mass 2.0] [--arm-length 0.175]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize, differential_evolution, least_squares
from scipy.signal import butter, filtfilt
from pyulog import ULog


DEFAULT_ARM_LENGTH = 0.175
ROTOR_POSITIONS = np.array([
    [ 0.175,  0.175, 0.0],
    [-0.175, -0.175, 0.0],
    [ 0.175, -0.175, 0.0],
    [-0.175,  0.175, 0.0],
])
ROTOR_DIRECTIONS = np.array([1, 1, -1, -1])  # +1=CW, -1=CCW


def quat_to_rotmat(q0, q1, q2, q3):
    """Quaternion (w,x,y,z) to rotation matrix (body→world)."""
    n = len(q0)
    R = np.zeros((n, 3, 3))
    R[:, 0, 0] = 1 - 2*(q2**2 + q3**2)
    R[:, 0, 1] = 2*(q1*q2 - q0*q3)
    R[:, 0, 2] = 2*(q1*q3 + q0*q2)
    R[:, 1, 0] = 2*(q1*q2 + q0*q3)
    R[:, 1, 1] = 1 - 2*(q1**2 + q3**2)
    R[:, 1, 2] = 2*(q2*q3 - q0*q1)
    R[:, 2, 0] = 2*(q1*q3 - q0*q2)
    R[:, 2, 1] = 2*(q2*q3 + q0*q1)
    R[:, 2, 2] = 1 - 2*(q1**2 + q2**2)
    return R


def lowpass(data, cutoff_hz, fs, order=4):
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        return data
    b, a = butter(order, cutoff_hz / nyq, btype='low')
    return filtfilt(b, a, data, axis=0)


def extract_flight_data(ulg_path, motor_count=4):
    ulog = ULog(ulg_path)
    data = {}
    for d in ulog.data_list:
        data[d.name] = d.data

    t0 = min(d.data["timestamp"][0] for d in ulog.data_list
             if len(d.data["timestamp"]) > 0)

    # Accelerometer
    accel_d = data.get("vehicle_acceleration") or data.get("sensor_accel")
    if accel_d is None:
        raise ValueError("No accelerometer data in ULG")
    t_accel = (accel_d["timestamp"] - t0) / 1e6
    ax = accel_d.get("xyz[0]", accel_d.get("x"))
    ay = accel_d.get("xyz[1]", accel_d.get("y"))
    az = accel_d.get("xyz[2]", accel_d.get("z"))

    # Gyroscope
    gyro_d = data["vehicle_angular_velocity"]
    t_gyro = (gyro_d["timestamp"] - t0) / 1e6
    gx, gy, gz = gyro_d["xyz[0]"], gyro_d["xyz[1]"], gyro_d["xyz[2]"]

    # Attitude
    att_d = data["vehicle_attitude"]
    t_att = (att_d["timestamp"] - t0) / 1e6
    q0, q1, q2, q3 = att_d["q[0]"], att_d["q[1]"], att_d["q[2]"], att_d["q[3]"]

    # Motors
    motor_d = data.get("actuator_motors") or data.get("actuator_outputs")
    t_motor = (motor_d["timestamp"] - t0) / 1e6
    motors = np.zeros((len(t_motor), motor_count))
    if "control[0]" in motor_d:
        for i in range(motor_count):
            motors[:, i] = np.clip(motor_d[f"control[{i}]"], 0, 1)
    else:
        for i in range(motor_count):
            raw = motor_d[f"output[{i}]"]
            motors[:, i] = np.clip((raw - 150) / (1000 - 150), 0, 1)

    # Velocity (NED)
    vel_d = data.get("vehicle_local_position")
    t_vel, vx, vy, vz = None, None, None, None
    if vel_d is not None:
        t_vel = (vel_d["timestamp"] - t0) / 1e6
        vx, vy, vz = vel_d["vx"], vel_d["vy"], vel_d["vz"]

    return {
        "t_accel": t_accel, "ax": ax, "ay": ay, "az": az,
        "t_gyro": t_gyro, "gx": gx, "gy": gy, "gz": gz,
        "t_att": t_att, "q0": q0, "q1": q1, "q2": q2, "q3": q3,
        "t_motor": t_motor, "motors": motors,
        "t_vel": t_vel, "vx": vx, "vy": vy, "vz": vz,
    }


def synchronize(fd, dt=0.004):
    """Interpolate all to 250Hz common timeline."""
    t_min = max(fd["t_accel"][0], fd["t_gyro"][0],
                fd["t_att"][0], fd["t_motor"][0])
    t_max = min(fd["t_accel"][-1], fd["t_gyro"][-1],
                fd["t_att"][-1], fd["t_motor"][-1])
    t = np.arange(t_min + 0.5, t_max - 0.5, dt)
    n = len(t)

    accel = np.column_stack([np.interp(t, fd["t_accel"], fd[k])
                             for k in ["ax", "ay", "az"]])
    gyro = np.column_stack([np.interp(t, fd["t_gyro"], fd[k])
                            for k in ["gx", "gy", "gz"]])
    q = np.column_stack([np.interp(t, fd["t_att"], fd[k])
                         for k in ["q0", "q1", "q2", "q3"]])
    q /= np.linalg.norm(q, axis=1, keepdims=True)

    motors = np.column_stack([np.interp(t, fd["t_motor"], fd["motors"][:, i])
                               for i in range(fd["motors"].shape[1])])

    vel_body = None
    if fd["t_vel"] is not None:
        vel_w = np.column_stack([np.interp(t, fd["t_vel"], fd[k])
                                 for k in ["vx", "vy", "vz"]])
        R = quat_to_rotmat(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
        vel_body = np.einsum("nij,nj->ni", np.transpose(R, (0, 2, 1)), vel_w)

    fs = 1.0 / dt
    accel = lowpass(accel, 40.0, fs)
    gyro = lowpass(gyro, 40.0, fs)
    gyro_dot = np.gradient(gyro, dt, axis=0)
    gyro_dot = lowpass(gyro_dot, 25.0, fs)

    return {"t": t, "dt": dt, "fs": fs, "accel": accel, "gyro": gyro,
            "gyro_dot": gyro_dot, "q": q, "motors": motors,
            "vel_body": vel_body}


def detect_flight(sd, min_throttle=0.15):
    """Detect the flight segment with actual maneuver excitation."""
    m = np.mean(sd["motors"], axis=1)
    flying = m > min_throttle
    idx = np.where(flying)[0]
    if len(idx) < 100:
        return 0, len(sd["t"])

    # Find segments with significant angular activity
    # Use a sliding window to detect regions with high gyro variance
    window = int(2.0 / sd["dt"])
    gyro_mag = np.sqrt(np.sum(sd["gyro"]**2, axis=1))
    n = len(gyro_mag)
    local_std = np.zeros(n)
    for i in range(window, n - window):
        local_std[i] = np.std(gyro_mag[i-window:i+window])

    # Threshold: regions with angular rate std > 0.05 rad/s (3 deg/s)
    excited = (local_std > 0.05) & flying
    excited_idx = np.where(excited)[0]

    if len(excited_idx) > 200:
        # Use the excited region with some padding
        margin = int(1.0 / sd["dt"])
        i0 = max(excited_idx[0] - margin, idx[0])
        i1 = min(excited_idx[-1] + margin, idx[-1])
        return int(i0), int(i1)

    margin = 25
    return int(idx[0] + margin), int(idx[-1] - margin)


def motor_model(cmds, omega_max, tau, dt):
    """First-order motor lag: command → angular velocity."""
    n = cmds.shape[0]
    omega = np.zeros_like(cmds)
    target = cmds * omega_max
    alpha = dt / (tau + dt)
    for i in range(1, n):
        omega[i] = omega[i-1] + alpha * (target[i] - omega[i-1])
    return omega


# ────────── Stage 1: Thrust Coefficient ──────────

def stage1_identify_kf(sd, i0, i1, mass, omega_max, verbose=True):
    """
    Identify k_f from Z-axis specific force.
    Accel_z = -sum(k_f * omega_i^2) / m  (body-Z, FRD convention)
    """
    accel_z = sd["accel"][i0:i1, 2]
    motors = sd["motors"][i0:i1]
    dt = sd["dt"]

    best_kf, best_tau, best_err = None, None, np.inf

    for tau_try in np.arange(0.005, 0.060, 0.002):
        omega = motor_model(motors, omega_max, tau_try, dt)
        omega_sq_sum = np.sum(omega**2, axis=1)

        # accel_z = -k_f * omega_sq_sum / mass
        # k_f = -mass * accel_z / omega_sq_sum
        mask = omega_sq_sum > 100
        kf_est = -mass * np.mean(accel_z[mask]) / np.mean(omega_sq_sum[mask])

        pred = -kf_est * omega_sq_sum / mass
        err = np.sqrt(np.mean((pred[mask] - accel_z[mask])**2))
        if err < best_err:
            best_err = err
            best_kf = kf_est
            best_tau = tau_try

    if verbose:
        truth_kf = 8.54858e-6
        print(f"  Stage 1 Result: k_f = {best_kf:.6e} "
              f"(truth: {truth_kf:.6e}, err: {(best_kf-truth_kf)/truth_kf*100:+.1f}%)")
        print(f"  Best tau_motor = {best_tau:.4f}s, RMSE_z = {best_err:.4f} m/s²")
    return best_kf, best_tau, best_err


# ────────── Stage 2: Inertia via Linear Regression ──────────

def stage2_identify_inertia(sd, i0, i1, k_f, tau, omega_max,
                            rotor_pos, rotor_dirs, verbose=True):
    """
    Identify Ixx, Iyy, Izz, k_m from angular dynamics.
    
    I * alpha = tau_motors - omega x (I * omega)
    
    Rearranged for linear regression on [1/Ixx, 1/Iyy, 1/Izz, k_m/Ixx, ...].
    We use an iterative approach: given k_m guess, solve for I, then refine.
    """
    dt = sd["dt"]
    motors = sd["motors"][i0:i1]
    gyro = sd["gyro"][i0:i1]
    alpha = sd["gyro_dot"][i0:i1]
    n = len(gyro)

    omega_m = motor_model(motors, omega_max, tau, dt)
    thrust_per = k_f * omega_m**2  # (n, 4) per-rotor thrust

    # Torques from thrust (geometry)
    tau_x_thrust = np.zeros(n)
    tau_y_thrust = np.zeros(n)
    for i in range(4):
        tau_x_thrust += rotor_pos[i, 1] * (-thrust_per[:, i])
        tau_y_thrust += -rotor_pos[i, 0] * (-thrust_per[:, i])

    omega_sq = omega_m**2

    best_cost = np.inf
    best_params = None

    for km_ratio in np.arange(0.005, 0.040, 0.001):
        k_m = k_f * km_ratio
        tau_z_reaction = np.sum(rotor_dirs[:, None].T * k_m * omega_sq, axis=1)

        tau_total = np.column_stack([tau_x_thrust, tau_y_thrust, tau_z_reaction])

        p, q_g, r = gyro[:, 0], gyro[:, 1], gyro[:, 2]
        cross_x = (q_g * r)
        cross_y = (p * r)
        cross_z = (p * q_g)

        # For each axis: I_ii * alpha_i = tau_i - (I_kk - I_jj)*omega_j*omega_k
        # Simplified OLS for each axis independently:
        # alpha_x = tau_x / Ixx - (Izz - Iyy)/Ixx * q*r
        # With 3 unknowns (Ixx, Iyy, Izz), we build a joint system:

        # x-axis: Ixx*alpha_x = tau_x - (Izz - Iyy)*q*r
        # y-axis: Iyy*alpha_y = tau_y - (Ixx - Izz)*p*r
        # z-axis: Izz*alpha_z = tau_z - (Iyy - Ixx)*p*q

        # Joint nonlinear solve with least_squares
        def residuals(params):
            Ixx, Iyy, Izz = params
            rx = Ixx * alpha[:, 0] - tau_total[:, 0] + (Izz - Iyy) * cross_x
            ry = Iyy * alpha[:, 1] - tau_total[:, 1] + (Ixx - Izz) * cross_y
            rz = Izz * alpha[:, 2] - tau_total[:, 2] + (Iyy - Ixx) * cross_z
            return np.concatenate([rx, ry, rz])

        res = least_squares(residuals, [0.025, 0.025, 0.05],
                            bounds=([0.005, 0.005, 0.01], [0.15, 0.15, 0.3]),
                            method='trf')
        cost = res.cost
        if cost < best_cost:
            best_cost = cost
            best_params = {"Ixx": res.x[0], "Iyy": res.x[1], "Izz": res.x[2],
                           "k_m": k_m, "km_ratio": km_ratio}

    if verbose:
        truth = {"Ixx": 0.02167, "Iyy": 0.02167, "Izz": 0.04}
        print(f"  Stage 2 Results (km_ratio={best_params['km_ratio']:.3f}):")
        for k in ["Ixx", "Iyy", "Izz"]:
            v = best_params[k]
            t = truth[k]
            print(f"    {k} = {v:.6f}  (truth: {t:.6f}, err: {(v-t)/t*100:+.1f}%)")
        print(f"    k_m = {best_params['k_m']:.6e}")
        print(f"    cost = {best_cost:.4f}")
    return best_params, best_cost


# ────────── Stage 3 (Optional): Joint Refinement ──────────

def stage3_refine(sd, i0, i1, initial, mass, omega_max,
                  rotor_pos, rotor_dirs, include_drag=False, verbose=True):
    """
    Joint nonlinear refinement with tau grid search.
    
    tau and inertia are coupled (both affect angular acceleration amplitude).
    We break this degeneracy by fixing tau on a grid and optimizing the rest.
    """
    dt = sd["dt"]
    motors = sd["motors"][i0:i1]
    accel = sd["accel"][i0:i1]
    gyro = sd["gyro"][i0:i1]
    alpha_meas = sd["gyro_dot"][i0:i1]

    accel_std = max(np.std(accel[:, 2]), 0.1)
    alpha_stds = np.maximum(np.std(alpha_meas, axis=0), 0.01)

    def make_cost(tau_fixed):
        omega_cached = motor_model(motors, omega_max, tau_fixed, dt)
        
        def cost_fn(params):
            k_f = params[0]
            k_m = params[1]
            Ixx, Iyy, Izz = params[2], params[3], params[4]

            thrust = k_f * omega_cached**2
            torque_r = k_m * omega_cached**2

            pred_az = -np.sum(thrust, axis=1) / mass
            err_az = (pred_az - accel[:, 2]) / accel_std

            tau_x = sum(rotor_pos[j, 1] * (-thrust[:, j]) for j in range(4))
            tau_y = sum(-rotor_pos[j, 0] * (-thrust[:, j]) for j in range(4))
            tau_z = sum(rotor_dirs[j] * torque_r[:, j] for j in range(4))

            p, q_g, r = gyro[:, 0], gyro[:, 1], gyro[:, 2]
            err_alpha_x = ((tau_x - (Izz - Iyy)*q_g*r)/Ixx - alpha_meas[:, 0]) / alpha_stds[0]
            err_alpha_y = ((tau_y - (Ixx - Izz)*p*r)/Iyy - alpha_meas[:, 1]) / alpha_stds[1]
            err_alpha_z = ((tau_z - (Iyy - Ixx)*p*q_g)/Izz - alpha_meas[:, 2]) / alpha_stds[2]

            return (np.mean(err_az**2) +
                    np.mean(err_alpha_x**2) +
                    np.mean(err_alpha_y**2) +
                    np.mean(err_alpha_z**2))
        return cost_fn

    bounds_5 = [
        (3e-6, 3e-5),     # k_f
        (1e-8, 5e-6),     # k_m
        (0.008, 0.08),    # Ixx
        (0.008, 0.08),    # Iyy
        (0.015, 0.15),    # Izz
    ]

    tau_grid = [0.0, 0.005, 0.010, 0.0125, 0.015, 0.020, 0.030, 0.050]
    best_overall_cost = np.inf
    best_result = None
    best_tau = None

    if verbose:
        print(f"\n  Stage 3: Grid search over tau ({len(tau_grid)} values)...")

    for tau_val in tau_grid:
        cost_fn = make_cost(tau_val)
        de_res = differential_evolution(
            cost_fn, bounds_5, maxiter=150, tol=1e-10,
            seed=42, polish=True, disp=False)
        if verbose:
            print(f"    tau={tau_val:.4f}: cost={de_res.fun:.6f}  "
                  f"k_f={de_res.x[0]:.4e} Ixx={de_res.x[2]:.5f} "
                  f"Iyy={de_res.x[3]:.5f} Izz={de_res.x[4]:.5f}")
        if de_res.fun < best_overall_cost:
            best_overall_cost = de_res.fun
            best_result = de_res.x
            best_tau = tau_val

    names = ["k_f", "k_m", "Ixx", "Iyy", "Izz"]
    refined = {n: float(v) for n, v in zip(names, best_result)}
    refined["tau"] = best_tau
    refined["cost"] = float(best_overall_cost)

    if verbose:
        truth = {"k_f": 8.54858e-6, "k_m": 1.36777e-7,
                 "Ixx": 0.02167, "Iyy": 0.02167, "Izz": 0.04, "tau": 0.0125}
        print(f"\n  Best tau = {best_tau:.4f}s")
        print(f"  Stage 3 Results:")
        for k in ["k_f", "k_m", "Ixx", "Iyy", "Izz", "tau"]:
            v = refined[k]
            t = truth.get(k, 0)
            e = (v - t) / t * 100 if t else float("nan")
            print(f"    {k:6s} = {v:12.6e}  (truth: {t:12.6e}, err: {e:+6.1f}%)")
        print(f"    cost = {refined['cost']:.6e}")

    return refined


def compute_excitation_level(sd, i0, i1):
    """Report angular excitation richness."""
    gyro = sd["gyro"][i0:i1]
    alpha = sd["gyro_dot"][i0:i1]
    p_std, q_std, r_std = np.std(gyro, axis=0)
    ap_std, aq_std, ar_std = np.std(alpha, axis=0)
    return {
        "omega_std_deg": [np.degrees(p_std), np.degrees(q_std), np.degrees(r_std)],
        "alpha_std_deg": [np.degrees(ap_std), np.degrees(aq_std), np.degrees(ar_std)],
    }


def run_identification(ulg_path, mass, arm_length, omega_max=1000.0,
                       include_drag=False, verbose=True):
    if verbose:
        print(f"Loading ULG: {ulg_path}")
    fd = extract_flight_data(ulg_path)

    if verbose:
        print("Synchronizing to 250 Hz...")
    sd = synchronize(fd)

    i0, i1 = detect_flight(sd)
    t = sd["t"]
    dur = (i1 - i0) * sd["dt"]
    if verbose:
        print(f"Flight segment: {t[i0]:.1f}s–{t[i1]:.1f}s ({dur:.1f}s, {i1-i0} pts)")

    exc = compute_excitation_level(sd, i0, i1)
    if verbose:
        print(f"Angular excitation (std):")
        print(f"  ω: roll={exc['omega_std_deg'][0]:.1f}°/s  "
              f"pitch={exc['omega_std_deg'][1]:.1f}°/s  "
              f"yaw={exc['omega_std_deg'][2]:.1f}°/s")
        print(f"  α: roll={exc['alpha_std_deg'][0]:.1f}°/s²  "
              f"pitch={exc['alpha_std_deg'][1]:.1f}°/s²  "
              f"yaw={exc['alpha_std_deg'][2]:.1f}°/s²")
        if max(exc['omega_std_deg'][:2]) < 5:
            print("  ⚠ Low excitation — inertia ID may be unreliable. "
                  "Use fly_sysid_maneuver.py for better results.")

    rotor_pos = ROTOR_POSITIONS * (arm_length / 0.175)

    if verbose:
        print("\n═══ Stage 1: Thrust Coefficient ═══")
    k_f, tau_est, _ = stage1_identify_kf(sd, i0, i1, mass, omega_max, verbose)

    if verbose:
        print("\n═══ Stage 2: Inertia (Linear Regression) ═══")
    inertia_result, _ = stage2_identify_inertia(
        sd, i0, i1, k_f, tau_est, omega_max, rotor_pos, ROTOR_DIRECTIONS, verbose)

    initial = {
        "k_f": k_f, "k_m": inertia_result["k_m"],
        "Ixx": inertia_result["Ixx"], "Iyy": inertia_result["Iyy"],
        "Izz": inertia_result["Izz"], "tau": tau_est,
    }

    if verbose:
        print("\n═══ Stage 3: Joint Nonlinear Refinement ═══")
    refined = stage3_refine(sd, i0, i1, initial, mass, omega_max,
                            rotor_pos, ROTOR_DIRECTIONS, include_drag, verbose)

    refined["mass"] = mass
    refined["omega_max"] = omega_max
    refined["arm_length"] = arm_length
    refined["excitation"] = exc
    refined["flight_duration_s"] = dur

    if verbose:
        print("\n" + "=" * 60)
        print("FINAL IDENTIFIED PARAMETERS")
        print("=" * 60)
        truth = {"k_f": 8.54858e-6, "k_m": 1.36777e-7,
                 "Ixx": 0.02167, "Iyy": 0.02167, "Izz": 0.04, "tau": 0.0125}
        print(f"{'Param':<8} {'Identified':>14} {'x500 Truth':>14} {'Err%':>8}")
        print("-" * 48)
        for k in ["k_f", "k_m", "Ixx", "Iyy", "Izz", "tau"]:
            v = refined[k]
            t = truth[k]
            e = (v - t) / t * 100
            print(f"{k:<8} {v:>14.6e} {t:>14.6e} {e:>+7.1f}%")
    return refined


def main():
    parser = argparse.ArgumentParser(description="Two-stage IMU system identification")
    parser.add_argument("ulg_file", help="PX4 ULG file path")
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--arm-length", type=float, default=0.175)
    parser.add_argument("--omega-max", type=float, default=1000.0)
    parser.add_argument("--drag", action="store_true")
    parser.add_argument("--output", type=str, default=None)
    args = parser.parse_args()

    if not Path(args.ulg_file).is_file():
        print(f"Error: {args.ulg_file} not found", file=sys.stderr)
        return 1

    result = run_identification(
        args.ulg_file, args.mass, args.arm_length,
        args.omega_max, args.drag, verbose=True)

    out = args.output or str(Path(args.ulg_file).with_suffix(".sysid.json"))
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nSaved: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
