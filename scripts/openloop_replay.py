#!/usr/bin/env python3
"""
Open-loop actuator replay for pure dynamics validation.

Extracts motor commands from a PX4 ULG log (the "truth" flight),
replays them directly into a Gazebo model (bypassing PX4 controller),
and compares the resulting trajectory with the truth.

Usage:
    python3 openloop_replay.py \
        --ulg data/flight_logs/x500_truth.ulg \
        --model-name interceptor \
        --output-dir results/openloop_round4
"""

import os, sys, time, signal, subprocess, argparse, json, threading
import numpy as np
import pandas as pd
from pathlib import Path

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from pyulog import ULog


# ---------------------------------------------------------------------------
# 1. Extract truth data from ULG
# ---------------------------------------------------------------------------

def extract_from_ulg(ulg_path):
    """Extract actuator commands + truth states from ULG."""
    ulog = ULog(ulg_path)
    data = {}
    for d in ulog.data_list:
        data[d.name] = d.data

    if "actuator_outputs" not in data:
        raise ValueError("No actuator_outputs in ULG")

    act = data["actuator_outputs"]
    ts_us = act["timestamp"]
    t0 = ts_us[0]
    ts_s = (ts_us - t0) / 1e6

    cmds = np.column_stack([act[f"output[{i}]"] for i in range(4)])

    result = {"cmd_ts": ts_s, "cmds": cmds}

    if "vehicle_local_position" in data:
        pos = data["vehicle_local_position"]
        pos_ts = (pos["timestamp"] - t0) / 1e6
        result["truth_pos"] = pd.DataFrame({
            "t": pos_ts, "x": pos["x"], "y": pos["y"], "z": pos["z"],
            "vx": pos["vx"], "vy": pos["vy"], "vz": pos["vz"],
        })

    if "vehicle_attitude" in data:
        att = data["vehicle_attitude"]
        att_ts = (att["timestamp"] - t0) / 1e6
        q0 = att["q[0]"]
        q1 = att["q[1]"]
        q2 = att["q[2]"]
        q3 = att["q[3]"]
        roll = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
        yaw = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))
        result["truth_att"] = pd.DataFrame({
            "t": att_ts, "roll": roll, "pitch": pitch, "yaw": yaw,
        })

    return result


# ---------------------------------------------------------------------------
# 2. Gazebo management
# ---------------------------------------------------------------------------

GZ_PROC = None

def start_gazebo(world_path, model_resource_path):
    """Start Gazebo server headless."""
    global GZ_PROC
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = model_resource_path
    cmd = ["gz", "sim", "-s", "--headless-rendering", str(world_path)]
    print(f"Starting Gazebo: {' '.join(cmd)}")
    print(f"  GZ_SIM_RESOURCE_PATH={model_resource_path}")
    GZ_PROC = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
    time.sleep(8)
    if GZ_PROC.poll() is not None:
        stderr = GZ_PROC.stderr.read().decode()
        raise RuntimeError(f"Gazebo failed to start:\n{stderr}")
    print("Gazebo server running.")


def stop_gazebo():
    global GZ_PROC
    if GZ_PROC and GZ_PROC.poll() is None:
        GZ_PROC.terminate()
        try:
            GZ_PROC.wait(timeout=5)
        except subprocess.TimeoutExpired:
            GZ_PROC.kill()
    subprocess.run(["pkill", "-f", "gz-sim-server"], capture_output=True)


# ---------------------------------------------------------------------------
# 3. Gazebo transport: publish commands + subscribe to pose
# ---------------------------------------------------------------------------

def run_replay(cmd_ts, cmds, model_name, duration_margin=2.0):
    """
    Publish motor commands to Gazebo and record model pose.

    Uses gz-transport Python bindings for publishing and subscribing.
    Gazebo runs at max speed (real_time_factor=0), so we use wall-clock
    timing relative to simulation start.
    """
    from gz.transport13 import Node
    from gz.msgs10 import actuators_pb2, pose_v_pb2

    node = Node()

    cmd_topic = f"/{model_name}/command/motor_speed"
    pose_topic = "/world/openloop/dynamic_pose/info"

    pub = node.advertise(cmd_topic, actuators_pb2.Actuators)
    if not pub:
        raise RuntimeError(f"Failed to advertise on {cmd_topic}")
    print(f"Publishing motor commands on: {cmd_topic}")

    poses_log = []
    pose_lock = threading.Lock()

    def pose_callback(msg):
        """Collect all pose updates from SceneBroadcaster."""
        wall_t = time.monotonic()
        try:
            pose_v = pose_v_pb2.Pose_V()
            pose_v.ParseFromString(msg)
        except Exception:
            return

        for pose in pose_v.pose:
            if model_name in pose.name:
                p = pose.position
                o = pose.orientation
                with pose_lock:
                    poses_log.append({
                        "wall_t": wall_t,
                        "x": p.x, "y": p.y, "z": p.z,
                        "qw": o.w, "qx": o.x, "qy": o.y, "qz": o.z,
                    })
                break

    subscribed = node.subscribe(pose_topic, pose_callback, pose_v_pb2.Pose_V)
    if not subscribed:
        print(f"WARNING: Could not subscribe to {pose_topic}")
        print("  Falling back to polling via gz service...")

    flight_end = cmd_ts[-1] + duration_margin
    num_cmds = len(cmd_ts)
    cmd_idx = 0

    print(f"Replaying {num_cmds} commands over {flight_end:.1f}s of sim time...")

    t_start = time.monotonic()

    while True:
        elapsed = time.monotonic() - t_start

        while cmd_idx < num_cmds and cmd_ts[cmd_idx] <= elapsed:
            msg = actuators_pb2.Actuators()
            for j in range(4):
                msg.velocity.append(float(cmds[cmd_idx, j]))
            pub.publish(msg)
            cmd_idx += 1

        if elapsed > flight_end:
            break

        time.sleep(0.001)

    print(f"Replay complete. Published {cmd_idx} commands, recorded {len(poses_log)} poses.")
    return poses_log


def run_replay_cli(cmd_ts, cmds, model_name, duration_margin=2.0):
    """
    Fallback: use gz topic CLI to publish commands.
    Slower but more robust if Python transport has issues.
    """
    poses_log = []
    cmd_topic = f"/{model_name}/command/motor_speed"
    num_cmds = len(cmd_ts)

    print(f"Replaying {num_cmds} commands via CLI on {cmd_topic}...")

    t_start = time.monotonic()
    cmd_idx = 0
    flight_end = cmd_ts[-1] + duration_margin

    while True:
        elapsed = time.monotonic() - t_start

        if cmd_idx < num_cmds and cmd_ts[cmd_idx] <= elapsed:
            v = cmds[cmd_idx]
            payload = f"velocity: [{v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}, {v[3]:.1f}]"
            subprocess.run(
                ["gz", "topic", "-t", cmd_topic, "-m", "gz.msgs.Actuators",
                 "-p", payload],
                capture_output=True, timeout=2
            )
            cmd_idx += 1

            if cmd_idx % 50 == 0:
                print(f"  [{elapsed:.1f}s] Published {cmd_idx}/{num_cmds} commands")

        if elapsed > flight_end:
            break

        time.sleep(0.005)

    print(f"Replay complete. Published {cmd_idx} commands.")
    return poses_log


# ---------------------------------------------------------------------------
# 4. State extraction from Gazebo (post-replay via gz topic echo)
# ---------------------------------------------------------------------------

def record_poses_cli(model_name, duration, output_csv):
    """
    Record model pose by echoing the dynamic_pose topic for a duration.
    Run this in a separate thread during replay.
    """
    topic = "/world/openloop/dynamic_pose/info"
    cmd = ["gz", "topic", "-e", "-t", topic, "-d", str(duration)]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=duration + 10)
    with open(output_csv + ".raw", "w") as f:
        f.write(result.stdout)
    return result.stdout


# ---------------------------------------------------------------------------
# 5. Alternative: step-based replay for precise synchronization
# ---------------------------------------------------------------------------

def run_stepped_replay(cmd_ts, cmds, model_name, step_size=0.004):
    """
    Precise replay: pause Gazebo, step simulation manually between commands.
    Each command interval is divided into exact physics steps.
    """
    from gz.transport13 import Node
    from gz.msgs10 import actuators_pb2, boolean_pb2, world_control_pb2

    node = Node()

    cmd_topic = f"/model/{model_name}/command/motor_speed"
    control_service = "/world/openloop/control"

    pub = node.advertise(cmd_topic, actuators_pb2.Actuators)
    if not pub:
        raise RuntimeError(f"Failed to advertise on {cmd_topic}")

    pause_req = world_control_pb2.WorldControl()
    pause_req.pause = True
    ok, resp = node.request(control_service, pause_req,
                            "gz.msgs.WorldControl", "gz.msgs.Boolean", 3000)
    if not ok:
        print("WARNING: Could not pause simulation via service, trying CLI...")
        subprocess.run(["gz", "service", "-s", control_service,
                        "--reqtype", "gz.msgs.WorldControl",
                        "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000",
                        "--req", "pause: true"], capture_output=True)

    print(f"Stepped replay: {len(cmd_ts)} commands, step_size={step_size}s")

    sim_time = 0.0
    recorded = []

    for i in range(len(cmd_ts)):
        msg = actuators_pb2.Actuators()
        for j in range(4):
            msg.velocity.append(float(cmds[i, j]))
        pub.publish(msg)

        if i < len(cmd_ts) - 1:
            dt = cmd_ts[i + 1] - cmd_ts[i]
        else:
            dt = 0.1

        n_steps = max(1, int(round(dt / step_size)))

        step_req = world_control_pb2.WorldControl()
        step_req.multi_step = n_steps

        ok, resp = node.request(control_service, step_req,
                                "gz.msgs.WorldControl", "gz.msgs.Boolean", 5000)
        if not ok:
            subprocess.run(
                ["gz", "service", "-s", control_service,
                 "--reqtype", "gz.msgs.WorldControl",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", f"multi_step: {n_steps}"],
                capture_output=True
            )

        sim_time += n_steps * step_size

        if i % 50 == 0:
            print(f"  Step {i}/{len(cmd_ts)}, sim_time={sim_time:.2f}s")

    print(f"Stepped replay complete. sim_time={sim_time:.2f}s")
    return sim_time


# ---------------------------------------------------------------------------
# 6. Pose extraction via gz service (after replay)
# ---------------------------------------------------------------------------

def get_model_pose_cli(model_name):
    """Get current model pose via gz model CLI."""
    result = subprocess.run(
        ["gz", "model", "-m", model_name, "-p"],
        capture_output=True, text=True, timeout=5
    )
    return result.stdout.strip()


# ---------------------------------------------------------------------------
# 7. Full pipeline: record states during stepped replay
# ---------------------------------------------------------------------------

def run_full_stepped_replay(cmd_ts, cmds, model_name, step_size=0.004):
    """
    Stepped replay using Python gz-transport bindings.
    For each command: publish motor speed → step simulation → record pose.
    Much faster than CLI subprocess approach.
    """
    from gz.transport13 import Node
    from gz.msgs10 import (actuators_pb2, world_control_pb2,
                            boolean_pb2, pose_v_pb2)

    node = Node()
    cmd_topic = f"/{model_name}/command/motor_speed"
    control_service = "/world/openloop/control"

    pub = node.advertise(cmd_topic, actuators_pb2.Actuators)
    if not pub:
        raise RuntimeError(f"Failed to advertise on {cmd_topic}")

    # Subscribe to dynamic pose for state recording
    latest_pose = {"x": 0, "y": 0, "z": 0.24, "qw": 1, "qx": 0, "qy": 0, "qz": 0}
    pose_lock = threading.Lock()

    def on_pose(pose_v):
        try:
            for pose in pose_v.pose:
                if model_name in pose.name:
                    with pose_lock:
                        latest_pose["x"] = pose.position.x
                        latest_pose["y"] = pose.position.y
                        latest_pose["z"] = pose.position.z
                        latest_pose["qw"] = pose.orientation.w
                        latest_pose["qx"] = pose.orientation.x
                        latest_pose["qy"] = pose.orientation.y
                        latest_pose["qz"] = pose.orientation.z
                    break
        except Exception:
            pass

    pose_topic = "/world/openloop/dynamic_pose/info"
    node.subscribe(pose_v_pb2.Pose_V, pose_topic, on_pose)

    # Pause simulation before starting replay
    pause_req = world_control_pb2.WorldControl()
    pause_req.pause = True
    ok, _ = node.request(control_service, pause_req,
                         world_control_pb2.WorldControl,
                         boolean_pb2.Boolean, 5000)
    if ok:
        print("  Simulation paused via Python transport.")
    else:
        subprocess.run(
            ["gz", "service", "-s", control_service,
             "--reqtype", "gz.msgs.WorldControl",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", "pause: true"],
            capture_output=True, timeout=5
        )
        print("  Simulation paused via CLI fallback.")
    time.sleep(0.5)

    n_cmds = len(cmd_ts)
    print(f"  Replaying {n_cmds} commands on topic: {cmd_topic}")
    print(f"  Step size: {step_size}s, pose subscription: {pose_topic}")

    sim_time = 0.0
    states = []

    for i in range(n_cmds):
        # 1. Publish motor command
        msg = actuators_pb2.Actuators()
        for j in range(4):
            msg.velocity.append(float(cmds[i, j]))
        pub.publish(msg)

        # 2. Step simulation
        if i < n_cmds - 1:
            dt = cmd_ts[i + 1] - cmd_ts[i]
        else:
            dt = 0.1
        n_steps = max(1, int(round(dt / step_size)))

        step_req = world_control_pb2.WorldControl()
        step_req.multi_step = n_steps
        ok, _ = node.request(control_service, step_req,
                             world_control_pb2.WorldControl,
                             boolean_pb2.Boolean, 5000)
        if not ok:
            # Fallback to CLI
            subprocess.run(
                ["gz", "service", "-s", control_service,
                 "--reqtype", "gz.msgs.WorldControl",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", f"multi_step: {n_steps}"],
                capture_output=True, timeout=10
            )

        sim_time += n_steps * step_size

        # 3. Delay to let pose subscription update after stepping
        time.sleep(0.02)

        # 4. Record state
        with pose_lock:
            qw, qx, qy, qz = (latest_pose["qw"], latest_pose["qx"],
                                latest_pose["qy"], latest_pose["qz"])

        roll = np.degrees(np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1)))
        yaw = np.degrees(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))

        with pose_lock:
            states.append({
                "t": sim_time,
                "x": latest_pose["x"],
                "y": latest_pose["y"],
                "z": latest_pose["z"],
                "roll": roll, "pitch": pitch, "yaw": yaw,
            })

        if i % 50 == 0:
            v = cmds[i]
            print(f"  [{i}/{n_cmds}] sim_t={sim_time:.2f}s  "
                  f"z={latest_pose['z']:.3f}  "
                  f"motors=[{v[0]:.0f},{v[1]:.0f},{v[2]:.0f},{v[3]:.0f}]")

    print(f"Replay done. Recorded {len(states)} states over {sim_time:.2f}s")
    return pd.DataFrame(states)


def parse_gz_pose(pose_str, sim_time):
    """
    Parse 'gz model -m <name> -p' output. Example format:
      Requesting state for world [openloop]...
      Model: [7]
        - Name: interceptor
        - Pose [ XYZ (m) ] [ RPY (rad) ]:
          [0.000000 0.000000 0.240000]
          [0.000000 -0.000000 0.000000]
    """
    import re
    brackets = re.findall(r'\[([^\[\]]+)\]', pose_str)
    xyz_vals = None
    rpy_vals = None

    for b in brackets:
        parts = b.strip().split()
        if len(parts) == 3:
            try:
                vals = [float(p) for p in parts]
                if xyz_vals is None:
                    xyz_vals = vals
                elif rpy_vals is None:
                    rpy_vals = vals
            except ValueError:
                continue

    if xyz_vals and rpy_vals:
        return {
            "t": sim_time,
            "x": xyz_vals[0], "y": xyz_vals[1], "z": xyz_vals[2],
            "roll": np.degrees(rpy_vals[0]),
            "pitch": np.degrees(rpy_vals[1]),
            "yaw": np.degrees(rpy_vals[2]),
        }
    return None


# ---------------------------------------------------------------------------
# 8. Comparison and plotting
# ---------------------------------------------------------------------------

def compute_metrics(truth_df, sim_df, cols, truth_prefix="", sim_prefix=""):
    """Compute RMSE and R² between truth and simulation on common time axis."""
    t_start = max(truth_df["t"].iloc[0], sim_df["t"].iloc[0])
    t_end = min(truth_df["t"].iloc[-1], sim_df["t"].iloc[-1])

    if t_end <= t_start:
        print(f"WARNING: No overlapping time range!")
        return {}

    t_common = np.linspace(t_start, t_end, 2000)
    metrics = {}

    for col in cols:
        tcol = f"{truth_prefix}{col}" if truth_prefix else col
        scol = f"{sim_prefix}{col}" if sim_prefix else col

        if tcol not in truth_df.columns or scol not in sim_df.columns:
            continue

        truth_interp = np.interp(t_common, truth_df["t"], truth_df[tcol])
        sim_interp = np.interp(t_common, sim_df["t"], sim_df[scol])

        err = truth_interp - sim_interp
        rmse = np.sqrt(np.mean(err ** 2))
        ss_res = np.sum(err ** 2)
        ss_tot = np.sum((truth_interp - np.mean(truth_interp)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

        metrics[col] = {"rmse": rmse, "r2": r2}

    return metrics, t_common


def plot_comparison(truth_df, sim_df, cols, output_path, title,
                    truth_prefix="", sim_prefix="",
                    label_truth="x500 Truth", label_sim="Interceptor (open-loop)"):
    """Generate comparison plots."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_start = max(truth_df["t"].iloc[0], sim_df["t"].iloc[0])
    t_end = min(truth_df["t"].iloc[-1], sim_df["t"].iloc[-1])
    t_common = np.linspace(t_start, t_end, 2000)

    n = len(cols)
    fig, axes = plt.subplots(n, 1, figsize=(12, 3 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, col in zip(axes, cols):
        tcol = f"{truth_prefix}{col}" if truth_prefix else col
        scol = f"{sim_prefix}{col}" if sim_prefix else col

        truth_interp = np.interp(t_common, truth_df["t"], truth_df[tcol])
        sim_interp = np.interp(t_common, sim_df["t"], sim_df[scol])

        ax.plot(t_common, truth_interp, "b-", label=label_truth, linewidth=1)
        ax.plot(t_common, sim_interp, "r--", label=label_sim, linewidth=1)

        err = truth_interp - sim_interp
        rmse = np.sqrt(np.mean(err ** 2))
        ax.set_ylabel(col)
        ax.legend(loc="upper right")
        ax.text(0.02, 0.95, f"RMSE={rmse:.4f}", transform=ax.transAxes,
                fontsize=10, va="top", bbox=dict(boxstyle="round", fc="yellow", alpha=0.8))

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {output_path}")


# ---------------------------------------------------------------------------
# 9. Velocity computation from pose
# ---------------------------------------------------------------------------

def compute_velocity(states_df):
    """Compute velocity by numerical differentiation of position."""
    dt = np.diff(states_df["t"].values)
    dt[dt == 0] = 1e-6

    vx = np.gradient(states_df["x"].values, states_df["t"].values)
    vy = np.gradient(states_df["y"].values, states_df["t"].values)
    vz = np.gradient(states_df["z"].values, states_df["t"].values)

    states_df = states_df.copy()
    states_df["vx"] = vx
    states_df["vy"] = vy
    states_df["vz"] = vz
    return states_df


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Open-loop actuator replay")
    parser.add_argument("--ulg", required=True, help="Path to x500 truth ULG file")
    parser.add_argument("--model-name", default="interceptor")
    parser.add_argument("--model-path", default=None,
                        help="Path to directory containing model directories")
    parser.add_argument("--world", default=None, help="Path to world SDF")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--skip-replay", action="store_true",
                        help="Skip replay, use existing state CSV")
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent.parent
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.model_path is None:
        args.model_path = str(base_dir / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models")
    if args.world is None:
        args.world = str(base_dir / "worlds" / "openloop.sdf")

    print("=" * 60)
    print("Open-Loop Actuator Replay")
    print("=" * 60)

    # Step 1: Extract truth data
    print("\n[1/4] Extracting truth data from ULG...")
    truth = extract_from_ulg(args.ulg)
    cmd_ts = truth["cmd_ts"]
    cmds = truth["cmds"]
    print(f"  Commands: {len(cmd_ts)} samples, duration={cmd_ts[-1]:.1f}s")

    flying_mask = cmds.max(axis=1) > 200
    if flying_mask.any():
        takeoff_idx = np.argmax(flying_mask)
        takeoff_t = cmd_ts[takeoff_idx]
        print(f"  Motors active from t={takeoff_t:.2f}s (sample {takeoff_idx})")
    else:
        takeoff_t = 0
        takeoff_idx = 0

    # Skip the initial zeros (drone on ground, motors off).
    # Start from first non-zero motor command (arm point).
    first_nonzero = np.argmax(cmds.max(axis=1) > 0)
    print(f"  First non-zero command at t={cmd_ts[first_nonzero]:.2f}s (sample {first_nonzero})")
    replay_ts = cmd_ts[first_nonzero:] - cmd_ts[first_nonzero]
    replay_cmds = cmds[first_nonzero:]
    replay_truth_offset = cmd_ts[first_nonzero]

    states_csv = output_dir / "replay_states.csv"

    if not args.skip_replay:
        # Step 2: Start Gazebo
        print("\n[2/4] Starting Gazebo (headless, paused)...")
        try:
            start_gazebo(args.world, args.model_path)

            time.sleep(3)

            # Step 3: Run stepped replay with full command sequence
            print("\n[3/4] Running stepped replay (full sequence from arm)...")
            states_df = run_full_stepped_replay(
                replay_ts,
                replay_cmds,
                args.model_name,
            )

            if len(states_df) > 0:
                states_df.to_csv(states_csv, index=False)
                print(f"  States saved to {states_csv}")
            else:
                print("  ERROR: No states recorded!")
                return 1

        finally:
            stop_gazebo()
    else:
        print("\n[2-3/4] Skipping replay, loading existing states...")
        states_df = pd.read_csv(states_csv)

    # Step 4: Compare
    print("\n[4/4] Comparing with truth...")

    states_df = compute_velocity(states_df)

    truth_pos = truth.get("truth_pos")
    truth_att = truth.get("truth_att")

    if truth_pos is not None:
        tp = truth_pos.copy()
        tp["t"] = tp["t"] - replay_truth_offset

        metrics_pos, _ = compute_metrics(tp, states_df, ["x", "y", "z"])
        metrics_vel, _ = compute_metrics(tp, states_df, ["vx", "vy", "vz"])

        plot_comparison(tp, states_df, ["x", "y", "z"],
                        output_dir / "openloop_position.png",
                        "Open-Loop Replay: Position Comparison")
        plot_comparison(tp, states_df, ["vx", "vy", "vz"],
                        output_dir / "openloop_velocity.png",
                        "Open-Loop Replay: Velocity Comparison")

        print("\n  Position metrics:")
        for k, v in metrics_pos.items():
            print(f"    {k}: RMSE={v['rmse']:.4f}, R²={v['r2']:.4f}")
        print("  Velocity metrics:")
        for k, v in metrics_vel.items():
            print(f"    {k}: RMSE={v['rmse']:.4f}, R²={v['r2']:.4f}")

    if truth_att is not None:
        ta = truth_att.copy()
        ta["t"] = ta["t"] - replay_truth_offset

        metrics_att, _ = compute_metrics(ta, states_df, ["roll", "pitch", "yaw"])

        plot_comparison(ta, states_df, ["roll", "pitch", "yaw"],
                        output_dir / "openloop_attitude.png",
                        "Open-Loop Replay: Attitude Comparison")

        print("  Attitude metrics:")
        for k, v in metrics_att.items():
            print(f"    {k}: RMSE={v['rmse']:.4f}, R²={v['r2']:.4f}")

    all_metrics = {}
    for name, m in [("position", metrics_pos if truth_pos is not None else {}),
                    ("velocity", metrics_vel if truth_pos is not None else {}),
                    ("attitude", metrics_att if truth_att is not None else {})]:
        for k, v in m.items():
            all_metrics[f"{name}_{k}"] = v

    with open(output_dir / "openloop_metrics.json", "w") as f:
        json.dump(all_metrics, f, indent=2)

    print(f"\nResults saved to {output_dir}/")
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s, f: (stop_gazebo(), sys.exit(1)))
    sys.exit(main())
