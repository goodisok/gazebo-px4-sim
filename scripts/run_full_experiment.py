#!/usr/bin/env python3
"""
Full high-speed interceptor dynamics experiment automation.

Phases:
1. Start PX4 SITL with x500 (standard model, "truth")
2. Fly multi-speed mission and collect ULG
3. Start PX4 SITL with x500_hp (high-performance model)
4. Fly multi-speed mission and collect ULG
5. Run system identification on all ULGs
6. Generate comprehensive analysis

Usage:
    python3 run_full_experiment.py [--phase 1|2|3|all]
"""

import asyncio
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS_DIR = Path(__file__).parent
PROJECT_DIR = SCRIPTS_DIR.parent
PX4_ROOT = PROJECT_DIR / "PX4-Autopilot"
BUILD_DIR = PX4_ROOT / "build/px4_sitl_default"
ROOTFS = BUILD_DIR / "rootfs"
RESULTS_DIR = PROJECT_DIR / "results" / "highspeed"

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def cleanup():
    """Kill any running PX4/Gazebo processes."""
    log("Cleaning up processes...")
    for pattern in ["gz-sim-server", "ruby.*gz", "parameter_bridge"]:
        subprocess.run(["pkill", "-9", "-f", pattern],
                       capture_output=True, timeout=5)
    # Kill PX4 by binary path
    subprocess.run(["pkill", "-9", "-f", str(BUILD_DIR / "bin/px4")],
                   capture_output=True, timeout=5)
    time.sleep(3)
    log("Cleanup done")


def start_px4(model: str, logfile: str) -> subprocess.Popen:
    """Start PX4 SITL with specified model."""
    log(f"Starting PX4 with model={model}")
    cleanup()

    # Clean parameters for fresh start
    for f in ["parameters.bson", "parameters_backup.bson", "dataman"]:
        p = ROOTFS / f
        if p.exists():
            p.unlink()

    env = os.environ.copy()
    env["PX4_SIM_MODEL"] = f"gz_{model}"
    env["GZ_IP"] = "127.0.0.1"
    env["HEADLESS"] = "1"

    log_path = Path(logfile)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    with open(log_path, "w") as logf:
        proc = subprocess.Popen(
            [str(BUILD_DIR / "bin/px4")],
            cwd=str(ROOTFS),
            env=env,
            stdout=logf,
            stderr=subprocess.STDOUT,
        )

    log(f"PX4 PID: {proc.pid}")

    # Wait for PX4 to be ready
    for i in range(120):
        time.sleep(1)
        if proc.poll() is not None:
            log(f"PX4 exited with code {proc.returncode}")
            with open(log_path) as f:
                print(f.read()[-2000:])
            return None

        try:
            content = log_path.read_text()
            if "Startup script returned successfully" in content:
                log(f"PX4 ready after {i+1}s")
                time.sleep(5)
                return proc
        except Exception:
            pass

    log("PX4 startup timeout!")
    return None


def get_latest_ulg():
    """Find the most recent ULG file."""
    log_dirs = sorted(ROOTFS.glob("log/*/"), key=os.path.getmtime, reverse=True)
    for d in log_dirs:
        ulgs = sorted(d.glob("*.ulg"), key=os.path.getmtime, reverse=True)
        if ulgs:
            return ulgs[0]
    return None


async def run_multispeed_flight(speeds_str: str):
    """Run multi-speed flight test."""
    from mavsdk import System
    from mavsdk.offboard import VelocityBodyYawspeed

    speeds = [float(s) for s in speeds_str.split(",")]
    log(f"Flying multi-speed: {speeds} m/s")

    drone = System()
    await drone.connect(system_address="udp://:14540")

    log("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    log("Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    await drone.action.set_takeoff_altitude(20.0)
    log("Arming...")
    await drone.action.arm()
    log("Taking off to 20m...")
    await drone.action.takeoff()
    await asyncio.sleep(12)

    # Initialize offboard
    for _ in range(20):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(0.1)
    await drone.offboard.start()
    await asyncio.sleep(2)

    # Phase 1: Hover baseline
    log("Phase: Hover baseline (5s)")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(5)

    # Phase 2: Sysid doublets at hover
    log("Phase: Sysid doublets")
    for amp in [5.0, 8.0]:
        for _ in range(4):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, amp, 0, 0))
            await asyncio.sleep(0.3)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, -amp, 0, 0))
            await asyncio.sleep(0.3)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.0)

    # Phase 3: Multi-speed forward dashes
    for target_speed in speeds:
        log(f"Phase: Forward dash at {target_speed} m/s")

        # Climb higher for high-speed segments
        if target_speed >= 20:
            log(f"  Climbing to safe altitude...")
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, -5, 0))
            await asyncio.sleep(5)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, 0, 0))
            await asyncio.sleep(2)

        # Ramp up
        ramp_time = min(target_speed / 5.0, 10.0)
        steps = max(int(ramp_time / 0.2), 5)
        for step in range(steps):
            v = target_speed * (step + 1) / steps
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v, 0, 0, 0))
            await asyncio.sleep(0.2)

        # Cruise at target speed
        cruise_time = 4.0 if target_speed < 20 else 6.0
        log(f"  Cruising at {target_speed} m/s for {cruise_time}s")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(target_speed, 0, 0, 0))
        await asyncio.sleep(cruise_time)

        # Doublets while cruising (for angular dynamics at speed)
        if target_speed >= 10:
            log(f"  Sysid at speed")
            for _ in range(3):
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(target_speed, 3, 0, 0))
                await asyncio.sleep(0.25)
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(target_speed, -3, 0, 0))
                await asyncio.sleep(0.25)

        # Deceleration (natural drag measurement)
        log(f"  Deceleration from {target_speed} m/s")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(5)

    # Phase 4: Final hover
    log("Phase: Final hover (3s)")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(3)

    # Land
    log("Landing...")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(12)

    try:
        await drone.action.disarm()
    except Exception:
        pass

    log("Flight complete!")


def run_analysis(ulg_path: Path, name: str, omega_max: float, mass: float,
                 truth_kf: float):
    """Run comprehensive analysis on a ULG file."""
    log(f"Analyzing {name}: {ulg_path}")
    out_dir = RESULTS_DIR / f"analysis_{name}"
    out_dir.mkdir(parents=True, exist_ok=True)

    sys.path.insert(0, str(SCRIPTS_DIR))
    from analyze_sysid_results import (
        extract_data, analyze_speed_segments,
        identify_kf_multispeed, estimate_drag_coefficient,
        generate_plots
    )

    data = extract_data(ulg_path)

    speed_analysis = analyze_speed_segments(data)
    kf_results = identify_kf_multispeed(data, omega_max, mass)
    drag = estimate_drag_coefficient(data, truth_kf, omega_max, mass)
    generate_plots(data, str(out_dir), name)

    summary = {
        "ulg": str(ulg_path),
        "name": name,
        "omega_max": omega_max,
        "mass": mass,
        "truth_kf": truth_kf,
        "speed_analysis": speed_analysis,
        "kf_results": kf_results,
        "drag": drag,
    }

    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    # Print summary
    print(f"\n{'='*60}")
    print(f"  Analysis: {name}")
    print(f"{'='*60}")

    if "speed_segments" in speed_analysis:
        print(f"Max speed: {speed_analysis.get('max_speed', 0):.1f} m/s")
        for seg, v in speed_analysis["speed_segments"].items():
            print(f"  {seg:>10s}: v_mean={v['v_mean']:.1f}, "
                  f"motor={v['motor_mean']:.3f}±{v['motor_std']:.3f}")

    print(f"\nk_f identification (truth: {truth_kf:.6e}):")
    for seg, v in kf_results.items():
        kf = v["k_f"]
        err = (kf - truth_kf) / truth_kf * 100
        print(f"  {seg:>14s}: k_f={kf:.6e} (err: {err:+.1f}%), "
              f"n={v['n_samples']}")

    if "error" not in drag:
        print(f"\nDrag: Cd_quad={drag['Cd_quadratic']:.4f}, "
              f"Cd_lin={drag['Cd_linear']:.4f}, "
              f"max_v={drag['max_speed']:.1f} m/s")

    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", type=str, default="all",
                        help="Phase to run: 1(x500), 2(hp), 3(analyze), all")
    parser.add_argument("--speeds", type=str, default="5,10,15",
                        help="Speed levels for standard x500 test")
    parser.add_argument("--hp-speeds", type=str, default="5,10,15,25",
                        help="Speed levels for HP model test")
    args = parser.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    phases = args.phase.split(",") if "," in args.phase else [args.phase]
    run_all = "all" in phases

    # ═══════════════════════════════════════════
    # Phase 1: Standard x500 baseline
    # ═══════════════════════════════════════════
    if run_all or "1" in phases:
        log("═══ PHASE 1: Standard x500 baseline ═══")
        proc = start_px4("x500", "/tmp/px4_phase1.log")
        if proc is None:
            log("ERROR: PX4 x500 failed to start!")
            return 1

        try:
            asyncio.run(run_multispeed_flight(args.speeds))
        except Exception as e:
            log(f"Flight error: {e}")

        time.sleep(3)
        ulg = get_latest_ulg()
        if ulg:
            dest = RESULTS_DIR / "x500_multispeed.ulg"
            import shutil
            shutil.copy2(ulg, dest)
            log(f"ULG saved: {dest}")
        else:
            log("WARNING: No ULG found!")

        cleanup()
        time.sleep(5)

    # ═══════════════════════════════════════════
    # Phase 2: HP model
    # ═══════════════════════════════════════════
    if run_all or "2" in phases:
        log("═══ PHASE 2: High-Performance x500_hp ═══")
        proc = start_px4("x500_hp", "/tmp/px4_phase2.log")
        if proc is None:
            log("ERROR: PX4 x500_hp failed to start!")
            # Try with standard x500 base (model is x500_hp but airframe 4001)
            log("Retrying with x500 airframe base...")
            os.environ.pop("PX4_SYS_AUTOSTART", None)
            proc = start_px4("x500_hp", "/tmp/px4_phase2b.log")
            if proc is None:
                log("ERROR: x500_hp still failed!")
                return 1

        try:
            asyncio.run(run_multispeed_flight(args.hp_speeds))
        except Exception as e:
            log(f"Flight error: {e}")

        time.sleep(3)
        ulg = get_latest_ulg()
        if ulg:
            dest = RESULTS_DIR / "hp_multispeed.ulg"
            import shutil
            shutil.copy2(ulg, dest)
            log(f"ULG saved: {dest}")
        else:
            log("WARNING: No ULG found!")

        cleanup()
        time.sleep(5)

    # ═══════════════════════════════════════════
    # Phase 3: Analysis
    # ═══════════════════════════════════════════
    if run_all or "3" in phases:
        log("═══ PHASE 3: Comprehensive Analysis ═══")

        # x500 standard: motorConstant=8.54858e-6, omega_max=1000, mass=2.0
        x500_ulg = RESULTS_DIR / "x500_multispeed.ulg"
        if x500_ulg.exists():
            run_analysis(x500_ulg, "x500_standard",
                         omega_max=1000.0, mass=2.0,
                         truth_kf=8.54858e-6)

        # x500_hp: motorConstant=2.73e-5, omega_max=1200, mass=2.0
        hp_ulg = RESULTS_DIR / "hp_multispeed.ulg"
        if hp_ulg.exists():
            run_analysis(hp_ulg, "x500_hp",
                         omega_max=1200.0, mass=2.0,
                         truth_kf=2.73e-5)

    log("Experiment complete!")
    log(f"Results in: {RESULTS_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
