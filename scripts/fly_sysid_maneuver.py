#!/usr/bin/env python3
"""
System-Identification Flight Maneuver Script.

Generates rich angular excitation for parameter identification:
  1. Doublets in roll, pitch, yaw
  2. Frequency sweeps
  3. Multi-axis combined maneuvers
  4. High-speed forward dashes (optional)

Usage:
    python3 fly_sysid_maneuver.py [--max-speed 15] [--high-speed 50]
"""

import asyncio
import argparse
import math
import time
import sys

from mavsdk import System
from mavsdk.offboard import (
    OffboardError, VelocityNedYaw, VelocityBodyYawspeed,
    PositionNedYaw, Attitude,
)


async def wait_armed_airborne(drone, alt=-3.0):
    """Wait until the drone reaches target altitude."""
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > abs(alt) * 0.6:
            return


async def run_sysid(drone, max_speed=15.0, high_speed=0.0, verbose=True):
    """Execute the sysid maneuver sequence."""
    def log(msg):
        if verbose:
            print(f"[{time.strftime('%H:%M:%S')}] {msg}")

    log("Connecting...")
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    log("Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    await drone.action.set_takeoff_altitude(10.0)
    log("Arming...")
    await drone.action.arm()
    log("Taking off to 10m...")
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # Set initial setpoint before starting offboard
    for _ in range(10):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(0.1)
    await drone.offboard.start()
    await asyncio.sleep(2)

    log("═══ Phase 1: Hover baseline (5s) ═══")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(5)

    # ── Roll doublets ──
    log("═══ Phase 2: Roll doublets ═══")
    for amp in [3.0, 6.0, 10.0]:
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, amp, 0, 0))
            await asyncio.sleep(0.4)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, -amp, 0, 0))
            await asyncio.sleep(0.4)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.5)

    # ── Pitch doublets ──
    log("═══ Phase 3: Pitch doublets ═══")
    for amp in [3.0, 6.0, 10.0]:
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(amp, 0, 0, 0))
            await asyncio.sleep(0.4)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(-amp, 0, 0, 0))
            await asyncio.sleep(0.4)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.5)

    # ── Yaw doublets ──
    log("═══ Phase 4: Yaw doublets ═══")
    for yaw_rate in [30, 60, 90]:
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, 0, yaw_rate))
            await asyncio.sleep(0.5)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, 0, -yaw_rate))
            await asyncio.sleep(0.5)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.5)

    # ── Vertical doublets ──
    log("═══ Phase 5: Vertical (thrust) doublets ═══")
    for vz in [2.0, 4.0, 6.0]:
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, vz, 0))
            await asyncio.sleep(0.5)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0, 0, -vz, 0))
            await asyncio.sleep(0.5)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.5)

    # ── Multi-axis combined ──
    log("═══ Phase 6: Multi-axis combined ═══")
    dt = 0.05
    duration = 8.0
    t = 0
    while t < duration:
        vx = max_speed * 0.5 * math.sin(2 * math.pi * 0.3 * t)
        vy = max_speed * 0.3 * math.sin(2 * math.pi * 0.5 * t)
        vz = 2.0 * math.sin(2 * math.pi * 0.2 * t)
        yr = 40 * math.sin(2 * math.pi * 0.4 * t)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yr))
        await asyncio.sleep(dt)
        t += dt
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(2)

    # ── Forward speed ramp ──
    log(f"═══ Phase 7: Speed ramp to {max_speed} m/s ═══")
    for v in range(0, int(max_speed) + 1, 2):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(float(v), 0, 0, 0))
        await asyncio.sleep(1.0)
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(max_speed, 0, 0, 0))
    await asyncio.sleep(3)
    # Decelerate
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(3)

    # ── High-speed phase (optional, e.g. 50 m/s) ──
    if high_speed > max_speed:
        log(f"═══ Phase 8: High-speed dash to {high_speed} m/s ═══")
        # Climb first for safety margin
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, -5, 0))
        await asyncio.sleep(4)

        for v in range(int(max_speed), int(high_speed) + 1, 5):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(float(v), 0, 0, 0))
            await asyncio.sleep(1.5)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(high_speed, 0, 0, 0))
        await asyncio.sleep(4)

        # High-speed doublets
        log("  High-speed roll doublets...")
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(high_speed, 5, 0, 0))
            await asyncio.sleep(0.3)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(high_speed, -5, 0, 0))
            await asyncio.sleep(0.3)

        log("  High-speed pitch doublets...")
        for _ in range(3):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(high_speed + 5, 0, 0, 0))
            await asyncio.sleep(0.3)
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(high_speed - 5, 0, 0, 0))
            await asyncio.sleep(0.3)

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(5)

    # ── Frequency sweep ──
    log("═══ Phase 9: Frequency sweep (0.5–5 Hz lateral) ═══")
    dt = 0.02
    duration = 10.0
    t = 0
    while t < duration:
        freq = 0.5 + 4.5 * (t / duration)
        amp = 5.0
        vy = amp * math.sin(2 * math.pi * freq * t)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, vy, 0, 0))
        await asyncio.sleep(dt)
        t += dt
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(2)

    # ── Land ──
    log("═══ Landing ═══")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(8)
    await drone.action.disarm()
    log("Done.")


async def main_async(args):
    drone = System()
    await run_sysid(drone, max_speed=args.max_speed, high_speed=args.high_speed)


def main():
    parser = argparse.ArgumentParser(description="SysID maneuver flight")
    parser.add_argument("--max-speed", type=float, default=15.0,
                        help="Max forward speed in m/s (default: 15)")
    parser.add_argument("--high-speed", type=float, default=0.0,
                        help="High-speed dash phase (0=skip)")
    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
