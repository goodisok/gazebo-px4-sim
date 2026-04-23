#!/usr/bin/env python3
"""
Multi-speed flight test for drag identification and high-speed validation.

Phases:
  1. Hover baseline (5s)
  2. Sysid doublets (roll/pitch/yaw)
  3. Speed levels: 5, 10, 15, 20, 30, 40, 50 m/s forward dashes
  4. Deceleration profiles (natural drag measurement)
  5. High-speed sysid doublets (at peak speed)
  6. Land

Usage:
    python3 fly_multispeed.py [--speeds 5,15,30,50]
"""

import asyncio
import argparse
import math
import time
import sys

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


async def run_multispeed(drone, speeds, verbose=True):
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

    await drone.action.set_takeoff_altitude(15.0)
    log("Arming...")
    await drone.action.arm()
    log("Taking off to 15m...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    for _ in range(20):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(0.1)
    await drone.offboard.start()
    await asyncio.sleep(2)

    # Phase 1: Hover baseline
    log("Phase 1: Hover baseline (5s)")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(5)

    # Phase 2: Low-speed sysid doublets
    log("Phase 2: Sysid doublets at hover")
    for amp in [5.0, 8.0]:
        for _ in range(4):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, amp, 0, 0))
            await asyncio.sleep(0.3)
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, -amp, 0, 0))
            await asyncio.sleep(0.3)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.0)

    for amp in [5.0, 8.0]:
        for _ in range(4):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(amp, 0, 0, 0))
            await asyncio.sleep(0.3)
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-amp, 0, 0, 0))
            await asyncio.sleep(0.3)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(1.0)

    # Phase 3: Multi-speed forward dashes with deceleration profiles
    for target_speed in speeds:
        log(f"Phase 3: Forward dash at {target_speed} m/s")

        # Climb to safe altitude before high-speed segments
        if target_speed >= 30:
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -5, 0))
            await asyncio.sleep(4)
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            await asyncio.sleep(1)

        # Ramp up
        ramp_time = min(target_speed / 5.0, 10.0)
        steps = max(int(ramp_time / 0.2), 5)
        for step in range(steps):
            v = target_speed * (step + 1) / steps
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(v, 0, 0, 0))
            await asyncio.sleep(0.2)

        # Cruise at target speed
        cruise_time = 4.0 if target_speed < 30 else 6.0
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(target_speed, 0, 0, 0))
        await asyncio.sleep(cruise_time)

        # High-speed doublets while cruising
        if target_speed >= 15:
            log(f"  Sysid at {target_speed} m/s")
            for _ in range(3):
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(target_speed, 3, 0, 0))
                await asyncio.sleep(0.25)
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(target_speed, -3, 0, 0))
                await asyncio.sleep(0.25)

        # Deceleration (natural drag measurement)
        log(f"  Deceleration from {target_speed} m/s")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(5)

    # Phase 4: Final hover
    log("Phase 4: Final hover (3s)")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(3)

    # Land
    log("Landing...")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(10)
    log("Done.")


async def main_async(args):
    drone = System()
    speeds = [float(s) for s in args.speeds.split(",")]
    await run_multispeed(drone, speeds)


def main():
    parser = argparse.ArgumentParser(description="Multi-speed flight test")
    parser.add_argument("--speeds", type=str, default="5,10,15",
                        help="Comma-separated speed levels in m/s")
    args = parser.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
