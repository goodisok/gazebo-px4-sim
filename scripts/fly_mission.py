#!/usr/bin/env python3
"""
Automated flight mission for Sim-to-Real dynamics validation.
Executes a standardized maneuver sequence and records PX4 ULG log.

Usage:
    1. Start PX4 SITL: HEADLESS=1 make px4_sitl gz_x500
    2. Run this script: python3 fly_mission.py
    3. ULG log is saved by PX4 automatically in build/px4_sitl_default/rootfs/log/
"""

import asyncio
import sys

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityNedYaw,
)


async def wait_for_ready(drone):
    print("=== Waiting for GPS fix and home position ===")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS fix acquired, home position set.")
            break


async def get_position(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position


async def get_yaw(drone):
    async for att in drone.telemetry.attitude_euler():
        return att.yaw_deg


async def wait_altitude(drone, target_down, tol=0.3, timeout=15):
    """Wait until NED down reaches target (e.g., -5 for 5m up)."""
    for _ in range(int(timeout * 10)):
        pos = await get_position(drone)
        if abs(pos.down_m - target_down) < tol:
            return True
        await asyncio.sleep(0.1)
    return False


async def run():
    drone = System()
    print("Connecting to PX4 SITL on udp://:14540 ...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for connection ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    await wait_for_ready(drone)

    await drone.action.set_takeoff_altitude(5.0)

    print("Arming ...")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Arm failed: {e}")
        sys.exit(1)

    # Phase 1: Use action.takeoff first to reliably get airborne
    print("=== Phase 1: Takeoff to 5m (action mode) ===")
    await drone.action.takeoff()
    if not await wait_altitude(drone, -5.0, tol=0.5, timeout=15):
        print("WARNING: takeoff altitude not reached, continuing anyway")
    await asyncio.sleep(2)

    # Get current pose for offboard reference
    pos = await get_position(drone)
    home_yaw = await get_yaw(drone)
    hover_n, hover_e = pos.north_m, pos.east_m

    # Transition to offboard: send setpoints first, then start
    await drone.offboard.set_position_ned(
        PositionNedYaw(hover_n, hover_e, -5.0, home_yaw)
    )
    await asyncio.sleep(0.5)
    await drone.offboard.set_position_ned(
        PositionNedYaw(hover_n, hover_e, -5.0, home_yaw)
    )

    print("=== Phase 2: Switch to Offboard mode ===")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        await drone.action.land()
        sys.exit(1)

    # Phase 3: Hover 10s
    print("=== Phase 3: Hover 10s (thrust balance check) ===")
    await drone.offboard.set_position_ned(
        PositionNedYaw(hover_n, hover_e, -5.0, home_yaw)
    )
    await asyncio.sleep(10)

    # Phase 4: Z step up 5 → 7m
    print("=== Phase 4: Z step UP  5m → 7m (thrust model) ===")
    await drone.offboard.set_position_ned(
        PositionNedYaw(hover_n, hover_e, -7.0, home_yaw)
    )
    await asyncio.sleep(5)

    # Phase 5: Z step down 7 → 5m
    print("=== Phase 5: Z step DOWN  7m → 5m ===")
    await drone.offboard.set_position_ned(
        PositionNedYaw(hover_n, hover_e, -5.0, home_yaw)
    )
    await asyncio.sleep(5)

    # Phase 6: Forward flight 3 m/s
    print("=== Phase 6: Forward flight 3m/s (drag model) ===")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(3.0, 0.0, 0.0, home_yaw)
    )
    await asyncio.sleep(5)

    # Phase 7: Decelerate to hover
    print("=== Phase 7: Decelerate to hover ===")
    pos = await get_position(drone)
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, -5.0, home_yaw)
    )
    await asyncio.sleep(3)

    # Phase 8: Lateral velocity step → roll excitation
    print("=== Phase 8: Roll excitation (lateral velocity step, Ixx) ===")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 2.0, 0.0, home_yaw)
    )
    await asyncio.sleep(3)
    print("  Back to hover")
    pos = await get_position(drone)
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, -5.0, home_yaw)
    )
    await asyncio.sleep(3)

    # Phase 9: Forward velocity step → pitch excitation
    print("=== Phase 9: Pitch excitation (forward velocity step, Iyy) ===")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(2.0, 0.0, 0.0, home_yaw)
    )
    await asyncio.sleep(3)
    print("  Back to hover")
    pos = await get_position(drone)
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, -5.0, home_yaw)
    )
    await asyncio.sleep(3)

    # Phase 10: Yaw step 90°
    print("=== Phase 10: Yaw step +90° (Izz validation) ===")
    target_yaw = home_yaw + 90.0
    if target_yaw > 180.0:
        target_yaw -= 360.0
    pos = await get_position(drone)
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, -5.0, target_yaw)
    )
    await asyncio.sleep(3)
    print("  Yaw back to original heading")
    await drone.offboard.set_position_ned(
        PositionNedYaw(pos.north_m, pos.east_m, -5.0, home_yaw)
    )
    await asyncio.sleep(3)

    # Phase 11: Land
    print("=== Phase 11: Landing ===")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()

    print("Waiting for disarm ...")
    async for armed in drone.telemetry.armed():
        if not armed:
            print("Disarmed. Mission complete.")
            break

    print()
    print("=" * 60)
    print("ULG log saved by PX4 in: build/px4_sitl_default/rootfs/log/")
    print("Use 'plotjuggler' or 'pyulog' to analyse the log.")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(run())
