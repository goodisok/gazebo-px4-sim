#!/usr/bin/env python3
"""
Multi-speed flight test for Gobi interceptor model in Gazebo+PX4.
Progressive speed from hover to 97 m/s, with sysid excitation at each stage.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw


def log(msg):
    print(msg, flush=True)


async def set_vel(drone, north, east, down, yaw, hold_s):
    await drone.offboard.set_velocity_ned(VelocityNedYaw(north, east, down, yaw))
    await asyncio.sleep(hold_s)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    log("Waiting for connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            log("Connected!")
            break

    log("Waiting for GPS fix...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log("GPS fix OK")
            break

    log("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(1)

    log("-- Taking off to 15m")
    await drone.action.set_takeoff_altitude(15.0)
    await drone.action.takeoff()
    await asyncio.sleep(12)

    log("-- Starting offboard mode")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.offboard.start()

    log("-- Hover baseline (8s)")
    await asyncio.sleep(8)

    log("-- Sysid doublets: north/east")
    for _ in range(3):
        await set_vel(drone, 8, 0, 0, 0, 0.8)
        await set_vel(drone, -8, 0, 0, 0, 0.8)
    for _ in range(3):
        await set_vel(drone, 0, 8, 0, 0, 0.8)
        await set_vel(drone, 0, -8, 0, 0, 0.8)
    await set_vel(drone, 0, 0, 0, 0, 3)

    speed_steps = [5, 10, 15, 25, 40, 50, 70, 85, 97]

    for speed in speed_steps:
        log(f"-- Forward (North): {speed} m/s")
        hold = max(3.0, min(8.0, 400 / max(speed, 1)))
        await set_vel(drone, speed, 0, 0, 0, hold)

        log(f"   Braking from {speed} m/s")
        brake = max(3.0, speed / 8)
        await set_vel(drone, 0, 0, 0, 0, brake)

    log("-- Final hover (5s)")
    await set_vel(drone, 0, 0, 0, 0, 5)

    log("-- Landing")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(15)

    try:
        await drone.action.disarm()
    except Exception:
        pass

    log("Flight complete!")


if __name__ == "__main__":
    asyncio.run(run())
