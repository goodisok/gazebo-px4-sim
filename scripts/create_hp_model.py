#!/usr/bin/env python3
"""
Create a high-performance x500 model for high-speed interceptor testing.

Changes from standard x500:
- motorConstant: 2.73e-5 (3.2x) → T/W ≈ 8
- maxRotVelocity: 1200 rad/s
- Added velocity_decay for body drag simulation
"""

import shutil
import sys
from pathlib import Path

PX4_ROOT = Path(__file__).parent.parent / "PX4-Autopilot"
MODELS_DIR = PX4_ROOT / "Tools/simulation/gz/models"
AIRFRAME_DIR = PX4_ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes"

HP_MOTOR_CONSTANT = "2.73e-05"
HP_MAX_ROT_VEL = "1200.0"
HP_MOMENT_CONSTANT = "0.016"


def create_hp_base_model():
    """Create x500_hp_base from x500_base with added drag."""
    src = MODELS_DIR / "x500_base"
    dst = MODELS_DIR / "x500_hp_base"
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)

    sdf_path = dst / "model.sdf"
    sdf = sdf_path.read_text()

    # Add velocity_decay for body drag (linear approximation)
    # Cd * A ≈ 0.04 m², at v=50 m/s want ~60N drag for 2kg drone
    # F_drag = decay_coeff * v (linear model in Gazebo)
    # For v=50: F = 60 → decay = 60/50 = 1.2 per link
    # But velocity_decay in Gazebo is per-link, applied as damping
    # A moderate value simulates realistic drag
    drag_xml = """
      <velocity_decay>
        <linear>0.3</linear>
        <angular>0.01</angular>
      </velocity_decay>"""

    # Insert velocity_decay into the base_link
    sdf = sdf.replace(
        "</inertial>",
        "</inertial>" + drag_xml,
        1  # only first occurrence (base_link)
    )

    # Update model name
    sdf = sdf.replace("name='x500_base'", "name='x500_hp_base'")

    sdf_path.write_text(sdf)

    # Update model.config
    config_path = dst / "model.config"
    if config_path.exists():
        cfg = config_path.read_text()
        cfg = cfg.replace("x500_base", "x500_hp_base")
        cfg = cfg.replace("x500 base", "x500 high-performance base")
        config_path.write_text(cfg)

    print(f"Created {dst}")
    return dst


def create_hp_model():
    """Create x500_hp from x500 with HP motor constants."""
    src = MODELS_DIR / "x500"
    dst = MODELS_DIR / "x500_hp"
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)

    sdf_path = dst / "model.sdf"
    sdf = sdf_path.read_text()

    # Change base model reference
    sdf = sdf.replace("model://x500_base", "model://x500_hp_base")
    sdf = sdf.replace("name='x500'", "name='x500_hp'")

    # Update motor parameters
    sdf = sdf.replace(
        "<motorConstant>8.54858e-06</motorConstant>",
        f"<motorConstant>{HP_MOTOR_CONSTANT}</motorConstant>")
    sdf = sdf.replace(
        "<maxRotVelocity>1000.0</maxRotVelocity>",
        f"<maxRotVelocity>{HP_MAX_ROT_VEL}</maxRotVelocity>")

    sdf_path.write_text(sdf)

    # Update model.config
    config_path = dst / "model.config"
    if config_path.exists():
        cfg = config_path.read_text()
        cfg = cfg.replace("x500", "x500_hp")
        config_path.write_text(cfg)

    print(f"Created {dst}")
    return dst


def create_hp_airframe():
    """Create PX4 airframe config for x500_hp."""
    # Base on airframe 4001 (x500)
    airframe_id = "4010"
    src = AIRFRAME_DIR / "4001_gz_x500"
    dst = AIRFRAME_DIR / f"{airframe_id}_gz_x500_hp"

    if src.exists():
        content = src.read_text()
    else:
        content = """#!/bin/sh
# @name Gazebo x500
# @type Quadrotor x
# @class Copter
# @output MAIN1 motor 1
# @output MAIN2 motor 2
# @output MAIN3 motor 3
# @output MAIN4 motor 4

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}
"""

    content = content.replace("x500", "x500_hp")
    content = content.replace("Gazebo x500_hp", "Gazebo x500 High-Performance")

    # Add high-speed parameter overrides
    if "param set-default" not in content:
        content += "\n"

    hp_params = """
# High-performance interceptor parameters
param set-default MPC_XY_VEL_MAX 50.0
param set-default MPC_Z_VEL_MAX_UP 15.0
param set-default MPC_Z_VEL_MAX_DN 10.0
param set-default MPC_TILTMAX_AIR 75.0
param set-default MPC_MAN_TILT_MAX 75.0
param set-default MPC_XY_P 1.2
param set-default MPC_XY_VEL_P_ACC 3.0
param set-default MPC_XY_VEL_D_ACC 0.3
"""
    content += hp_params

    dst.write_text(content)
    dst.chmod(0o755)
    print(f"Created airframe: {dst}")
    return dst


def main():
    print("Creating high-performance x500 model for interceptor testing...")
    print(f"Target: T/W ≈ 8, max speed ≈ 50 m/s")
    print(f"  motorConstant: {HP_MOTOR_CONSTANT}")
    print(f"  maxRotVelocity: {HP_MAX_ROT_VEL}")
    print()

    create_hp_base_model()
    create_hp_model()
    create_hp_airframe()

    print("\nDone. To run:")
    print(f"  PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL=x500_hp make px4_sitl gz_x500_hp")


if __name__ == "__main__":
    main()
