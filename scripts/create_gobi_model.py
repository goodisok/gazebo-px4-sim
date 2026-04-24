#!/usr/bin/env python3
"""
Create a Gobi-class high-speed interceptor model for Gazebo+PX4.

Based on x500_hp but with parameters matching Gobi-class drones:
- Mass: 2.2 kg (compact body)
- Inertia: smaller (compact quadrotor, 32×34×34 cm)
- T/W ≈ 10 (high-speed sprint capability)
- velocity_decay = 0.3 (Gazebo linear drag — known limitation vs real v² drag)
- MPC_XY_VEL_MAX = 100 m/s (allows up to 97 m/s test)
- MPC_TILTMAX_AIR = 85° (extreme tilt for high-speed flight)
"""

import shutil
from pathlib import Path

PX4_ROOT = Path(__file__).parent.parent / "PX4-Autopilot"
MODELS_DIR = PX4_ROOT / "Tools/simulation/gz/models"
AIRFRAME_DIR = PX4_ROOT / "ROMFS/px4fmu_common/init.d-posix/airframes"

GOBI_MASS = 2.2
GOBI_IXX = 0.012
GOBI_IYY = 0.012
GOBI_IZZ = 0.020
GOBI_TW = 10
GOBI_MAX_ROT_VEL = 1200.0
GOBI_MOTOR_CONSTANT = (GOBI_MASS * 9.81 * GOBI_TW) / (4 * GOBI_MAX_ROT_VEL**2)
GOBI_VELOCITY_DECAY = 0.3
GOBI_AIRFRAME_ID = 4098


def create_gobi_base_model():
    """Create gobi_base from x500_base with Gobi mass/inertia and drag."""
    src = MODELS_DIR / "x500_base"
    dst = MODELS_DIR / "gobi_base"
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)

    sdf_path = dst / "model.sdf"
    sdf = sdf_path.read_text()

    sdf = sdf.replace("name='x500_base'", "name='gobi_base'")

    sdf = sdf.replace("<mass>2.0</mass>", f"<mass>{GOBI_MASS}</mass>")
    sdf = sdf.replace(
        "<ixx>0.02166666666666667</ixx>",
        f"<ixx>{GOBI_IXX}</ixx>")
    sdf = sdf.replace(
        "<iyy>0.02166666666666667</iyy>",
        f"<iyy>{GOBI_IYY}</iyy>")
    sdf = sdf.replace(
        "<izz>0.04000000000000001</izz>",
        f"<izz>{GOBI_IZZ}</izz>")

    drag_xml = f"""
      <velocity_decay>
        <linear>{GOBI_VELOCITY_DECAY}</linear>
        <angular>0.01</angular>
      </velocity_decay>"""

    sdf = sdf.replace("</inertial>", "</inertial>" + drag_xml, 1)

    config_path = dst / "model.config"
    if config_path.exists():
        cfg = config_path.read_text()
        cfg = cfg.replace("x500_base", "gobi_base")
        cfg = cfg.replace("x500 base", "Gobi interceptor base")
        config_path.write_text(cfg)

    sdf_path.write_text(sdf)
    print(f"Created {dst}")
    print(f"  mass={GOBI_MASS} kg, Ixx={GOBI_IXX}, Iyy={GOBI_IYY}, Izz={GOBI_IZZ}")
    print(f"  velocity_decay={GOBI_VELOCITY_DECAY}")


def create_gobi_model():
    """Create gobi from x500 with Gobi motor constants."""
    src = MODELS_DIR / "x500"
    dst = MODELS_DIR / "gobi"
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)

    sdf_path = dst / "model.sdf"
    sdf = sdf_path.read_text()

    sdf = sdf.replace("model://x500_base", "model://gobi_base")
    sdf = sdf.replace("name='x500'", "name='gobi'")

    sdf = sdf.replace(
        "<motorConstant>8.54858e-06</motorConstant>",
        f"<motorConstant>{GOBI_MOTOR_CONSTANT:.6e}</motorConstant>")
    sdf = sdf.replace(
        "<maxRotVelocity>1000.0</maxRotVelocity>",
        f"<maxRotVelocity>{GOBI_MAX_ROT_VEL}</maxRotVelocity>")

    sdf_path.write_text(sdf)

    config_path = dst / "model.config"
    if config_path.exists():
        cfg = config_path.read_text()
        cfg = cfg.replace("x500", "gobi")
        config_path.write_text(cfg)

    print(f"Created {dst}")
    max_thrust_per_motor = GOBI_MOTOR_CONSTANT * GOBI_MAX_ROT_VEL**2
    total_thrust = 4 * max_thrust_per_motor
    tw = total_thrust / (GOBI_MASS * 9.81)
    print(f"  motorConstant={GOBI_MOTOR_CONSTANT:.6e}")
    print(f"  maxRotVelocity={GOBI_MAX_ROT_VEL}")
    print(f"  Max thrust/motor={max_thrust_per_motor:.1f} N, Total={total_thrust:.1f} N")
    print(f"  T/W={tw:.1f}")


def create_gobi_airframe():
    """Create PX4 airframe config for Gobi interceptor."""
    src = AIRFRAME_DIR / "4001_gz_x500"
    dst = AIRFRAME_DIR / f"{GOBI_AIRFRAME_ID}_gz_gobi"

    if src.exists():
        content = src.read_text()
    else:
        content = """#!/bin/sh
. ${R}etc/init.d/rc.mc_defaults
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
"""

    content = f"""#!/bin/sh
#
# @name Gazebo Gobi High-Speed Interceptor
#
# @type Quadrotor X
#

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=default}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:=gobi}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

param set-default MPC_THR_HOVER 0.12
param set-default NAV_DLL_ACT 2

# Gobi interceptor high-speed parameters
param set-default MPC_XY_VEL_MAX 100.0
param set-default MPC_Z_VEL_MAX_UP 20.0
param set-default MPC_Z_VEL_MAX_DN 15.0
param set-default MPC_TILTMAX_AIR 85.0
param set-default MPC_MAN_TILT_MAX 85.0
param set-default MPC_XY_P 1.2
param set-default MPC_XY_VEL_P_ACC 3.0
param set-default MPC_XY_VEL_D_ACC 0.3
"""

    dst.write_text(content)
    dst.chmod(0o755)
    print(f"Created airframe: {dst}")
    print(f"  MPC_XY_VEL_MAX=100, MPC_TILTMAX_AIR=85°")


def register_airframe_cmake():
    """Add gobi airframe to CMakeLists.txt if not already there."""
    cmake_path = AIRFRAME_DIR / "CMakeLists.txt"
    if not cmake_path.exists():
        return
    content = cmake_path.read_text()
    entry = f"\t{GOBI_AIRFRAME_ID}_gz_gobi"
    if entry.strip() in content:
        print(f"Airframe already in CMakeLists.txt")
        return
    content = content.replace(
        "\t4099_gz_x500_hp",
        f"\t4099_gz_x500_hp\n{entry}")
    cmake_path.write_text(content)
    print(f"Added {GOBI_AIRFRAME_ID}_gz_gobi to CMakeLists.txt")


def main():
    print("=" * 60)
    print("Creating Gobi-class Interceptor Model for Gazebo+PX4")
    print("=" * 60)
    print(f"  Mass: {GOBI_MASS} kg")
    print(f"  T/W: ~{GOBI_TW}")
    print(f"  motorConstant: {GOBI_MOTOR_CONSTANT:.6e}")
    print(f"  Max speed target: 97 m/s (350 km/h)")
    print(f"  Drag model: linear velocity_decay={GOBI_VELOCITY_DECAY}")
    print()

    create_gobi_base_model()
    print()
    create_gobi_model()
    print()
    create_gobi_airframe()
    print()
    register_airframe_cmake()

    print()
    print("To build and run:")
    print(f"  cd PX4-Autopilot")
    print(f"  rm -f build/px4_sitl_default/rootfs/parameters.bson build/px4_sitl_default/rootfs/dataman")
    print(f"  PX4_SYS_AUTOSTART={GOBI_AIRFRAME_ID} PX4_GZ_MODEL=gobi HEADLESS=1 make px4_sitl gz_x500")


if __name__ == "__main__":
    main()
