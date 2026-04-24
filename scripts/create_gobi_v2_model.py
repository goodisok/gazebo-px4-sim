#!/usr/bin/env python3
"""Create Gobi v2: x500 geometry + Gobi mass/inertia + QuadraticDrag (v²) plugin."""

import shutil
from pathlib import Path

PX4 = Path(__file__).resolve().parent.parent / "PX4-Autopilot"
M = PX4 / "Tools/simulation/gz/models"
AF = PX4 / "ROMFS/px4fmu_common/init.d-posix/airframes"


def main():
    b = M / "gobi_v2_base"
    if b.exists():
        shutil.rmtree(b)
    shutil.copytree(M / "x500_base", b)
    s = (b / "model.sdf").read_text()
    s = s.replace("name='x500_base'", "name='gobi_v2_base'")
    s = s.replace("<mass>2.0</mass>", "<mass>2.2</mass>")
    s = s.replace("<ixx>0.02166666666666667</ixx>", "<ixx>0.012</ixx>")
    s = s.replace("<iyy>0.02166666666666667</iyy>", "<iyy>0.012</iyy>")
    s = s.replace("<izz>0.04000000000000001</izz>", "<izz>0.020</izz>")
    (b / "model.sdf").write_text(s)
    if (b / "model.config").exists():
        c = (b / "model.config").read_text()
        c = c.replace("x500_base", "gobi_v2_base")
        (b / "model.config").write_text(c)

    d = M / "gobi_v2"
    if d.exists():
        shutil.rmtree(d)
    shutil.copytree(M / "x500", d)
    s = (d / "model.sdf").read_text()
    s = s.replace("model://x500_base", "model://gobi_v2_base")
    s = s.replace("name='x500'", "name='gobi_v2'")
    s = s.replace(
        "<motorConstant>8.54858e-06</motorConstant>",
        "<motorConstant>3.746875e-05</motorConstant>")
    s = s.replace(
        "<maxRotVelocity>1000.0</maxRotVelocity>",
        "<maxRotVelocity>1200.0</maxRotVelocity>")
    plug = """
    <plugin filename="QuadraticDrag" name="quadratic_drag::QuadraticDrag">
      <link_name>base_link</link_name>
      <CdA>0.020</CdA>
      <air_density>1.225</air_density>
    </plugin>"""
    s = s.replace("</model>", plug + "\n  </model>")
    (d / "model.sdf").write_text(s)
    if (d / "model.config").exists():
        c = (d / "model.config").read_text()
        c = c.replace("x500", "gobi_v2")
        (d / "model.config").write_text(c)
    print("Wrote", b, d)


if __name__ == "__main__":
    main()
