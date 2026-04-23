#!/usr/bin/env python3
"""
Update interceptor model.sdf parameters for iterative Sim-to-Real tuning.
Modifies mass, inertia, and motor parameters in the self-contained SDF file.

Usage:
    python3 update_interceptor_params.py --mass 2.0 --ixx 0.02167 --iyy 0.02167 --motorConstant 8.54858e-06 --timeConstantUp 0.0125
"""

import argparse
import re
import os

SDF_PATH = os.path.join(os.path.dirname(__file__), "..",
    "PX4-Autopilot/Tools/simulation/gz/models/interceptor/model.sdf")


def replace_tag(content, tag, value, occurrence="first_in_base_link"):
    if occurrence == "all":
        pattern = rf"(<{tag}>)[^<]*(<!--[^>]*-->)?[^<]*(</{tag}>)"
        return re.sub(pattern, rf"\g<1>{value}\3", content)
    elif occurrence == "first_in_base_link":
        base_link_match = re.search(r'<link name="base_link">', content)
        if not base_link_match:
            return content
        start = base_link_match.start()
        end_link = content.index("</link>", start) + len("</link>")
        section = content[start:end_link]
        pattern = rf"(<{tag}>)[^<]*(</{tag}>)"
        new_section = re.sub(pattern, rf"\g<1>{value}\2", section, count=1)
        return content[:start] + new_section + content[end_link:]
    return content


def replace_motor_param(content, param_name, value):
    pattern = rf"(<{param_name}>)[^<]*(</{param_name}>)"
    return re.sub(pattern, rf"\g<1>{value}\2", content)


def main():
    parser = argparse.ArgumentParser(description="Update interceptor SDF parameters")
    parser.add_argument("--mass", type=float, default=None)
    parser.add_argument("--ixx", type=float, default=None)
    parser.add_argument("--iyy", type=float, default=None)
    parser.add_argument("--izz", type=float, default=None)
    parser.add_argument("--motorConstant", type=float, default=None)
    parser.add_argument("--timeConstantUp", type=float, default=None)
    parser.add_argument("--timeConstantDown", type=float, default=None)
    args = parser.parse_args()

    with open(SDF_PATH, "r") as f:
        content = f.read()

    changes = []
    if args.mass is not None:
        content = replace_tag(content, "mass", str(args.mass), "first_in_base_link")
        changes.append(f"mass → {args.mass}")
    if args.ixx is not None:
        content = replace_tag(content, "ixx", str(args.ixx), "first_in_base_link")
        changes.append(f"Ixx → {args.ixx}")
    if args.iyy is not None:
        content = replace_tag(content, "iyy", str(args.iyy), "first_in_base_link")
        changes.append(f"Iyy → {args.iyy}")
    if args.izz is not None:
        content = replace_tag(content, "izz", str(args.izz), "first_in_base_link")
        changes.append(f"Izz → {args.izz}")
    if args.motorConstant is not None:
        content = replace_motor_param(content, "motorConstant", str(args.motorConstant))
        changes.append(f"motorConstant → {args.motorConstant}")
    if args.timeConstantUp is not None:
        content = replace_motor_param(content, "timeConstantUp", str(args.timeConstantUp))
        changes.append(f"timeConstantUp → {args.timeConstantUp}")
    if args.timeConstantDown is not None:
        content = replace_motor_param(content, "timeConstantDown", str(args.timeConstantDown))
        changes.append(f"timeConstantDown → {args.timeConstantDown}")

    with open(SDF_PATH, "w") as f:
        f.write(content)

    if changes:
        print(f"Updated {SDF_PATH}:")
        for c in changes:
            print(f"  {c}")
    else:
        print("No changes specified.")


if __name__ == "__main__":
    main()
