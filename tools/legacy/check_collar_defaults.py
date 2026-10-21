#!/usr/bin/env python3
"""Fail if the free_target_object collar defaults diverge from CubeSatelliteModel::Params.

Reads the defaults of CubeSatelliteModel::Params straight from cube_satellite_model.hpp and the
xacro:arg defaults (and the launch-file defaults) of free_target_object, and compares them:

    Params.collar_radius_m   <-> xacro/launch collar_radius_m
    Params.wall_thickness_m  <-> xacro/launch wall_thickness_m
    Params.standoff_m        <-> xacro/launch standoff_m
    Params.cube_side_m       <-> xacro/launch size_x (the collar sits on the +x face)
    Params.face_normal_body  == UnitX <-> collar axis = link +x (fixed in the xacro)

Pure Python (no ROS needed). Exit 0 = consistent, 1 = divergence (message per mismatch).
Usage: tools/check_collar_defaults.py [repo_root]
"""
import ast
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HPP = "src/haptic_dmp_learning/include/haptic_dmp_learning/core/cube_satellite_model.hpp"
XACRO = "src/free_target_object/models/free_target_object.sdf.xacro"
LAUNCH = "src/free_target_object/launch/free_target_object.launch.py"

# planner Params field -> (xacro/launch arg name)
PAIRS = {
    "collar_radius_m": "collar_radius_m",
    "wall_thickness_m": "wall_thickness_m",
    "standoff_m": "standoff_m",
    "cube_side_m": "size_x",
}
TOL = 1e-12


def params_defaults(hpp_text):
    m = re.search(r"struct\s+Params\s*\{(.*?)\n\s*\};", hpp_text, re.S)
    if not m:
        sys.exit("check_collar_defaults: cannot find 'struct Params' in " + HPP)
    body = m.group(1)
    out = {}
    for name in list(PAIRS):
        mm = re.search(r"\bdouble\s+" + name + r"\s*=\s*([-+0-9.eE]+)\s*;", body)
        if not mm:
            sys.exit(f"check_collar_defaults: no 'double {name} = <number>;' in Params ({HPP})")
        out[name] = float(mm.group(1))
    mm = re.search(r"face_normal_body\s*=\s*Eigen::Vector3d::(\w+)\(\)", body)
    out["face_normal_body"] = mm.group(1) if mm else None
    return out


def xacro_defaults(xacro_text):
    root = ET.fromstring(xacro_text)
    ns = "{http://www.ros.org/wiki/xacro}"
    return {a.get("name"): a.get("default") for a in root.iter(ns + "arg")}


def launch_defaults(launch_text):
    tree = ast.parse(launch_text)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "_ARG_DEFAULTS" for t in node.targets):
            d = ast.literal_eval(node.value)
            return {k: v[0] for k, v in d.items()}
    sys.exit("check_collar_defaults: no _ARG_DEFAULTS in " + LAUNCH)


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent
    p = params_defaults((root / HPP).read_text())
    sources = {
        "xacro": xacro_defaults((root / XACRO).read_text()),
        "launch": launch_defaults((root / LAUNCH).read_text()),
    }
    errors = []
    for field, arg in PAIRS.items():
        for label, d in sources.items():
            if arg not in d:
                errors.append(f"{label}: argument '{arg}' missing (planner Params.{field} = {p[field]})")
                continue
            v = float(d[arg])
            if abs(v - p[field]) > TOL:
                errors.append(f"{label}: '{arg}' default {v} != CubeSatelliteModel::Params.{field} default "
                              f"{p[field]}")
    if p["face_normal_body"] != "UnitX":
        errors.append(f"CubeSatelliteModel::Params.face_normal_body default is Vector3d::{p['face_normal_body']}(), "
                      "but the xacro collar axis is fixed to the link +x (UnitX)")
    wc = sources["xacro"].get("with_collar")
    if wc is None or wc.lower() != "false":
        errors.append(f"xacro: with_collar default must be 'false' (got {wc!r}), else the default SDF changes")
    if errors:
        print("COLLAR DEFAULTS DIVERGE from CubeSatelliteModel::Params:", file=sys.stderr)
        for e in errors:
            print("  - " + e, file=sys.stderr)
        return 1
    print("OK: " + ", ".join(f"{a}={p[f]}" for f, a in PAIRS.items())
          + ", axis=+x (UnitX), with_collar=false; xacro and launch agree with "
          + "CubeSatelliteModel::Params")
    return 0


if __name__ == "__main__":
    sys.exit(main())
