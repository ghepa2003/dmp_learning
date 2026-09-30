#!/usr/bin/env python3
"""Numeric check of the collar in an expanded free_target_object SDF (with_collar=true).

Samples every collar box (collision, or visual when there is no collision) densely and compares
radial extent / x interval / axis with the CubeSatelliteModel values given on the command line
(defaults: collar_radius_m=0.05, wall_thickness_m=0.015, standoff_m=0.08, cube side 0.2, axis +x).

Expected polygonal-approximation tolerance for N segments and overlap o (flats at R and r):
    outer radius   in [R, hypot(R, w/2)], w = 2 R tan(pi/N) (1+o)   (outer box corners)
    inner radius   free bore >= r everywhere (flats at r), max r/cos(pi/N)
Usage: check_collar_sdf.py expanded.sdf [--segments N] [--radius R] [--wall W] [--standoff S] [--side L]
Exit 0 = within tolerance.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("--segments", type=int, default=24)
    ap.add_argument("--radius", type=float, default=0.05)
    ap.add_argument("--wall", type=float, default=0.015)
    ap.add_argument("--standoff", type=float, default=0.08)
    ap.add_argument("--side", type=float, default=0.2)
    ap.add_argument("--overlap", type=float, default=0.05)
    a = ap.parse_args()
    R, r, N = a.radius, a.radius - a.wall, a.segments

    link = ET.parse(a.sdf).getroot().find("model/link[@name='body']")
    tag = "collision"
    boxes = [e for e in link.findall(tag) if e.get("name", "").startswith("collar_seg_")]
    if not boxes:
        tag = "visual"
        boxes = [e for e in link.findall(tag) if e.get("name", "").startswith("collar_seg_")]
    names = [e.get("name") for e in boxes]
    errs = []
    if len(boxes) != N:
        errs.append(f"segment count {len(boxes)} != {N}")
    expect = [f"collar_seg_{i}" + ("" if tag == "collision" else "_visual") for i in range(N)]
    if names != expect:
        errs.append("segment names not collar_seg_0..N-1 in order")

    pts = []
    for e in boxes:
        pose = [float(v) for v in e.findtext("pose").split()]
        size = [float(v) for v in e.findtext("geometry/box/size").split()]
        Rm = rpy(*pose[3:])
        g = [np.linspace(-s / 2, s / 2, 9) for s in size]
        loc = np.array([[x, y, z] for x in g[0] for y in g[1] for z in g[2]])
        pts.append(loc @ Rm.T + np.array(pose[:3]))
    P = np.vstack(pts)
    rad = np.hypot(P[:, 1], P[:, 2])
    x0, x1 = P[:, 0].min(), P[:, 0].max()
    # axis: the ring is centred on y=z=0 and every segment has x-length = standoff along link x
    cy, cz = P[:, 1].mean(), P[:, 2].mean()

    # Bore clearance: minimum radius of the MATERIAL along the 360 deg directions = inner radius.
    # Material min radius is the smallest |p|; the bore polygon has circumradius r/cos(pi/N).
    w = 2 * R * math.tan(math.pi / N) * (1 + a.overlap)
    tol_out = math.hypot(R, w / 2) - R  # outer box corner
    print(f"file {a.sdf}  ({tag} boxes)")
    print(f"segments           : {len(boxes)} (expected {N})")
    print(f"radial min / max   : {rad.min()*1000:.3f} / {rad.max()*1000:.3f} mm "
          f"(planner inner {r*1000:.3f}, outer {R*1000:.3f} mm)")
    print(f"x interval         : [{x0:.6f}, {x1:.6f}] m (expected [{a.side/2:.6f}, {a.side/2+a.standoff:.6f}])")
    print(f"axis offset y,z    : {cy*1000:.4f}, {cz*1000:.4f} mm (expected 0,0: axis = link +x)")
    if abs(rad.min() - r) > 1e-6:
        errs.append(f"min radius {rad.min():.6f} != inner radius {r:.6f} (flats at r must give exactly r)")
    if not (R - 1e-6 <= rad.max() <= R + tol_out + 1e-6):
        errs.append(f"max radius {rad.max():.6f} outside [{R}, {R + tol_out:.6f}]")
    if abs(x0 - a.side / 2) > 1e-9 or abs(x1 - (a.side / 2 + a.standoff)) > 1e-9:
        errs.append("x interval mismatch")
    if abs(cy) > 1e-9 or abs(cz) > 1e-9:
        errs.append("ring not centred on the link +x axis")
    # No gap: every direction must hit material at radius in (r, R): check 720 directions at mid radius
    rm = 0.5 * (R + r)
    cov = 0
    for k in range(720):
        ph = 2 * math.pi * (k + 0.5) / 720
        q = np.array([rm * math.cos(ph), rm * math.sin(ph)])
        ok = False
        for e in boxes:
            pose = [float(v) for v in e.findtext("pose").split()]
            size = [float(v) for v in e.findtext("geometry/box/size").split()]
            v = q - np.array(pose[1:3])
            c, s = math.cos(pose[3]), math.sin(pose[3])
            ly, lz = c * v[0] + s * v[1], -s * v[0] + c * v[1]
            if abs(ly) <= size[1] / 2 + 1e-12 and abs(lz) <= size[2] / 2 + 1e-12:
                ok = True
                break
        cov += ok
    print(f"gap check          : {cov}/720 directions covered at mid radius")
    if cov != 720:
        errs.append("gap in the ring")
    print(f"polygon tolerance  : outer up to +{tol_out*1000:.3f} mm beyond R, bore circumradius "
          f"{r/math.cos(math.pi/N)*1000:.3f} mm (r={r*1000:.3f})")
    if errs:
        print("FAIL:", *errs, sep="\n  - ", file=sys.stderr)
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
