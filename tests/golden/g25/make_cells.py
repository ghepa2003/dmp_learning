#!/usr/bin/env python3
import sys
import os
import math
import numpy as np

# Add tools/ to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../tools')))
from s1_run import tilt_axis, tilt_rotation, tilt_quat_xyzw

NOMINAL_CENTER = np.array([0.70, 0.00, 0.77], dtype=float)

CELLS_16 = [
    ("C0",  -2.00, (0, 0, 0),        0.0,   0.0),
    ("C1",  -1.00, (0, 0, 0),        0.0,   0.0),
    ("C2",  -1.50, (0, 0, 0),        0.0,   0.0),
    ("C3",  -2.50, (0, 0, 0),        0.0,   0.0),
    ("C4",  -3.00, (0, 0, 0),        0.0,   0.0),
    ("C5",  -2.00, (0, 0, 0),       20.0,   0.0),
    ("C6",  -2.00, (0, 0, 0),       20.0,  90.0),
    ("C7",  -2.00, (0, 0, 0),       20.0, 180.0),
    ("C8",  -2.00, (0, 0, 0),       20.0, 270.0),
    ("C9",  -1.00, (50, -50, 50),   10.0,  45.0),
    ("C10", -1.50, (-50, 50, 50),   20.0, 135.0),
    ("C11", -1.75, (0, -50, -50),   15.0, 315.0),
    ("C12", -2.00, (50, 50, -50),    5.0, 225.0),
    ("C13", -2.25, (-50, 0, 50),    20.0, 300.0),
    ("C14", -2.50, (-50, -50, -50), 15.0,  90.0),
    ("C15", -3.00, (50, 0, 50),     10.0, 180.0),
]

CTRL_ROWS = [
    ("CTRL_alpha0_phi123", -3.00, (0, 0, 0), 0.0, 123.4),
    ("CTRL_nominal",        -3.00, (0, 0, 0), 0.0,   0.0),
]

G19_ROWS = [
    ("G19_om3", -3.00, (0, 0, 0), 0.0, 0.0),
    ("G19_om4", -4.00, (0, 0, 0), 0.0, 0.0),
    ("G19_om5", -5.00, (0, 0, 0), 0.0, 0.0),
    ("G19_om6", -6.00, (0, 0, 0), 0.0, 0.0),
]

def process_row(name, omega, offset_mm, alpha, phi):
    offset_m = np.array(offset_mm, dtype=float) * 1e-3
    center = NOMINAL_CENTER + offset_m

    a = tilt_axis(alpha, phi)
    R_tilt = tilt_rotation(alpha, phi)
    qx, qy, qz, qw = tilt_quat_xyzw(alpha, phi)
    n = R_tilt @ np.array([1.0, 0.0, 0.0], dtype=float)

    # Verifiche
    norm_a = np.linalg.norm(a)
    dot_na = abs(float(np.dot(n, a)))
    diff_Rz_a = np.max(np.abs(R_tilt @ np.array([0.0, 0.0, 1.0], dtype=float) - a))

    assert abs(norm_a - 1.0) < 1e-14, f"{name}: |a| - 1 = {abs(norm_a - 1.0)}"
    assert dot_na < 1e-12, f"{name}: |n.a| = {dot_na} >= 1e-12"
    assert diff_Rz_a < 1e-14, f"{name}: max|R_tilt*z - a| = {diff_Rz_a}"

    log_line = (f"{name:18s} | |a|={norm_a:.16f} | |n.a|={dot_na:.3e} | "
                f"max|R_tilt*z - a|={diff_Rz_a:.3e} | a=[{a[0]:.6f}, {a[1]:.6f}, {a[2]:.6f}] "
                f"n=[{n[0]:.6f}, {n[1]:.6f}, {n[2]:.6f}]")

    csv_fields = [
        name,
        repr(float(omega)),
        repr(float(center[0])), repr(float(center[1])), repr(float(center[2])),
        repr(float(alpha)), repr(float(phi)),
        repr(float(a[0])), repr(float(a[1])), repr(float(a[2])),
        repr(float(qx)), repr(float(qy)), repr(float(qz)), repr(float(qw)),
        repr(float(n[0])), repr(float(n[1])), repr(float(n[2]))
    ]
    return log_line, ",".join(csv_fields)

def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(out_dir, "g25_cells_input.csv")
    g19_file = os.path.join(out_dir, "g25_g19_input.csv")

    header = "cell,omega_deg_s,cx,cy,cz,alpha_deg,phi_deg,ax,ay,az,qx,qy,qz,qw,nx,ny,nz"

    lines_csv = [header]
    print("=== PASSO 1: VERIFICA RIGHE CELLE ===")
    
    # 16 cells
    for spec in CELLS_16:
        log_l, csv_l = process_row(*spec)
        print(log_l)
        lines_csv.append(csv_l)

    # 2 CTRL rows
    print("\n=== PASSO 1: VERIFICA RIGHE CONTROLLO ===")
    for spec in CTRL_ROWS:
        log_l, csv_l = process_row(*spec)
        print(log_l)
        lines_csv.append(csv_l)

    with open(csv_file, "w") as f:
        f.write("\n".join(lines_csv) + "\n")
    print(f"\nScritto {csv_file} ({len(lines_csv)-1} righe)")

    # G19 rows
    g19_lines = [header]
    for spec in G19_ROWS:
        _, csv_l = process_row(*spec)
        g19_lines.append(csv_l)
    with open(g19_file, "w") as f:
        f.write("\n".join(g19_lines) + "\n")
    print(f"Scritto {g19_file} ({len(g19_lines)-1} righe)")

if __name__ == "__main__":
    main()
