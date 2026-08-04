#!/usr/bin/env python3
"""Generates a synthetic "picking" trajectory (horizontal translation
followed by vertical translation, with smoothed transition) in the same CSV
format used by real demos (t,x,y,z,qw,qx,qy,qz).

Smoothing methodology: minimum-jerk profile applied independently to each axis.

Usage:
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.15 --dy 0.05 --dz -0.10 --output demo_synth_45s.csv

    # for different goals:
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.20 --dy -0.05 --dz -0.12 --output demo_synth_45s_goalB.csv
"""
import argparse
import csv
import math


def min_jerk_profile(t, t0, t1, start, end):
    """1D minimum-jerk position profile between start and end on window [t0, t1]."""
    if t <= t0:
        return start
    if t >= t1:
        return end
    s = (t - t0) / (t1 - t0)
    smoothed = 10 * s**3 - 15 * s**4 + 6 * s**5
    return start + (end - start) * smoothed


def min_jerk_blend(t, t0, t1):
    """Returns smoothed blend factor (0 before t0, 1 after t1)."""
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    s = (t - t0) / (t1 - t0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def quat_from_axis_angle(axis, angle_deg):
    """Constructs a quaternion (w,x,y,z) from axis and angle in degrees."""
    angle_rad = math.radians(angle_deg)
    n = math.sqrt(sum(c * c for c in axis))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    ax = tuple(c / n for c in axis)
    s = math.sin(angle_rad / 2.0)
    return (math.cos(angle_rad / 2.0), ax[0] * s, ax[1] * s, ax[2] * s)


def slerp(q0, q1, s):
    """Spherical linear interpolation between unit quaternions (w,x,y,z)."""
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-c for c in q1)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))

    if dot > 0.9995:
        result = tuple(a + s * (b - a) for a, b in zip(q0, q1))
    else:
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        theta = theta0 * s
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
        s1 = math.sin(theta) / sin_theta0
        result = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))

    norm = math.sqrt(sum(c * c for c in result))
    return tuple(c / norm for c in result)


def generate(duration, dx, dy, dz, dt=0.001, transition_duration=10.0,
             horizontal_window=(0.0, 0.6), vertical_window=(0.4, 1.0),
             y0=(0.0, 0.0, 0.0), q0=(1.0, 0.0, 0.0, 0.0),
             rot_axis=(0.0, 0.0, 1.0), rot_angle_deg=0.0):
    """Generates complete trajectory."""
    if transition_duration > duration:
        raise ValueError(
            f"transition_duration ({transition_duration}s) cannot exceed "
            f"duration ({duration}s)."
        )

    hx0, hx1 = horizontal_window[0] * transition_duration, horizontal_window[1] * transition_duration
    vz0, vz1 = vertical_window[0] * transition_duration, vertical_window[1] * transition_duration

    x0, y0_, z0 = y0
    q_target = quat_from_axis_angle(rot_axis, rot_angle_deg)

    n_samples = int(round(duration / dt)) + 1
    rows = []
    for k in range(n_samples):
        t = k * dt
        x = min_jerk_profile(t, hx0, hx1, x0, x0 + dx)
        y = min_jerk_profile(t, hx0, hx1, y0_, y0_ + dy)
        z = min_jerk_profile(t, vz0, vz1, z0, z0 + dz)

        if rot_angle_deg != 0.0:
            blend = min_jerk_blend(t, 0.0, transition_duration)
            qw, qx, qy, qz = slerp(q0, q_target, blend)
        else:
            qw, qx, qy, qz = q0

        rows.append((t, x, y, z, qw, qx, qy, qz))
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for row in rows:
            writer.writerow([f"{v:.6f}" for v in row])


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, required=True,
                    help="total trajectory duration in seconds (motion + hold)")
    p.add_argument("--transition-duration", type=float, default=10.0,
                    help="ABSOLUTE duration in seconds of motion phase (default 10s)")
    p.add_argument("--dx", type=float, default=0.15, help="x-axis horizontal displacement [m]")
    p.add_argument("--dy", type=float, default=0.0, help="y-axis horizontal displacement [m]")
    p.add_argument("--dz", type=float, default=-0.10, help="z-axis vertical displacement [m]")
    p.add_argument("--dt", type=float, default=0.001, help="sampling step [s] (default 1kHz)")
    p.add_argument("--h-start-frac", type=float, default=0.0, help="horizontal window start fraction")
    p.add_argument("--h-end-frac", type=float, default=0.6, help="horizontal window end fraction")
    p.add_argument("--v-start-frac", type=float, default=0.4, help="vertical window start fraction")
    p.add_argument("--v-end-frac", type=float, default=1.0, help="vertical window end fraction")
    p.add_argument("--rot-axis", type=str, default="0,0,1",
                    help="rotation axis 'x,y,z' (default 0,0,1 = yaw)")
    p.add_argument("--rot-angle-deg", type=float, default=0.0,
                    help="rotation angle in degrees (default 0 = no rotation)")
    p.add_argument("--output", required=True, help="output CSV path")
    args = p.parse_args()

    rot_axis = tuple(float(v) for v in args.rot_axis.split(","))
    if len(rot_axis) != 3:
        raise ValueError("--rot-axis must have 3 components, e.g. '0,0,1'")

    rows = generate(
        duration=args.duration, dx=args.dx, dy=args.dy, dz=args.dz, dt=args.dt,
        transition_duration=args.transition_duration,
        horizontal_window=(args.h_start_frac, args.h_end_frac),
        vertical_window=(args.v_start_frac, args.v_end_frac),
        rot_axis=rot_axis, rot_angle_deg=args.rot_angle_deg,
    )
    write_csv(rows, args.output)
    print(f"Wrote {args.output}: {len(rows)} samples, total duration {args.duration}s "
          f"(transition {args.transition_duration}s + hold {args.duration - args.transition_duration:.1f}s), "
          f"final goal = ({rows[-1][1]:.4f}, {rows[-1][2]:.4f}, {rows[-1][3]:.4f})")


if __name__ == "__main__":
    main()

