#!/usr/bin/env python3
"""Comparative plot of the two-stage moving average filter (test 05_vel_filt_test).

SIGNAL SCALE EXPLANATION:
Direct central differences on noisy data (raw) introduce massive noise spikes
after the second derivative:
  - Raw acceleration: from -1400 m/s² to +1200 m/s² (due to dt ~ 1ms)
  - Raw angular acceleration (eta_dot): from -74,000 rad/s² to +72,000 rad/s²

To prevent raw noise scale from crushing the filtered signal and ground truth
visually to zero, the script generates two columns for each plot:
  - Left Column (Wide Scale): Raw vs Filtered (shows noise reduction)
  - Right Column (Detailed Zoom): Filtered vs Ground Truth (shows signal fidelity)
"""

import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_demo(path):
    t, pos, quat = [], [], []
    import csv
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            pos.append([float(row["x"]), float(row["y"]), float(row["z"])])
            quat.append([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])
    return np.array(t), np.array(pos), np.array(quat)


def load_truth(path):
    t, vel, acc, eta, eta_dot = [], [], [], [], []
    import csv
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            vel.append([float(row["vx"]), float(row["vy"]), float(row["vz"])])
            acc.append([float(row["ax"]), float(row["ay"]), float(row["az"])])
            eta.append([float(row["etax"]), float(row["etay"]), float(row["etaz"])])
            eta_dot.append([float(row["eta_dot_x"]), float(row["eta_dot_y"]), float(row["eta_dot_z"])])
    return np.array(t), np.array(vel), np.array(acc), np.array(eta), np.array(eta_dot)


def quat_normalize(q):
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else q


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def log_map(q):
    v = q[1:4]
    vnorm = np.linalg.norm(v)
    if vnorm < 1e-8:
        return np.zeros(3)
    w = max(-1.0, min(1.0, q[0]))
    angle = math.acos(w)
    return angle * v / vnorm


def central_diff(t, signal):
    N = len(t)
    out = np.zeros_like(signal)
    for k in range(N):
        km1 = max(0, k - 1)
        kp1 = min(N - 1, k + 1)
        dt = t[kp1] - t[km1]
        if dt <= 0.0:
            dt = 1e-6
        out[k] = (signal[kp1] - signal[km1]) / dt
    return out


def moving_average_smooth(signal, window_samples):
    N = len(signal)
    half = window_samples // 2
    out = np.zeros_like(signal)
    for k in range(N):
        lo = max(0, k - half)
        hi = min(N, k + half + 1)
        out[k] = signal[lo:hi].mean(axis=0)
    return out


def unwrap_rotation_vector(t, quat):
    N = len(t)
    r = np.zeros((N, 3))
    for k in range(1, N):
        dt = t[k] - t[k - 1]
        if dt <= 0.0:
            dt = 1e-6
        qk = quat_normalize(quat[k])
        qkm1 = quat_normalize(quat[k - 1])
        dq = quat_multiply(qk, quat_conjugate(qkm1))
        incr = 2.0 * log_map(dq)
        r[k] = r[k - 1] + incr
    return r


def compute_raw_signals(t, pos, quat, tau):
    vel = central_diff(t, pos)
    acc = central_diff(t, vel)

    N = len(t)
    eta = np.zeros((N, 3))
    for k in range(N):
        km1 = max(0, k - 1)
        kp1 = min(N - 1, k + 1)
        dt = t[kp1] - t[km1]
        if dt <= 0.0:
            dt = 1e-6
        qk1 = quat_normalize(quat[kp1])
        qk0 = quat_normalize(quat[km1])
        dq = quat_multiply(qk1, quat_conjugate(qk0))
        omega = 2.0 * log_map(dq) / dt
        eta[k] = tau * omega
    eta_dot = central_diff(t, eta)
    return vel, acc, eta, eta_dot


def compute_filtered_signals(t, pos, quat, tau, dt, w1_sec, w2_sec):
    w1 = max(1, int(round(w1_sec / dt)))
    w2 = max(1, int(round(w2_sec / dt)))

    pos_s = moving_average_smooth(pos, w1)
    vel = central_diff(t, pos_s)
    vel_s = moving_average_smooth(vel, w2)
    acc = central_diff(t, vel_s)

    r = unwrap_rotation_vector(t, quat)
    r_s = moving_average_smooth(r, w1)
    eta = tau * central_diff(t, r_s)
    eta_s = moving_average_smooth(eta, w2)
    eta_dot = central_diff(t, eta_s)

    return vel, acc, eta, eta_dot


def plot_dual_scale_comparison(t, raw_data, filt_data, true_data, title, y_label, output_filename):
    """Creates a 2-column x 4-row plot:
    - Left Column: Raw vs Filtered (Global Scale for noise)
    - Right Column: Filtered vs Ground Truth (Detailed Zoom Scale)
    """
    raw_norm = np.linalg.norm(raw_data, axis=1)
    filt_norm = np.linalg.norm(filt_data, axis=1)
    true_norm = np.linalg.norm(true_data, axis=1)

    signals = [
        ("Magnitude (Norm)", raw_norm, filt_norm, true_norm),
        ("X Component", raw_data[:, 0], filt_data[:, 0], true_data[:, 0]),
        ("Y Component", raw_data[:, 1], filt_data[:, 1], true_data[:, 1]),
        ("Z Component", raw_data[:, 2], filt_data[:, 2], true_data[:, 2]),
    ]

    fig, axes = plt.subplots(4, 2, figsize=(15, 11), sharex=True)

    for idx, (sub_title, raw_s, filt_s, true_s) in enumerate(signals):
        # Column 0: Raw vs Filtered vs True (full scale)
        ax_left = axes[idx, 0]
        ax_left.plot(t, raw_s, color="crimson", alpha=0.4, linewidth=0.7, label="Raw (Noisy)")
        ax_left.plot(t, filt_s, color="dodgerblue", linewidth=1.5, label="Filtered (Moving average)")
        ax_left.plot(t, true_s, color="black", linestyle="--", linewidth=1.2, label="Ground Truth")
        ax_left.set_ylabel(y_label)
        ax_left.set_title(f"{sub_title} - Global View (Noise Reduction)", fontsize=9.5, fontweight="bold")
        ax_left.grid(True, alpha=0.3)
        if idx == 0:
            ax_left.legend(loc="upper right", fontsize=8)

        # Column 1: Detailed ZOOM Filtered vs Ground Truth
        ax_right = axes[idx, 1]
        ax_right.plot(t, filt_s, color="dodgerblue", linewidth=1.8, label="Filtered (Moving average)")
        ax_right.plot(t, true_s, color="black", linestyle="--", linewidth=1.5, label="Ground Truth")
        ax_right.set_ylabel(y_label)
        ax_right.set_title(f"{sub_title} - DETAILED ZOOM (Filtered vs Truth)", fontsize=9.5, fontweight="bold")
        ax_right.grid(True, alpha=0.3)

        margin_idx = max(1, len(t) // 50)
        v_min = min(np.min(filt_s[margin_idx:-margin_idx]), np.min(true_s[margin_idx:-margin_idx]))
        v_max = max(np.max(filt_s[margin_idx:-margin_idx]), np.max(true_s[margin_idx:-margin_idx]))
        span = max(1e-5, v_max - v_min)
        ax_right.set_ylim(v_min - 0.15 * span, v_max + 0.15 * span)

        if idx == 0:
            ax_right.legend(loc="upper right", fontsize=8)

    axes[-1, 0].set_xlabel("Time t [s]")
    axes[-1, 1].set_xlabel("Time t [s]")

    fig.suptitle(title, fontsize=14, fontweight="bold", y=0.995)
    fig.tight_layout()
    fig.savefig(output_filename, dpi=150)
    plt.close(fig)
    print(f"Plot saved in: {output_filename}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", default="demo.csv", help="Noisy demo CSV (default: demo.csv)")
    parser.add_argument("--truth", default="truth.csv", help="Ground truth CSV (default: truth.csv)")
    parser.add_argument("--w1", type=float, default=0.05, help="Stage 1 window in seconds (default: 0.05)")
    parser.add_argument("--w2", type=float, default=0.05, help="Stage 2 window in seconds (default: 0.05)")
    parser.add_argument("--output-dir", default="plots", help="Output directory for plots")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Loading {args.demo} and {args.truth}...")
    t, pos, quat = load_demo(args.demo)
    t_truth, vel_true, acc_true, eta_true, eta_dot_true = load_truth(args.truth)
    assert np.allclose(t, t_truth), "Mismatch in time grids between demo and truth"

    tau = t[-1] - t[0]
    dt = t[1] - t[0]

    print(f"Computing raw vs filtered signals (w1={args.w1}s, w2={args.w2}s)...")
    vel_raw, acc_raw, eta_raw, eta_dot_raw = compute_raw_signals(t, pos, quat, tau)
    vel_filt, acc_filt, eta_filt, eta_dot_filt = compute_filtered_signals(t, pos, quat, tau, dt, args.w1, args.w2)

    print("Generating dual-scale comparative plots (Global View + Detailed Zoom)...")
    plot_dual_scale_comparison(t, vel_raw, vel_filt, vel_true,
                               f"Velocity Comparison v(t) [m/s] (w1={args.w1}s, w2={args.w2}s)",
                               "v [m/s]", os.path.join(args.output_dir, "filter_comp_velocity.png"))

    plot_dual_scale_comparison(t, acc_raw, acc_filt, acc_true,
                               f"Acceleration Comparison a(t) [m/s^2] (w1={args.w1}s, w2={args.w2}s)",
                               "a [m/s^2]", os.path.join(args.output_dir, "filter_comp_acceleration.png"))

    plot_dual_scale_comparison(t, eta_raw, eta_filt, eta_true,
                               f"Angular Velocity Comparison eta(t) [rad/s] (w1={args.w1}s, w2={args.w2}s)",
                               "eta [rad/s]", os.path.join(args.output_dir, "filter_comp_eta.png"))

    plot_dual_scale_comparison(t, eta_dot_raw, eta_dot_filt, eta_dot_true,
                               f"Angular Acceleration Comparison eta_dot(t) [rad/s^2] (w1={args.w1}s, w2={args.w2}s)",
                               "eta_dot [rad/s^2]", os.path.join(args.output_dir, "filter_comp_eta_dot.png"))

    # Dashboard 2x2: Detailed Zoom Filtered vs Ground Truth for all 4 norms
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    pairs = [
        (axes[0, 0], "Velocity |v(t)| [m/s]", np.linalg.norm(vel_filt, axis=1), np.linalg.norm(vel_true, axis=1)),
        (axes[0, 1], "Acceleration |a(t)| [m/s²]", np.linalg.norm(acc_filt, axis=1), np.linalg.norm(acc_true, axis=1)),
        (axes[1, 0], "Angular Velocity |η(t)| [rad/s]", np.linalg.norm(eta_filt, axis=1), np.linalg.norm(eta_true, axis=1)),
        (axes[1, 1], "Angular Acceleration |η_dot(t)| [rad/s²]", np.linalg.norm(eta_dot_filt, axis=1), np.linalg.norm(eta_dot_true, axis=1)),
    ]

    for ax, title, f_s, t_s in pairs:
        ax.plot(t, f_s, color="dodgerblue", linewidth=1.8, label="Filtered (2-stage moving average)")
        ax.plot(t, t_s, color="black", linestyle="--", linewidth=1.5, label="Ground Truth")
        ax.set_title(title, fontsize=12, fontweight="bold")
        ax.set_xlabel("Time t [s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        margin_idx = max(1, len(t) // 50)
        v_min = min(np.min(f_s[margin_idx:-margin_idx]), np.min(t_s[margin_idx:-margin_idx]))
        v_max = max(np.max(f_s[margin_idx:-margin_idx]), np.max(t_s[margin_idx:-margin_idx]))
        span = max(1e-5, v_max - v_min)
        ax.set_ylim(v_min - 0.15 * span, v_max + 0.15 * span)

    fig.suptitle(f"Detailed Zoom Dashboard: Filtered vs Ground Truth (w1={args.w1}s, w2={args.w2}s)", fontsize=15, fontweight="bold")
    fig.tight_layout()
    dash_path = os.path.join(args.output_dir, "filter_comp_dashboard.png")
    fig.savefig(dash_path, dpi=150)
    plt.close(fig)
    print(f"Zoom dashboard saved in: {dash_path}")


if __name__ == "__main__":
    main()

