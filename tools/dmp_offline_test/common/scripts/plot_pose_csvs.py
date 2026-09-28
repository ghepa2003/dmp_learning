#!/usr/bin/env python3
"""Overlays one or more t,x,y,z,qw,qx,qy,qz pose CSVs on the same 3D
trajectory + position/orientation time-series plot - the same CSV schema
used everywhere else in this project (demo_raw.csv, live_demo_raw.csv,
replay_from_yaml.csv, and record_pose_topic.py's output).

Also prints the same position-RMSE/max-error and mean/max angular-error
metrics as plot_real_demo.py, computed between the first two trajectories
given (the second resampled onto the first's own time base) - if more than
two are given, only that first pair gets metrics. Each input otherwise
keeps its own recording's timestamps (after any trimming/alignment
requested below), so trajectories are not assumed to already be index- or
time-aligned with each other.

Usage:
    python3 plot_pose_csvs.py [--trim-idle] <out_png_path> <csv1> <label1> [<csv2> <label2> ...]

--align <i>:<j> (repeatable): rigidly transforms the i-th CSV's position and
orientation so its first sample exactly matches the j-th CSV's first sample
- the same one-shot SE(3) offset that
franka_cartesian_control/ros_utils.hpp's FrameAligner applies once when the
Cartesian controller is activated (translation p_offset = p_j0 - p_i0, plus
full orientation composition q_offset = q_j0 * q_i0^-1, not just a position
shift). Use this whenever comparing a device-frame trajectory (e.g.
live_demo_raw.csv, in the Geomagic Touch's own workspace frame) against a
robot-frame one (actual_pose*.csv, in panda_link0) - without it, two
otherwise-matching trajectories differ by a constant rigid offset and one
looks "shifted" relative to the other even though the motion is correct.
Applied after any --duration/--window*/--trim-idle trimming, so the anchor
is each trajectory's first real-motion sample, not idle padding. Not needed
between two CSVs already in the same frame (e.g. two actual_pose*.csv, or a
demo_raw.csv/replay_from_yaml.csv pair).

--duration <i>:<seconds> (repeatable, PREFERRED for cross-node comparisons):
trims the i-th CSV (0-based, in command-line order) to [motion_start,
motion_start + seconds], where motion_start is auto-detected the same way
as --trim-idle (first sample moving >3mm from the initial position), and
`seconds` is a known-good duration you measured as a plain wall-clock
difference between two log lines FROM THE SAME NODE (e.g.
dmp_gazebo_executor_node's own "Starting DMP rollout" -> "DMP rollout
completed at goal", or live_demo_recorder_node's own "Recording started" ->
"Recording stopped"). This avoids ever comparing timestamps across two
different clocks: the recorded topic's own `epoch`/`t` only has to be
self-consistent to find motion_start, and `seconds` never leaves the one
node's own clock domain. Use this whenever the CSV was recorded from a topic
that might be on a different clock than the node whose log you're reading
(e.g. a Gazebo-driven controller's feedback topic runs on sim time - small
numbers starting near 0 in the epoch column - while most other nodes in
this project use wall-clock time; --window-epoch below silently produces an
empty plot if you get this wrong).

--window-epoch <i>:<epoch0>:<epoch1> (repeatable): trims/aligns the i-th CSV
to samples whose absolute `epoch` column (added by the current
record_pose_topic.py) falls in [epoch0, epoch1], re-anchoring t=0 to epoch0.
Only valid when the recorded topic and the log line you copied epoch0/epoch1
from share the same clock (check: does the CSV's epoch column look like a
~1.7-billion Unix timestamp, or small numbers starting near 0? the latter is
sim time - use --duration instead).

--window <i>:<t0>:<t1> / --trim-idle (fallback, for CSVs without an `epoch`
column, or when there's no independently known duration to anchor to): the
same idea but operating on the recording's own relative `t` (--window) or a
3mm-position-distance heuristic for both start AND end (--trim-idle).
--trim-idle's end-detection is the least reliable of the three: a smoothly-
converging DMP replay crosses back under the distance threshold earlier
than a human teleop demo with residual hand jitter, which can make an
otherwise equal-duration motion look shorter than it was.

Example (compare the robot's actual executed pose during the live teleop
demo against its actual executed pose while replaying the learned DMP, each
duration measured from its own producing node's log):
    python3 plot_pose_csvs.py plots/actual_pose_live_vs_replay.png \\
        actual_pose_live.csv "live (teleop)" \\
        actual_pose_replay.csv "replay (DMP)" \\
        --duration 0:27.040 --duration 1:27.040

Requires: matplotlib
"""
import csv
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

LINESTYLES = ['x', 'y', 'z']
AXIS_STYLES = {'x': '-', 'y': '--', 'z': ':'}
COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']


def _dist(traj, i, rx, ry, rz):
    return ((traj['x'][i] - rx) ** 2 + (traj['y'][i] - ry) ** 2 + (traj['z'][i] - rz) ** 2) ** 0.5


def _detect_start_idx(traj, threshold):
    x0, y0, z0 = traj['x'][0], traj['y'][0], traj['z'][0]
    return next((i for i in range(len(traj['t'])) if _dist(traj, i, x0, y0, z0) > threshold), 0)


def trim_idle(traj, threshold=0.003, tail_pad=1.0):
    n = len(traj['t'])
    xg, yg, zg = traj['x'][-1], traj['y'][-1], traj['z'][-1]

    start_idx = _detect_start_idx(traj, threshold)
    end_idx = next((i for i in range(n - 1, -1, -1) if _dist(traj, i, xg, yg, zg) > threshold), n - 1)

    t0 = traj['t'][start_idx]
    t_end_cutoff = traj['t'][end_idx] + tail_pad
    keep = [i for i in range(n) if t0 <= traj['t'][i] <= t_end_cutoff]

    out = {'has_quat': traj['has_quat']}
    for k in ['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']:
        out[k] = [(traj[k][i] - t0 if k == 't' else traj[k][i]) for i in keep]
    return out


def window_trim(traj, t0, t1, tail_pad=1.0):
    """Re-anchors t=0 to an explicit, caller-supplied t0 (e.g. read off the
    recording node's own start/stop log lines) instead of guessing it from a
    position-distance threshold - use this when trim_idle()'s heuristic picks
    inconsistent boundaries (e.g. a smoothly-converging DMP replay crosses
    back under the threshold earlier than a human teleop demo with residual
    hand jitter, making an otherwise equal-duration motion look shorter)."""
    n = len(traj['t'])
    t_end_cutoff = t1 + tail_pad
    keep = [i for i in range(n) if t0 <= traj['t'][i] <= t_end_cutoff]
    out = {'has_quat': traj['has_quat']}
    for k in ['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']:
        out[k] = [(traj[k][i] - t0 if k == 't' else traj[k][i]) for i in keep]
    return out


def duration_trim(traj, duration, threshold=0.003, tail_pad=1.0):
    """Detects motion start the same way trim_idle() does (first sample that
    moves more than `threshold` from the initial position), then keeps
    exactly `duration` seconds of motion from there (plus tail_pad).

    Use this instead of trim_idle()'s own end-detection, and instead of
    --window-epoch, whenever the recorded topic's timestamps are not on the
    same clock as the node whose log lines you'd use to bound the window
    (e.g. a Gazebo-driven controller's feedback topic uses sim time - small
    numbers starting near 0 - while most other nodes in this project use
    wall-clock time; check by eyeballing the CSV's epoch column). `duration`
    should instead come from a wall-clock difference between two log lines
    of the SAME node (e.g. dmp_gazebo_executor_node's own "Starting DMP
    rollout" -> "DMP rollout completed at goal", or live_demo_recorder_node's
    own "Recording started" -> "Recording stopped") - that's a same-clock-
    domain measurement even if the recorded topic itself is on a different
    clock, since it only depends on one node's own timestamps."""
    start_idx = _detect_start_idx(traj, threshold)
    t0 = traj['t'][start_idx]
    n = len(traj['t'])
    t_end_cutoff = t0 + duration + tail_pad
    keep = [i for i in range(n) if t0 <= traj['t'][i] <= t_end_cutoff]
    out = {'has_quat': traj['has_quat']}
    for k in ['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']:
        out[k] = [(traj[k][i] - t0 if k == 't' else traj[k][i]) for i in keep]
    return out


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def align_frames(traj_a, traj_b):
    """Rigidly transforms traj_a (translation + full orientation
    composition) so its first sample exactly matches traj_b's first sample
    - the same one-shot SE(3) anchoring that
    franka_cartesian_control/ros_utils.hpp's FrameAligner applies once at
    controller activation:
        p_offset = p_b0 - p_a0
        q_offset = q_b0 * q_a0^-1
        p_aligned,k = p_offset + p_a,k
        q_aligned,k = (q_offset * q_a,k).normalized()
    Use this before overlaying a device-frame trajectory (e.g.
    live_demo_raw.csv, in the Geomagic Touch's own workspace frame) against
    a robot-frame one (actual_pose*.csv, in panda_link0) - otherwise the two
    differ by exactly this constant rigid offset and the correct, matching
    motion looks like a "shifted" trajectory instead of an overlapping one.
    Apply this AFTER any --duration/--window*/--trim-idle trimming, so the
    anchor point is each trajectory's first sample of real motion, not
    whatever idle padding happened to be recorded first."""
    n = len(traj_a['t'])
    ax0, ay0, az0 = traj_a['x'][0], traj_a['y'][0], traj_a['z'][0]
    bx0, by0, bz0 = traj_b['x'][0], traj_b['y'][0], traj_b['z'][0]
    dx, dy, dz = bx0 - ax0, by0 - ay0, bz0 - az0

    out = {'has_quat': traj_a['has_quat'], 't': list(traj_a['t'])}
    out['x'] = [v + dx for v in traj_a['x']]
    out['y'] = [v + dy for v in traj_a['y']]
    out['z'] = [v + dz for v in traj_a['z']]

    if traj_a['has_quat'] and traj_b['has_quat']:
        qa0 = (traj_a['qw'][0], traj_a['qx'][0], traj_a['qy'][0], traj_a['qz'][0])
        qb0 = (traj_b['qw'][0], traj_b['qx'][0], traj_b['qy'][0], traj_b['qz'][0])
        q_offset = _quat_mul(qb0, _quat_conj(qa0))
        qw, qx, qy, qz = [], [], [], []
        for i in range(n):
            qa = (traj_a['qw'][i], traj_a['qx'][i], traj_a['qy'][i], traj_a['qz'][i])
            w, x, y, z = _quat_mul(q_offset, qa)
            norm = (w * w + x * x + y * y + z * z) ** 0.5
            qw.append(w / norm)
            qx.append(x / norm)
            qy.append(y / norm)
            qz.append(z / norm)
        out['qw'], out['qx'], out['qy'], out['qz'] = qw, qx, qy, qz
    else:
        out['qw'] = out['qx'] = out['qy'] = out['qz'] = []

    return out


def window_trim_epoch(traj, epoch0, epoch1, tail_pad=1.0):
    """Trims to samples with traj['epoch'] in [epoch0, epoch1 + tail_pad],
    re-anchoring t=0 to epoch0. WARNING: only valid if the recorded topic's
    `epoch` (msg.header.stamp) is on the same clock as whatever other node's
    log line you copied epoch0/epoch1 from - check the CSV's epoch column
    first: if it's small numbers starting near 0 rather than a ~1.7-billion
    Unix timestamp, that topic is on Gazebo sim time, not wall-clock time,
    and this function will silently produce an empty/nonsensical result. Use
    duration_trim() instead in that case."""
    if not traj.get('has_epoch'):
        raise ValueError("--window-epoch requires a CSV with an 'epoch' column "
                          '(re-record with the current record_pose_topic.py)')
    n = len(traj['epoch'])
    keep = [i for i in range(n) if epoch0 <= traj['epoch'][i] <= epoch1 + tail_pad]
    out = {'has_quat': traj['has_quat']}
    for k in ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']:
        out[k] = [traj[k][i] for i in keep]
    out['t'] = [traj['epoch'][i] - epoch0 for i in keep]
    return out


def load_csv(path):
    t, epoch, x, y, z = [], [], [], [], []
    qw, qx, qy, qz = [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        has_quat = 'qw' in fieldnames
        has_epoch = 'epoch' in fieldnames
        for row in reader:
            t.append(float(row['t']))
            if has_epoch:
                epoch.append(float(row['epoch']))
            x.append(float(row['x']))
            y.append(float(row['y']))
            z.append(float(row['z']))
            if has_quat:
                qw.append(float(row['qw']))
                qx.append(float(row['qx']))
                qy.append(float(row['qy']))
                qz.append(float(row['qz']))
    return {'t': t, 'epoch': epoch, 'x': x, 'y': y, 'z': z, 'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
            'has_quat': bool(qw), 'has_epoch': has_epoch}


def _slerp_arrays(t_src, qw, qx, qy, qz, t_query):
    """Interpolates quaternions onto a new time base with hemisphere
    continuity correction (nlerp + renormalization) to prevent sign flips -
    ported from plot_real_demo.py."""
    qw2, qx2, qy2, qz2 = list(qw), list(qx), list(qy), list(qz)
    for k in range(1, len(qw2)):
        dot = qw2[k] * qw2[k - 1] + qx2[k] * qx2[k - 1] + qy2[k] * qy2[k - 1] + qz2[k] * qz2[k - 1]
        if dot < 0:
            qw2[k], qx2[k], qy2[k], qz2[k] = -qw2[k], -qx2[k], -qy2[k], -qz2[k]

    out = [list(np.interp(t_query, t_src, comp)) for comp in (qw2, qx2, qy2, qz2)]
    qwq, qxq, qyq, qzq = out
    for k in range(len(qwq)):
        n = math.sqrt(qwq[k] ** 2 + qxq[k] ** 2 + qyq[k] ** 2 + qzq[k] ** 2)
        if n > 1e-9:
            qwq[k], qxq[k], qyq[k], qzq[k] = qwq[k] / n, qxq[k] / n, qyq[k] / n, qzq[k] / n
    return qwq, qxq, qyq, qzq


def _resample_to_common_time(a, b):
    """Resamples b onto a's time base so point-by-point comparison occurs at
    equal time instants rather than sample indices - ported from
    plot_real_demo.py."""
    t_common = a['t']
    rx = list(np.interp(t_common, b['t'], b['x']))
    ry = list(np.interp(t_common, b['t'], b['y']))
    rz = list(np.interp(t_common, b['t'], b['z']))
    has_quat = a['has_quat'] and b['has_quat']
    if has_quat:
        rqw, rqx, rqy, rqz = _slerp_arrays(b['t'], b['qw'], b['qx'], b['qy'], b['qz'], t_common)
    else:
        rqw = rqx = rqy = rqz = []
    return {'t': t_common, 'x': rx, 'y': ry, 'z': rz, 'qw': rqw, 'qx': rqx, 'qy': rqy, 'qz': rqz,
            'has_quat': has_quat}


def print_metrics(label_a, a, label_b, b):
    """Prints position RMSE/max-error and mean/max angular error between a
    and b (b resampled onto a's own time base first) - the same metrics
    plot_real_demo.py prints for the demo/replay pair, generalized to any
    two labeled trajectories."""
    if not a['x'] or not b['x']:
        return
    b_rs = _resample_to_common_time(a, b)
    n = len(a['t'])

    sq = [0.0, 0.0, 0.0]
    max_err = 0.0
    for k in range(n):
        dx = (b_rs['x'][k] - a['x'][k]) * 1000.0
        dy = (b_rs['y'][k] - a['y'][k]) * 1000.0
        dz = (b_rs['z'][k] - a['z'][k]) * 1000.0
        sq[0] += dx * dx
        sq[1] += dy * dy
        sq[2] += dz * dz
        max_err = max(max_err, math.sqrt(dx * dx + dy * dy + dz * dz))
    rmse = [math.sqrt(s / n) for s in sq]
    rmse_overall = math.sqrt(sum(r * r for r in rmse))
    print(f'  [{label_a} vs {label_b}] Position RMSE x/y/z: {rmse[0]:.4f} / {rmse[1]:.4f} / '
          f'{rmse[2]:.4f} mm | Total RMSE: {rmse_overall:.4f} mm | Max error: {max_err:.4f} mm')

    if a['has_quat'] and b_rs['has_quat']:
        sum_ang, max_ang = 0.0, 0.0
        idx_max_ang = 0
        for k in range(n):
            dot = abs(a['qw'][k] * b_rs['qw'][k] + a['qx'][k] * b_rs['qx'][k] +
                      a['qy'][k] * b_rs['qy'][k] + a['qz'][k] * b_rs['qz'][k])
            dot = max(-1.0, min(1.0, dot))
            angle_deg = 2.0 * math.acos(dot) * 180.0 / math.pi
            sum_ang += angle_deg
            if angle_deg > max_ang:
                max_ang = angle_deg
                idx_max_ang = k
        print(f'  [{label_a} vs {label_b}] Mean angular error: {sum_ang / n:.4f} deg | '
              f'max: {max_ang:.4f} deg at t={a["t"][idx_max_ang]:.3f}s (sample {idx_max_ang}/{n})')

    dgx, dgy, dgz = a['x'][-1], a['y'][-1], a['z'][-1]
    rgx, rgy, rgz = b['x'][-1], b['y'][-1], b['z'][-1]
    dx, dy, dz = (rgx - dgx) * 1000.0, (rgy - dgy) * 1000.0, (rgz - dgz) * 1000.0
    pos_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    print(f'  [{label_a} vs {label_b}] Final error - position: dx={dx:.4f} dy={dy:.4f} '
          f'dz={dz:.4f} mm | distance: {pos_dist:.4f} mm')
    if a['has_quat'] and b['has_quat']:
        dot = abs(a['qw'][-1] * b['qw'][-1] + a['qx'][-1] * b['qx'][-1] +
                  a['qy'][-1] * b['qy'][-1] + a['qz'][-1] * b['qz'][-1])
        dot = max(-1.0, min(1.0, dot))
        ang_dist = 2.0 * math.acos(dot) * 180.0 / math.pi
        print(f'  [{label_a} vs {label_b}] Final error - orientation: angular distance: {ang_dist:.4f} deg')


def main():
    args = sys.argv[1:]
    do_trim = '--trim-idle' in args
    if do_trim:
        args.remove('--trim-idle')

    windows = {}
    while '--window' in args:
        i = args.index('--window')
        idx_str, t0_str, t1_str = args[i + 1].split(':')
        windows[int(idx_str)] = (float(t0_str), float(t1_str))
        del args[i:i + 2]

    epoch_windows = {}
    while '--window-epoch' in args:
        i = args.index('--window-epoch')
        idx_str, e0_str, e1_str = args[i + 1].split(':')
        epoch_windows[int(idx_str)] = (float(e0_str), float(e1_str))
        del args[i:i + 2]

    durations = {}
    while '--duration' in args:
        i = args.index('--duration')
        idx_str, dur_str = args[i + 1].split(':')
        durations[int(idx_str)] = float(dur_str)
        del args[i:i + 2]

    aligns = {}
    while '--align' in args:
        i = args.index('--align')
        idx_a_str, idx_b_str = args[i + 1].split(':')
        aligns[int(idx_a_str)] = int(idx_b_str)
        del args[i:i + 2]

    if len(args) < 3 or len(args) % 2 != 1:
        print(__doc__)
        sys.exit(1)

    out_path = args[0]
    pairs = list(zip(args[1::2], args[2::2]))
    trajs = [(load_csv(path), label) for path, label in pairs]
    for idx, (traj, label) in enumerate(trajs):
        if idx in durations:
            trajs[idx] = (duration_trim(traj, durations[idx]), label)
        elif idx in epoch_windows:
            e0, e1 = epoch_windows[idx]
            trajs[idx] = (window_trim_epoch(traj, e0, e1), label)
        elif idx in windows:
            t0, t1 = windows[idx]
            trajs[idx] = (window_trim(traj, t0, t1), label)
        elif do_trim:
            trajs[idx] = (trim_idle(traj), label)

    for idx_a, idx_b in aligns.items():
        traj_a, label_a = trajs[idx_a]
        traj_b, _ = trajs[idx_b]
        trajs[idx_a] = (align_frames(traj_a, traj_b), label_a)

    if len(trajs) >= 2:
        (traj_a, label_a), (traj_b, label_b) = trajs[0], trajs[1]
        print_metrics(label_a, traj_a, label_b, traj_b)

    fig = plt.figure(figsize=(20, 6))

    ax3d = fig.add_subplot(1, 3, 1, projection='3d')
    for (traj, label), color in zip(trajs, COLORS):
        ax3d.plot(traj['x'], traj['y'], traj['z'], color=color, label=label, linewidth=2)
    ax3d.set_xlabel('x [m]')
    ax3d.set_ylabel('y [m]')
    ax3d.set_zlabel('z [m]')
    ax3d.set_title('3D Trajectory')
    ax3d.legend()

    ax_t = fig.add_subplot(1, 3, 2)
    for (traj, label), color in zip(trajs, COLORS):
        for axis in LINESTYLES:
            ax_t.plot(traj['t'], traj[axis], color=color, linestyle=AXIS_STYLES[axis],
                      label=f'{label} {axis}')
    ax_t.set_xlabel('t [s]')
    ax_t.set_ylabel('position [m]')
    ax_t.set_title('Position over time (solid=x, dashed=y, dotted=z)')
    ax_t.legend(fontsize='small')

    ax_q = fig.add_subplot(1, 3, 3)
    if all(traj['has_quat'] for traj, _ in trajs):
        for (traj, label), color in zip(trajs, COLORS):
            for comp, style in zip(['qw', 'qx', 'qy', 'qz'], ['-', '--', ':', '-.']):
                ax_q.plot(traj['t'], traj[comp], color=color, linestyle=style, alpha=0.8,
                          label=f'{label} {comp}')
        ax_q.set_xlabel('t [s]')
        ax_q.set_ylabel('quaternion components')
        ax_q.set_title('Orientation over time')
        ax_q.legend(fontsize='small')
    else:
        ax_q.text(0.5, 0.5, 'No orientation data in CSV', ha='center', va='center')
        ax_q.set_axis_off()

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f'Saved {out_path}')
    plt.show()


if __name__ == '__main__':
    main()
