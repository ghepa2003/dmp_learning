#!/usr/bin/env python3
"""
grasp_metrics.py - geometry of the TCP w.r.t. the satellite cube around the gripper trigger.

Usage:
  python3 tools/grasp_metrics.py <TS> [<TS> ...] [--root DIR] [--urdf FILE] [--out-dir DIR]
  python3 tools/grasp_metrics.py --all [--root DIR]

<TS> is the recording timestamp, either full ("20261014T150851") or the time part only
("150851", accepted if it identifies exactly one demo_raw_*T150851.csv). With --all every TS in
--root that has demo_raw_<TS>.csv + _joint_states.csv + _target_odom.csv is processed and a summary
table is printed.

Inputs (in --root, default: the workspace containing tools/), all on the same time base
(seconds since the start of the recording):
  demo_raw_<TS>.csv                 t,...,gripper_trigger
  demo_raw_<TS>_joint_states.csv    t,fer_joint1..7[,fer_finger_joint1,fer_finger_joint2]
  demo_raw_<TS>_target_odom.csv     t,x,y,z,qw,qx,qy,qz,... (cube pose, same world frame as the robot base)
  demo_raw_<TS>_finger_states.csv   optional fallback: t,fer_finger_joint1,fer_finger_joint2
Outputs: grasp_metrics_<TS>.csv (one row, header = metric names) in --out-dir (default --root);
with --all also grasp_metrics_all.csv (one row per TS).

Assumptions
  * Collar: inner radius 63.5 mm, outer radius 72.5 mm, nominal grasp radius 68.0 mm (mid-wall);
    collar edge at x = 155 mm in the cube frame (cube half-side 75 mm + standoff 80 mm).
  * Cube frame = the odometry pose (x = face normal, spin axis = world z). Odometry is in the
    world frame, which is the Pinocchio base frame of fer_flat_effort.urdf (as in
    analysis_s1_1013/run_full_analysis.py).
  * TCP = frame fer_hand_tcp, FK from fer_joint1..7 (the fingers sit downstream of it).
  * Duplicate t are dropped (first occurrence kept) in joint states and odometry.
  * Cube pose is interpolated at the joint-state times (linear position, SLERP orientation);
    metrics at an instant are read by the same interpolation of the TCP series.

Definitions (c = cube frame; p_c = R_cube^T (p_tcp - p_cube), R_c = R_cube^T R_tcp)
  t_trigger     first sample of demo_raw with gripper_trigger > 0.5. t1 = t_trigger + 1 s.
  r             sqrt(y_c^2 + z_c^2) [mm]; r_dev = r - 68.0; r_in_range = 63.5 <= r <= 72.5.
  x_c           axial coordinate [mm]; edge_dist = x_c - 155 (> 0: beyond the edge, away from the cube).
  theta_tcp     atan2(z_c, y_c), angle of the TCP around the collar axis.
  R_nom         planner's nominal pose at the TCP (CubeSatelliteModel::graspPoseAt):
                z_nom = -x_c, y_nom = -r_hat (centripetal), x_nom = y_nom x z_nom,
                r_hat = (0, cos theta_tcp, sin theta_tcp).
  R_rel         R_nom^T R_c, split in swing-twist about z:
                psi   = twist angle = 2 atan2(q_z, q_w) of R_rel, reported modulo pi in (-90, 90] deg
                        (psi and psi + pi are equivalent for the symmetric gripper);
                swing = angle between z_tcp and -x_c.
  closing_radial  INFORMATION ONLY: angle between the closing line (y of fer_hand_tcp) and the radial
                direction r_hat, as a line angle in [0, 90] deg (3D, no projection).
  cube yaw      rotation of the cube about world z, unwrapped; slope [deg/s] = least-squares line over
                [first sample, t_trigger] (pre) and [t1, end] (post) with RMS residual [deg];
                yaw0 = yaw at the first sample (initial phase); max displacement [mm] and max tilt of
                the cube z axis [deg] are measured from the first sample over the whole recording.
  persistence   between t1 and the end: max |p(t) - p(t1)| [mm] and max rotation angle of
                R(t1)^T R(t) [deg], for the TCP in the cube frame and in the world frame.
  finger width  fer_finger_joint1 + fer_finger_joint2 [m] at t_trigger, t1 and at the end of the
                recording; "dita non disponibili" if the finger columns / file are absent.

Only numpy, scipy and pinocchio (the latter imported lazily, only for the FK).
"""

import argparse
import csv
import math
import os
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

R_INNER_MM = 63.5
R_OUTER_MM = 72.5
R_NOMINAL_MM = 68.0
EDGE_X_MM = 155.0
POST_DELAY_S = 1.0
TCP_FRAME = "fer_hand_tcp"
ARM_JOINTS = [f"fer_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["fer_finger_joint1", "fer_finger_joint2"]
NO_FINGERS = "dita non disponibili"
DEFAULT_URDF = "/root/thesis_ws/fer_flat_effort.urdf"


# ----------------------------------------------------------------------------------------------
# I/O
# ----------------------------------------------------------------------------------------------
def default_root():
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def find_urdf(cli_urdf, root):
    candidates = [cli_urdf, os.environ.get("URDF_PATH"), DEFAULT_URDF,
                  os.path.join(root, "fer_flat_effort.urdf")]
    for p in candidates:
        if p and os.path.exists(p):
            return os.path.abspath(p)
    raise FileNotFoundError(f"fer_flat_effort.urdf not found, checked: {[c for c in candidates if c]}")


def resolve_ts(root, ts):
    """Full TS if demo_raw_<ts>.csv exists, else the unique demo_raw_*T<ts>.csv."""
    if os.path.exists(os.path.join(root, f"demo_raw_{ts}.csv")):
        return ts
    pat = re.compile(rf"^demo_raw_(\d{{8}}T{re.escape(ts)})\.csv$")
    hits = [m.group(1) for f in os.listdir(root) for m in [pat.match(f)] if m]
    if len(hits) != 1:
        raise FileNotFoundError(f"TS '{ts}': {len(hits)} matches for demo_raw_*T{ts}.csv in {root}")
    return hits[0]


def all_ts(root):
    pat = re.compile(r"^demo_raw_(\d{8}T\d{6})\.csv$")
    out = []
    for f in sorted(os.listdir(root)):
        m = pat.match(f)
        if m and all(os.path.exists(os.path.join(root, f"demo_raw_{m.group(1)}{s}.csv"))
                     for s in ("_joint_states", "_target_odom")):
            out.append(m.group(1))
    return out


def read_table(path):
    """Columns by name -> dict of 1-D float arrays (numpy only; 'nan' cells allowed)."""
    arr = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True, dtype=float))
    return {n: np.asarray(arr[n], dtype=float) for n in arr.dtype.names}


def dedupe(t, *arrays):
    """Sort by t and keep the first occurrence of each t."""
    t = np.asarray(t, dtype=float)
    order = np.argsort(t, kind="stable")
    t_s = t[order]
    _, first = np.unique(t_s, return_index=True)
    sel = order[first]
    return (t[sel],) + tuple(np.asarray(a)[sel] for a in arrays)


# ----------------------------------------------------------------------------------------------
# Kinematics
# ----------------------------------------------------------------------------------------------
def tcp_fk(urdf, q, fingers=None):
    """TCP position (N,3) and rotation (scipy Rotation, N) in the world/base frame. q: (N,7)."""
    import pinocchio as pin  # lazy: not needed by the pure-geometry functions

    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    fid = model.getFrameId(TCP_FRAME)
    if fid >= len(model.frames):
        raise ValueError(f"frame {TCP_FRAME} not in {urdf}")
    idx = [model.idx_qs[model.getJointId(n)] for n in ARM_JOINTS]
    f_idx = [model.idx_qs[model.getJointId(n)] if model.existJointName(n) else None
             for n in FINGER_JOINTS]
    pos = np.zeros((len(q), 3))
    mats = np.zeros((len(q), 3, 3))
    for i, qi in enumerate(q):
        q_full = np.zeros(model.nq)
        for j, k in enumerate(idx):
            q_full[k] = qi[j]
        if fingers is not None:
            for j, k in enumerate(f_idx):
                if k is not None and np.isfinite(fingers[i, j]):
                    q_full[k] = fingers[i, j]
        pin.forwardKinematics(model, data, q_full)
        pin.updateFramePlacements(model, data)
        pos[i] = data.oMf[fid].translation
        mats[i] = data.oMf[fid].rotation
    return pos, Rotation.from_matrix(mats)


def interp_pos(t, p, tq):
    return np.array([np.interp(tq, t, p[:, a]) for a in range(3)]).T


def interp_rot(t, rot, tq):
    """SLERP at tq, clamped to the range of t. Scalar tq -> single Rotation, array -> Rotation array."""
    scalar = np.ndim(tq) == 0
    out = Slerp(t, rot)(np.clip(np.atleast_1d(tq), t[0], t[-1]))
    return out[0] if scalar else out


def nominal_rotation(p_c):
    """R_nom (planner definition) at the TCP position p_c (cube frame): columns x,y,z."""
    th = math.atan2(p_c[2], p_c[1])
    r_hat = np.array([0.0, math.cos(th), math.sin(th)])
    z = np.array([-1.0, 0.0, 0.0])
    y = -r_hat
    x = np.cross(y, z)
    return np.column_stack([x, y, z]), r_hat, th


def wrap_pi_line(psi):
    """Angle modulo pi, in (-pi/2, pi/2]."""
    return (psi + math.pi / 2.0) % math.pi - math.pi / 2.0


def orientation_metrics(p_c, R_c):
    """p_c: TCP position in the cube frame [m]; R_c: 3x3 TCP rotation in the cube frame."""
    R_nom, r_hat, th = nominal_rotation(p_c)
    q = Rotation.from_matrix(R_nom.T @ R_c).as_quat()  # x,y,z,w
    psi = 2.0 * math.atan2(q[2], q[3])  # twist about z: angle of (w, 0, 0, qz)
    z_tcp = R_c[:, 2]
    swing = math.acos(float(np.clip(np.dot(z_tcp, [-1.0, 0.0, 0.0]), -1.0, 1.0)))
    y_tcp = R_c[:, 1]
    closing = math.acos(float(np.clip(abs(np.dot(y_tcp, r_hat)), 0.0, 1.0)))
    return {"theta_tcp_deg": math.degrees(th),
            "psi_deg": math.degrees(wrap_pi_line(psi)),
            "swing_deg": math.degrees(swing),
            "closing_radial_deg": math.degrees(closing)}


def position_metrics(p_c):
    y, z = p_c[1] * 1e3, p_c[2] * 1e3
    r = math.hypot(y, z)
    x = p_c[0] * 1e3
    return {"r_mm": r, "r_dev_mm": r - R_NOMINAL_MM,
            "r_in_range": bool(R_INNER_MM <= r <= R_OUTER_MM),
            "x_c_mm": x, "edge_dist_mm": x - EDGE_X_MM}


def line_fit(t, y):
    """Slope and RMS residual of a least-squares line; NaN if < 3 samples."""
    if len(t) < 3:
        return float("nan"), float("nan")
    a, b = np.polyfit(t, y, 1)
    return float(a), float(np.sqrt(np.mean((y - (a * t + b)) ** 2)))


def cube_yaw_deg(rot):
    m = rot.as_matrix()
    return np.degrees(np.unwrap(np.arctan2(m[:, 1, 0], m[:, 0, 0])))


# ----------------------------------------------------------------------------------------------
# Per-TS analysis
# ----------------------------------------------------------------------------------------------
def load_fingers(root, ts, js):
    """(t, (N,2)) or None. From the joint-states CSV columns, else the _finger_states.csv fallback."""
    if all(n in js for n in FINGER_JOINTS):
        return js["t"], np.column_stack([js[n] for n in FINGER_JOINTS])
    path = os.path.join(root, f"demo_raw_{ts}_finger_states.csv")
    if os.path.exists(path):
        tab = read_table(path)
        if all(n in tab for n in ["t"] + FINGER_JOINTS):
            return tab["t"], np.column_stack([tab[n] for n in FINGER_JOINTS])
    return None


def width_at(t, f, tq):
    ok = np.isfinite(f).all(axis=1)
    if not ok.any():
        return float("nan")
    t, w = dedupe(t[ok], f[ok].sum(axis=1))
    if tq < t[0] - 1e-9 or tq > t[-1] + 1e-9:
        return float("nan")
    return float(np.interp(tq, t, w))


def analyze(root, ts, urdf):
    p = lambda suffix: os.path.join(root, f"demo_raw_{ts}{suffix}.csv")
    for suffix in ("", "_joint_states", "_target_odom"):
        if not os.path.exists(p(suffix)):
            raise FileNotFoundError(p(suffix))

    demo = read_table(p(""))
    trig = np.nonzero(demo["gripper_trigger"] > 0.5)[0]
    if len(trig) == 0:
        raise ValueError(f"{p('')}: no sample with gripper_trigger > 0.5")
    t_trig = float(demo["t"][trig[0]])
    t_post = t_trig + POST_DELAY_S

    js = read_table(p("_joint_states"))
    fing = load_fingers(root, ts, js)
    n_js = len(js["t"])
    t_js, q_js = dedupe(js["t"], np.column_stack([js[n] for n in ARM_JOINTS]))
    f_js = None
    if fing is not None and len(fing[0]) == n_js:  # same rows as the joint states: dedupe consistently
        _, f_js = dedupe(js["t"], fing[1])
    p_w, R_w = tcp_fk(urdf, q_js, f_js)

    od = read_table(p("_target_odom"))
    t_od, od_p, od_q = dedupe(od["t"], np.column_stack([od["x"], od["y"], od["z"]]),
                              np.column_stack([od["qx"], od["qy"], od["qz"], od["qw"]]))
    R_od = Rotation.from_quat(od_q)
    if len(t_od) < 2:
        raise ValueError("target odometry has fewer than 2 distinct samples")

    # cube pose at the joint-state times, TCP in the cube frame
    t_eval = np.clip(t_js, t_od[0], t_od[-1])
    cube_p = interp_pos(t_od, od_p, t_eval)
    cube_R = interp_rot(t_od, R_od, t_eval)
    p_c = cube_R.inv().apply(p_w - cube_p)
    R_c = cube_R.inv() * R_w

    res = {"ts": ts, "t_trigger_s": t_trig, "t_end_s": float(t_js[-1])}

    # --- position / orientation at t_trigger and t_trigger + 1 s
    for tag, tq in (("trig", t_trig), ("trig1", t_post)):
        if tq > t_js[-1] or tq < t_js[0]:
            res[f"{tag}_note"] = "outside joint-state range"
            continue
        pq = interp_pos(t_js, p_c, tq)
        Rq = interp_rot(t_js, R_c, tq).as_matrix()
        for k, v in {**position_metrics(pq), **orientation_metrics(pq, Rq)}.items():
            res[f"{tag}_{k}"] = v

    # --- cube yaw, displacement, tilt
    yaw = cube_yaw_deg(R_od)
    pre = (t_od >= t_od[0]) & (t_od <= t_trig)
    post = (t_od >= t_post) & (t_od <= t_od[-1])
    res["cube_yaw_slope_pre_deg_s"], res["cube_yaw_rms_pre_deg"] = line_fit(t_od[pre], yaw[pre])
    res["cube_yaw_slope_post_deg_s"], res["cube_yaw_rms_post_deg"] = line_fit(t_od[post], yaw[post])
    res["cube_yaw0_deg"] = float(yaw[0])
    res["cube_max_disp_mm"] = float(np.max(np.linalg.norm(od_p - od_p[0], axis=1)) * 1e3)
    z_axis = R_od.as_matrix()[:, :, 2]
    res["cube_max_tilt_deg"] = float(np.degrees(np.arccos(np.clip(z_axis @ z_axis[0], -1.0, 1.0))).max())

    # --- TCP persistence after t_trigger + 1 s
    sel = np.nonzero(t_js >= t_post)[0]
    if t_post <= t_js[-1] and len(sel) > 0:
        pc1 = interp_pos(t_js, p_c, t_post)
        Rc1 = interp_rot(t_js, R_c, t_post)
        pw1 = interp_pos(t_js, p_w, t_post)
        Rw1 = interp_rot(t_js, R_w, t_post)
        res["tcp_cube_max_disp_mm"] = float(np.max(np.linalg.norm(p_c[sel] - pc1, axis=1)) * 1e3)
        res["tcp_cube_max_rot_deg"] = float(np.degrees((Rc1.inv() * R_c[sel]).magnitude()).max())
        res["tcp_world_max_disp_mm"] = float(np.max(np.linalg.norm(p_w[sel] - pw1, axis=1)) * 1e3)
        res["tcp_world_max_rot_deg"] = float(np.degrees((Rw1.inv() * R_w[sel]).magnitude()).max())

    # --- finger width
    if fing is None:
        res["finger_width"] = NO_FINGERS
    else:
        tf, ff = fing
        for tag, tq in (("trig", t_trig), ("trig1", t_post), ("end", float(tf.max()))):
            res[f"finger_width_{tag}_m"] = width_at(tf, ff, tq)
    return res


# ----------------------------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------------------------
def fmt(v):
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


def print_one(res):
    print(f"=== {res['ts']}  (t_trigger = {res['t_trigger_s']:.3f} s, end = {res['t_end_s']:.3f} s) ===")
    for k, v in res.items():
        if k not in ("ts", "t_trigger_s", "t_end_s"):
            print(f"  {k:<28s} {fmt(v)}")


SUMMARY_COLS = ["ts", "t_trigger_s", "trig_r_mm", "trig_r_in_range", "trig_x_c_mm", "trig_psi_deg",
                "trig_swing_deg", "trig1_r_mm", "trig1_psi_deg", "cube_yaw_slope_pre_deg_s",
                "cube_yaw_slope_post_deg_s", "cube_max_disp_mm", "tcp_cube_max_disp_mm",
                "tcp_cube_max_rot_deg", "finger_width_trig1_m"]


def print_summary(rows):
    cols = SUMMARY_COLS
    w = [max(len(c), 10) for c in cols]
    print(" ".join(c.rjust(n) for c, n in zip(cols, w)))
    for r in rows:
        cells = [fmt(r.get(c, r.get("finger_width", "")) if c == "finger_width_trig1_m" else r.get(c, ""))
                 for c in cols]
        print(" ".join(c.rjust(n) for c, n in zip(cells, w)))


def write_csv(path, rows):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=keys, restval="")
        wr.writeheader()
        wr.writerows(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Grasp geometry metrics around the gripper trigger.")
    ap.add_argument("ts", nargs="*", help="recording timestamp(s), full or time part only")
    ap.add_argument("--all", action="store_true", help="process every complete TS in --root")
    ap.add_argument("--root", default=default_root())
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--out-dir", default=None, help="default: --root")
    args = ap.parse_args(argv)

    root = os.path.abspath(args.root)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else root
    os.makedirs(out_dir, exist_ok=True)
    if args.all:
        tss = all_ts(root)
    else:
        tss = [resolve_ts(root, t) for t in args.ts]
    if not tss:
        ap.error("no TS given (or none found with --all)")
    urdf = find_urdf(args.urdf, root)

    rows, failed = [], 0
    for ts in tss:
        try:
            res = analyze(root, ts, urdf)
        except (FileNotFoundError, ValueError) as e:
            failed += 1
            print(f"[{ts}] SKIPPED: {e}", file=sys.stderr)
            continue
        rows.append(res)
        write_csv(os.path.join(out_dir, f"grasp_metrics_{ts}.csv"), [res])
        if not (args.all or len(tss) > 1):
            print_one(res)
    if len(rows) > 1 or args.all:
        print_summary(rows)
        write_csv(os.path.join(out_dir, "grasp_metrics_all.csv"), rows)
    return 1 if failed and not rows else 0


if __name__ == "__main__":
    sys.exit(main())
