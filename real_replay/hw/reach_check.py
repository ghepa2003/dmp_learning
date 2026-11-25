#!/usr/bin/env python3
"""reach_check.py - portata del robot 1 sul goal e sul follow (OFFLINE, nessun robot, nessun ROS).

Cinematica: stessa del planner (fer_flat_effort.urdf, frame fer_hand_tcp) letta direttamente dall'URDF (limiti inclusi).
IPOTESI (dichiarate):
  - frame = base robot 1 = world (robot_base_world = 0, come il planner);
  - dopo la presa la mano e' RIGIDA sul satellite: per una rotazione di fase d (rad, firmata, d = omega * t, negativa per
    omega < 0, antioraria positiva dall'alto) p(d) = c + Rz(d) (p_goal - c),  q(d) = Rz(d) * q_goal;
  - il follow dura omega*(T_close_max + T_follow + T_ramp) (da hw_config.yaml, sezione timing);
  - IK: damped least squares con pull verso il seed nello spazio nullo, clamp ai limiti URDF, seed = soluzione precedente.
NON modellati: carico del mockup (0.4 kg), collisioni (mano/mockup/robot 2/ambiente), dinamica, limiti di velocita'/coppia,
soft-limit del controllore, errore di tracking. Quindi NON e' una verifica completa della portata del robot reale.

Uso:  python3 hw/reach_check.py [--grasp-command F] [--config F] [--urdf F] [--step-deg 0.5] [--print-step-deg 2]
Exit 0 = IK ok su tutto il follow completo con margine >= --min-margin-deg e senza salti di ramo; 1 altrimenti.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(HERE, "..", "..", "fer_flat_effort.urdf")
Q_MEAS = np.array([1.570787, -0.786572, 0.000002, -2.378914, -0.000006, 3.141609, 0.785398])
JUMP_DEG = 5.0


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def rot_axis(a, t):
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def rot_z(a):
    return rot_axis([0, 0, 1], a)


def rot_log(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-12:
        return np.zeros(3)
    if math.pi - ang < 1e-6:
        w, V = np.linalg.eig(R)
        ax = np.real(V[:, np.argmin(np.abs(w - 1.0))])
        return ax / np.linalg.norm(ax) * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(ang)) * ang


def quat_wxyz_to_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat_wxyz(R):
    tr = np.trace(R)
    w = math.sqrt(max(0.0, 1 + tr)) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


class Arm:
    """Catena base -> fer_hand_tcp dall'URDF."""

    def __init__(self, urdf_path, tip="fer_hand_tcp"):
        root = ET.parse(urdf_path).getroot()
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip
        while link in by_child:
            j = by_child[link]; chain.append(j); link = j.find("parent").get("link")
        self.base_link = link
        self.steps, self.names, lo, hi = [], [], [], []
        for j in reversed(chain):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0.0] * 3
            movable = j.get("type") in ("revolute", "continuous")
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()]) if movable else None
            self.steps.append((xyz, rpy_matrix(*rpy), ax))
            if movable:
                self.names.append(j.get("name"))
                lim = j.find("limit")
                lo.append(float(lim.get("lower"))); hi.append(float(lim.get("upper")))
        self.lo, self.hi = np.array(lo), np.array(hi)
        if len(self.names) != 7:
            raise ValueError("catena con %d giunti mobili (attesi 7)" % len(self.names))

    def fk(self, q):
        """(R, p, [(z_i, o_i)] assi e origini dei giunti nel frame base)."""
        R, p, jac, i = np.eye(3), np.zeros(3), [], 0
        for xyz, Ro, ax in self.steps:
            p = p + R @ xyz; R = R @ Ro
            if ax is not None:
                jac.append((R @ ax / np.linalg.norm(ax), p.copy()))
                R = R @ rot_axis(ax, q[i]); i += 1
        return R, p, jac

    def jacobian(self, q):
        R, p, jac = self.fk(q)
        J = np.zeros((6, 7))
        for i, (z, o) in enumerate(jac):
            J[:3, i] = np.cross(z, p - o); J[3:, i] = z
        return J

    def ik(self, R_des, p_des, seed, iters=600, pos_tol=1e-5, ori_tol=1e-4, lam=0.05, k_null=1.0, max_step=0.2):
        q = np.clip(np.array(seed, float), self.lo, self.hi)
        for it in range(iters):
            kn = k_null if it < iters // 2 else 0.0   # seconda meta': solo convergenza, senza pull verso il seed
            R, p, _j = self.fk(q)
            e = np.concatenate([p_des - p, rot_log(R_des @ R.T)])
            if np.linalg.norm(e[:3]) < pos_tol and np.linalg.norm(e[3:]) < ori_tol:
                return q, True, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
            J = self.jacobian(q)
            Jp = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(6))
            dq = Jp @ e + (np.eye(7) - Jp @ J) @ (kn * 0.1 * (np.asarray(seed, float) - q))
            n = np.max(np.abs(dq))
            if n > max_step:
                dq *= max_step / n
            q = np.clip(q + dq, self.lo, self.hi)
        R, p, _j = self.fk(q)
        e = np.concatenate([p_des - p, rot_log(R_des @ R.T)])
        return q, False, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))

    def margins_deg(self, q):
        m = np.minimum(q - self.lo, self.hi - q)
        return np.degrees(m)


def follow_scan(arm, c, p_goal, R_goal, q_goal_sol, d_end_deg, step_deg=0.5):
    """IK continua da d=0 a d_end (firmato), passo step_deg. Ritorna lista di dict per ogni passo."""
    n = int(round(abs(d_end_deg) / step_deg))
    sgn = -1.0 if d_end_deg < 0 else 1.0
    out, q_prev = [], q_goal_sol
    for i in range(n + 1):
        d = sgn * i * step_deg
        Rz = rot_z(math.radians(d))
        p_d, R_d = c + Rz @ (p_goal - c), Rz @ R_goal
        q, ok, ep, eo = arm.ik(R_d, p_d, q_prev)
        m = arm.margins_deg(q)
        jump = float(np.degrees(np.max(np.abs(q - q_prev)))) if i > 0 else 0.0
        out.append({"d_deg": d, "q": q, "ok": ok, "err_pos_m": ep, "err_ori_rad": eo,
                    "margin_deg": float(m.min()), "limiting": arm.names[int(np.argmin(m))], "jump_deg": jump})
        q_prev = q
    return out


def first_below(rows, thr):
    for r in rows:
        if r["margin_deg"] < thr:
            return r["d_deg"]
    return None


def analyse(arm, cmd, cfg, step_deg=0.5, q_seed=Q_MEAS):
    """Ritorna dict con IK del goal, scansioni e verdetto. cfg: hw_config (scene, timing)."""
    c = np.array(cfg["scene"]["center_m"], float)
    w = float(cmd["omega_deg_s"])
    tm = cfg["timing"]
    d_close = w * float(tm["T_close_max_s"])
    d_full = w * (float(tm["T_close_max_s"]) + float(tm["T_follow_s"]) + float(tm["T_ramp_s"]))
    p_goal = np.array(cmd["goal_pose"]["position_m"], float)
    R_goal = quat_wxyz_to_R(cmd["goal_pose"]["quaternion_wxyz"])
    qg, ok, ep, eo = arm.ik(R_goal, p_goal, q_seed)
    res = {"goal_ik_ok": ok, "goal_q": qg, "goal_err": (ep, eo), "goal_margins_deg": arm.margins_deg(qg)}
    res["d_close_deg"], res["d_full_deg"] = d_close, d_full
    res["scan_full"] = follow_scan(arm, c, p_goal, R_goal, qg, d_full, step_deg) if ok else []
    res["scan_close"] = [r for r in res["scan_full"] if abs(r["d_deg"]) <= abs(d_close) + 1e-9]
    full = res["scan_full"]
    res["min_margin_close_deg"] = min([r["margin_deg"] for r in res["scan_close"]], default=None)
    res["min_margin_full_deg"] = min([r["margin_deg"] for r in full], default=None)
    res["max_jump_deg"] = max([r["jump_deg"] for r in full], default=0.0)
    res["all_ik_ok"] = ok and all(r["ok"] for r in full)
    res["branch_ok"] = res["max_jump_deg"] <= JUMP_DEG
    res["below10_full"], res["below5_full"] = first_below(full, 10.0), first_below(full, 5.0)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grasp-command", default=os.path.join(HERE, "examples", "grasp_command_hw_j1p90_w2_th0m10.yaml"))
    ap.add_argument("--config", default=os.path.join(HERE, "hw_config.yaml"))
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--step-deg", type=float, default=0.5)
    ap.add_argument("--print-step-deg", type=float, default=2.0)
    ap.add_argument("--min-margin-deg", type=float, default=5.0)
    a = ap.parse_args(argv)
    cmd = yaml.safe_load(open(a.grasp_command)); cfg = yaml.safe_load(open(a.config))
    arm = Arm(a.urdf)
    print("URDF %s, catena %s -> fer_hand_tcp" % (os.path.abspath(a.urdf), arm.base_link))
    print("limiti giunto [deg]: " + ", ".join("%s [%.2f, %.2f]" % (n[-6:], math.degrees(l), math.degrees(h)) for n, l, h in zip(arm.names, arm.lo, arm.hi)))
    r = analyse(arm, cmd, cfg, a.step_deg)
    print("IK goal: %s (err pos %.2e m, ori %.2e rad), q = %s" % ("OK" if r["goal_ik_ok"] else "FALLITA", *r["goal_err"], np.round(r["goal_q"], 4).tolist()))
    print("  margini goal [deg]: " + ", ".join("%.1f" % m for m in r["goal_margins_deg"]))
    if not r["goal_ik_ok"]:
        return 1
    print("follow: d da 0 a %.1f deg (solo chiusura %.1f deg), passo IK %.2f deg" % (r["d_full_deg"], r["d_close_deg"], a.step_deg))
    print("  %8s %10s %-12s %9s %s" % ("d [deg]", "margine", "giunto lim.", "salto", "ik"))
    k = max(1, int(round(a.print_step_deg / a.step_deg)))
    for i, row in enumerate(r["scan_full"]):
        if i % k == 0 or i == len(r["scan_full"]) - 1:
            print("  %8.1f %9.2f° %-12s %8.2f° %s" % (row["d_deg"], row["margin_deg"], row["limiting"][-6:], row["jump_deg"], "ok" if row["ok"] else "FAIL"))
    print("margine minimo: chiusura (0..%.0f) %.2f deg, follow completo (0..%.0f) %.2f deg" % (r["d_close_deg"], r["min_margin_close_deg"], r["d_full_deg"], r["min_margin_full_deg"]))
    print("margine < 10 deg a d = %s, < 5 deg a d = %s" % (r["below10_full"], r["below5_full"]))
    print("salto massimo tra passi: %.2f deg (limite %.1f) -> ramo %s" % (r["max_jump_deg"], JUMP_DEG, "coerente" if r["branch_ok"] else "SALTO"))
    ok = r["all_ik_ok"] and r["branch_ok"] and r["min_margin_full_deg"] >= a.min_margin_deg
    print("ESITO: %s (solo cinematica; carico mockup 0.4 kg e collisioni NON modellati)" % ("IK RAGGIUNGIBILE" if ok else "NON RAGGIUNGIBILE / MARGINE INSUFFICIENTE"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
