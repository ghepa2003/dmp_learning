#!/usr/bin/env python3
"""
tools/s1_kinematic_metrics.py - grandezze cinematiche continue di UN run S1 gia' registrato (nessun
processo viene lanciato), per il confronto k0/k2: la manipolabilita' w_full predice lo sforzo?

    python3 tools/s1_kinematic_metrics.py BAG_DIR [--executor-log executor.log] [--meta run_meta.json]
                 [--outdir DIR] [--csv aggregato.csv] [--predicted-w-full X] [--urdf fer_flat_effort.urdf]

Scrive DIR/kin_metrics.json (UNA riga JSON), DIR/kin_metrics.csv (header + riga) e, con --csv, aggiunge
la riga a un CSV aggregato (KIN_COLUMNS, ordine fisso). DIR default: <BAG_DIR>/.. (come s1_report).

Modello di w_full (stesso del planner, src/satellite_grasp_planner/core/manipulability.hpp):
  - URDF fer_flat_effort.urdf, frame fer_hand_tcp = fer_hand + (0, 0, 0.1034) m, rpy 0 (fer_hand_tcp_joint);
    e' il frame con cui selection_inputs.cpp costruisce il RobotModel del planner;
  - J = pinocchio getFrameJacobian(..., LOCAL_WORLD_ALIGNED), righe 0-2 lineari [m], 3-5 angolari [rad],
    colonne = i 7 giunti fer_joint1..7 (RobotModel::Jacobian6x7);
  - w_full = prodotto dei 6 valori singolari di J = sqrt(det(J J^T)) (fullManipulability().w_full).
  (w_full e' invariante alla rotazione del frame in cui e' espresso J: ruotare le due righe-blocco con la
  stessa R non cambia det(J J^T).)

RIFERIMENTO PER IL CONFRONTO COL PLANNER = AT_END (primo istante in cui il TCP raggiunge il goal), NON il
contatto: il contatto avviene ~1.7 s dopo, mentre il robot insegue la rotazione del cubo, quindi i giunti
(e w_full) sono gia' cambiati. w_full_contact resta nell'output come dato descrittivo, ma NON va usato per
confrontare con la previsione del planner (w_full_pred / --predicted-w-full): usare w_full_end e
w_full_end_rel_err. Campi a AT_END: w_full a AT_END e a +1, +2, +3 s (NaN se il bag finisce prima),
q_end_j1..7 [rad] misurati, errore di posa del TCP rispetto a GraspCommand.goal_pose (dal bag): posizione
|p_TCP - p_goal| [mm], orientazione = angolo della rotazione relativa R_TCP^T R_goal [deg] (come se3Distance).
goal_pose e' nel frame world = frame base del robot (il nodo impone robot_base_world = 0).

Definizioni (tempi = sim /clock, come s1_report):
  - istante del contatto (solo descrittivo, vedi sopra) = instants.first_contact_s di s1_report.analyze() (soglie di s1_report; se non
    rilevato analyze() ripiega su AT_END);
  - approccio = [TRIGGER, AT_END], follow = [AT_END, AT_END + follow_s] (follow_s dal meta, 3.0 se assente:
    come t_follow_end di s1_report);
  - velocita' di giunto = differenza centrata delle posizioni MISURATE (joint_states.position) con passo
    +-VEL_H_S ristretta alla fase (agli estremi e' unilaterale), campionata sulla griglia STEP_S: NON e' il campo velocity del messaggio (read_bag non lo
    legge). Il picco dipende da VEL_H_S;
  - rapporto picco/limite = max_t |qdot_j| / velocity del <limit> URDF del giunto j;
  - margine di giunto = min(q - lower, upper - q) per giunto, limiti del <limit> URDF (come jointMargins()
    in grasp_pose_eval.cpp), minimo su [TRIGGER, fine follow];
  - errore TCP-target = grasp.max_err_tgt_0_15s_mm di s1_report.analyze() (max di |TCP - /target_pose| in
    [AT_END, AT_END + 15 s]); non e' ricalcolato qui.
Dipendenze: numpy; pinocchio + rclpy solo in analyze()/main(): l'import del modulo non richiede ROS.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s1_report  # noqa: E402  (parse_executor_log / read_bag / analyze: nessuna logica duplicata)

N_JOINTS = 7
TCP_FRAME = "fer_hand_tcp"
STEP_S = 0.01        # passo della griglia di campionamento [s]
VEL_H_S = 0.05       # semi-ampiezza della differenza centrata per qdot [s]
PHASES = ("approach", "follow")

KIN_COLUMNS = (
    ["tag", "k", "theta_star_deg", "t_contact_rel_s",
     "w_full_contact", "w_full_pred", "w_full_contact_rel_err",
     "w_full_min", "w_full_min_t_rel_s", "w_full_follow_end",
     "w_full_end", "w_full_end_p1s", "w_full_end_p2s", "w_full_end_p3s", "w_full_end_rel_err"]
    + [f"q_end_j{j}" for j in range(1, N_JOINTS + 1)]
    + ["pose_err_end_pos_mm", "pose_err_end_ori_deg"]
    + [f"{name}_{ph}" for ph in PHASES for name in
       (["qdot_norm_peak", "qdot_norm_rms"]
        + [f"qdot_peak_j{j}" for j in range(1, N_JOINTS + 1)]
        + [f"qdot_ratio_j{j}" for j in range(1, N_JOINTS + 1)]
        + ["qdot_ratio_max", "qdot_ratio_worst_joint"])]
    + ["max_err_tgt_0_15s_mm",
       "margin_min_deg", "margin_min_joint", "margin_min_t_rel_s", "margin_contact_deg", "margin_pred_deg"]
)


# ------------------------------------------------------------------ funzioni pure

def full_manipulability(J) -> float:
    """w_full = prodotto dei valori singolari di J (6x7) = sqrt(det(J J^T)); 0 se J ha rango < 6."""
    J = np.asarray(J, dtype=float)
    if J.ndim != 2 or J.shape[0] != 6:
        raise ValueError("full_manipulability: J deve avere 6 righe")
    return float(max(0.0, np.prod(np.linalg.svd(J, compute_uv=False))))


def parse_urdf_limits(urdf, n=N_JOINTS):
    """(lower, upper, velocity) dei <limit> di fer_joint1..n, array [rad], [rad], [rad/s]."""
    root = ET.parse(str(urdf)).getroot()
    lims = {}
    for j in root.iter("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("name", "").startswith("fer_joint"):
            lims[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")), float(lim.get("velocity")))
    names = [f"fer_joint{i}" for i in range(1, n + 1)]
    missing = [nm for nm in names if nm not in lims]
    if missing:
        raise ValueError(f"limiti URDF mancanti per {missing}")
    arr = np.array([lims[nm] for nm in names])
    return arr[:, 0], arr[:, 1], arr[:, 2]


def window_grid(lo: float, hi: float, step: float = STEP_S) -> np.ndarray:
    """Griglia lo, lo+step, ... che contiene sempre hi come ultimo punto; vuota se hi < lo."""
    if not (hi >= lo):
        return np.empty(0)
    g = np.arange(lo, hi, step)
    return np.append(g, hi)


def interp_q(t, js_t, js_q) -> np.ndarray:
    """q (7) interpolata linearmente (clamp ai bordi) a un tempo o a un array di tempi -> (7,) o (len(t), 7)."""
    js_t, js_q = np.asarray(js_t), np.asarray(js_q)
    tt = np.atleast_1d(np.asarray(t, dtype=float))
    out = np.stack([np.interp(tt, js_t, js_q[:, c]) for c in range(js_q.shape[1])], axis=1)
    return out[0] if np.ndim(t) == 0 else out


def joint_velocity(ts, js_t, js_q, h: float = VEL_H_S, lo=None, hi=None) -> np.ndarray:
    """qdot(t) = (q(tb) - q(ta)) / (tb - ta) sulle posizioni misurate, ta = max(t-h, lo), tb = min(t+h, hi)
    (differenza centrata, ristretta a [lo, hi] se dati: cosi' il picco di una fase non assorbe l'altra)
    -> (len(ts), 7). Richiede lo < hi quando i limiti sono dati."""
    ts = np.asarray(ts, dtype=float)
    ta = ts - h if lo is None else np.maximum(ts - h, lo)
    tb = ts + h if hi is None else np.minimum(ts + h, hi)
    return (interp_q(tb, js_t, js_q) - interp_q(ta, js_t, js_q)) / (tb - ta)[:, None]


def velocity_stats(qdot, vel_limit) -> dict:
    """Norma 2 (picco, RMS) di qdot e, per giunto, picco |qdot_j| e rapporto picco/limite. qdot (N,7)."""
    nan = float("nan")
    out = {"norm_peak": nan, "norm_rms": nan, "peak": [nan] * N_JOINTS, "ratio": [nan] * N_JOINTS,
           "ratio_max": nan, "worst_joint": None}
    qdot = np.asarray(qdot, dtype=float)
    if qdot.ndim != 2 or len(qdot) == 0:
        return out
    norms = np.linalg.norm(qdot, axis=1)
    peak = np.max(np.abs(qdot), axis=0)
    ratio = peak / np.asarray(vel_limit, dtype=float)
    w = int(np.argmax(ratio))
    out.update(norm_peak=float(norms.max()), norm_rms=float(math.sqrt(np.mean(norms ** 2))),
               peak=[float(x) for x in peak], ratio=[float(x) for x in ratio],
               ratio_max=float(ratio[w]), worst_joint=w + 1)
    return out


def rotation_angle_deg(R1, R2) -> float:
    """Angolo [deg] della rotazione relativa R1^T R2 (atan2 di seno e coseno: stabile vicino a 0 e a pi)."""
    Rr = np.asarray(R1, dtype=float).T @ np.asarray(R2, dtype=float)
    c = 0.5 * (np.trace(Rr) - 1.0)
    s = 0.5 * float(np.linalg.norm([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]))
    return math.degrees(math.atan2(s, c))


def pose_error(p, R, goal_p, goal_R):
    """(errore di posizione [mm], errore di orientazione [deg]) fra la posa (p, R) e il goal."""
    return (float(np.linalg.norm((np.asarray(p, dtype=float) - np.asarray(goal_p, dtype=float)) * 1000.0)),
            rotation_angle_deg(R, goal_R))


def joint_margin(q, lower, upper) -> np.ndarray:
    """min(q - lower, upper - q) per giunto; q (7,) o (N,7)."""
    q = np.asarray(q, dtype=float)
    return np.minimum(q - lower, upper - q)


def rel_err(measured: float, predicted) -> float:
    """(misurato - previsto) / previsto; NaN se previsto assente, non finito o 0."""
    if predicted is None or not math.isfinite(float(predicted)) or float(predicted) == 0.0:
        return float("nan")
    return (float(measured) - float(predicted)) / float(predicted)


def compute_metrics(js_t, js_q, t_trig, t_end, t_contact, follow_s, jac_fn, lower, upper, vel_limit,
                    predicted_w_full=None, max_err_tgt_mm=None, margin_pred_deg=None,
                    step: float = STEP_S, vel_h: float = VEL_H_S, pose_fn=None, goal_p=None, goal_R=None) -> dict:
    """Tutte le grandezze (dict piatto, chiavi = KIN_COLUMNS tranne tag/k/theta_star_deg), funzione pura:
    jac_fn(q7) -> J 6x7 (in produzione JacobianKin.jacobian); pose_fn(q7) -> (p [m], R 3x3) del TCP
    (JacobianKin.pose) e goal_p/goal_R del goal: senza di essi l'errore di posa a AT_END e' NaN."""
    t_fend = t_end + follow_s
    nan = float("nan")

    q_c = interp_q(t_contact, js_t, js_q)
    w_contact = full_manipulability(jac_fn(q_c))

    # w_full e margine su [TRIGGER, fine follow]
    g_all = window_grid(t_trig, t_fend, step)
    q_all = interp_q(g_all, js_t, js_q)
    w_all = np.array([full_manipulability(jac_fn(q)) for q in q_all])
    i_min = int(np.argmin(w_all))
    mg_all = joint_margin(q_all, lower, upper)          # (N,7)
    mg_min_per_t = mg_all.min(axis=1)
    i_mg = int(np.argmin(mg_min_per_t))
    mg_c = joint_margin(q_c, lower, upper)

    # a AT_END e +1, +2, +3 s (NaN oltre la fine del bag: interp_q clamperebbe all'ultimo campione)
    t_bag_end = float(np.asarray(js_t)[-1])
    w_end = []
    for off in (0.0, 1.0, 2.0, 3.0):
        t = t_end + off
        w_end.append(full_manipulability(jac_fn(interp_q(t, js_t, js_q))) if t <= t_bag_end + 1e-9 else nan)
    q_end = interp_q(t_end, js_t, js_q)
    pos_err = ori_err = nan
    if pose_fn is not None and goal_p is not None and goal_R is not None:
        p_end, R_end = pose_fn(q_end)
        pos_err, ori_err = pose_error(p_end, R_end, goal_p, goal_R)

    out = {
        "w_full_end": w_end[0], "w_full_end_p1s": w_end[1], "w_full_end_p2s": w_end[2],
        "w_full_end_p3s": w_end[3], "w_full_end_rel_err": rel_err(w_end[0], predicted_w_full),
        "pose_err_end_pos_mm": pos_err, "pose_err_end_ori_deg": ori_err,
        "t_contact_rel_s": float(t_contact - t_end),
        "w_full_contact": w_contact,
        "w_full_pred": None if predicted_w_full is None else float(predicted_w_full),
        "w_full_contact_rel_err": rel_err(w_contact, predicted_w_full),
        "w_full_min": float(w_all[i_min]), "w_full_min_t_rel_s": float(g_all[i_min] - t_end),
        "w_full_follow_end": float(w_all[-1]),
        "max_err_tgt_0_15s_mm": nan if max_err_tgt_mm is None else float(max_err_tgt_mm),
        "margin_min_deg": math.degrees(float(mg_min_per_t[i_mg])),
        "margin_min_joint": int(np.argmin(mg_all[i_mg])) + 1,
        "margin_min_t_rel_s": float(g_all[i_mg] - t_end),
        "margin_contact_deg": math.degrees(float(mg_c.min())),
        "margin_pred_deg": nan if margin_pred_deg is None else float(margin_pred_deg),
    }

    for j in range(N_JOINTS):
        out[f"q_end_j{j + 1}"] = float(q_end[j])

    for ph, (lo, hi) in (("approach", (t_trig, t_end)), ("follow", (t_end, t_fend))):
        g = window_grid(lo, hi, step) if hi > lo else np.empty(0)
        st = velocity_stats(joint_velocity(g, js_t, js_q, vel_h, lo, hi) if len(g) else np.empty((0, N_JOINTS)),
                            vel_limit)
        out[f"qdot_norm_peak_{ph}"], out[f"qdot_norm_rms_{ph}"] = st["norm_peak"], st["norm_rms"]
        for j in range(N_JOINTS):
            out[f"qdot_peak_j{j + 1}_{ph}"] = st["peak"][j]
            out[f"qdot_ratio_j{j + 1}_{ph}"] = st["ratio"][j]
        out[f"qdot_ratio_max_{ph}"] = st["ratio_max"]
        out[f"qdot_ratio_worst_joint_{ph}"] = st["worst_joint"]
    return out


# ------------------------------------------------------------------ cinematica (pinocchio)

class JacobianKin:
    """Jacobiano 6x7 di fer_hand_tcp, LOCAL_WORLD_ALIGNED, colonne = fer_joint1..7 (come RobotModel)."""

    def __init__(self, urdf):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(TCP_FRAME)
        if self.frame_id >= self.model.nframes:
            raise ValueError(f"frame {TCP_FRAME} assente in {urdf}")
        jids = [self.model.getJointId(f"fer_joint{i}") for i in range(1, N_JOINTS + 1)]
        self.q_idx = [self.model.joints[j].idx_q for j in jids]
        self.v_idx = [self.model.joints[j].idx_v for j in jids]

    def _q_full(self, q7):
        q = np.zeros(self.model.nq)
        q[self.q_idx] = q7
        return q

    def pose(self, q7):
        """(posizione [m], rotazione 3x3) del TCP nel frame base."""
        pin = self.pin
        pin.forwardKinematics(self.model, self.data, self._q_full(q7))
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.frame_id]
        return M.translation.copy(), M.rotation.copy()

    def tcp_position(self, q7) -> np.ndarray:
        return self.pose(q7)[0]

    def jacobian(self, q7) -> np.ndarray:
        pin = self.pin
        q = self._q_full(q7)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return np.array(J)[:, self.v_idx]


# ------------------------------------------------------------------ analisi di un run

def analyze(bag_dir, executor_log, meta=None, urdf=None, predicted_w_full=None) -> dict:
    meta = dict(meta or {})
    urdf = urdf or s1_report.URDF_PATH
    ev = s1_report.parse_executor_log(executor_log)
    if ev["t_trigger"] is None or ev["t_at_end"] is None:
        raise s1_report.ReportError("executor log senza TRIGGER/AT_END")
    # contatto e errore TCP-target: direttamente da s1_report (il bag viene letto due volte)
    rep = s1_report.analyze(bag_dir, executor_log, meta)
    bag = s1_report.read_bag(bag_dir)
    lower, upper, vel = parse_urdf_limits(urdf)
    follow_s = float(meta["follow_s"]) if meta.get("follow_s") is not None else 3.0   # come s1_report
    gc = bag["grasp_cmd"]
    goal_p = goal_R = None
    if gc is not None:
        g = gc.goal_pose
        goal_p = np.array([g.position.x, g.position.y, g.position.z])
        goal_R = s1_report.quat_to_matrix([g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w])
    kin = JacobianKin(urdf)
    m = compute_metrics(
        bag["js_t"], bag["js_q"], ev["t_trigger"], ev["t_at_end"], rep["instants"]["first_contact_s"], follow_s,
        kin.jacobian, lower, upper, vel, predicted_w_full=predicted_w_full, pose_fn=kin.pose,
        goal_p=goal_p, goal_R=goal_R,
        max_err_tgt_mm=rep["grasp"]["max_err_tgt_0_15s_mm"],
        margin_pred_deg=rep["selection"]["min_joint_margin_deg"])
    row = {"tag": meta.get("tag"), "k": rep["selection"]["k"],
           "theta_star_deg": rep["selection"]["theta_star_deg"]}
    row.update(m)
    return {c: row.get(c) for c in KIN_COLUMNS}


# ------------------------------------------------------------------ uscite

def _clean(v):
    if isinstance(v, (np.floating, float)):
        v = float(v)
        return v if math.isfinite(v) else None
    if isinstance(v, np.integer):
        return int(v)
    return v


def json_line(row: dict) -> str:
    """UNA riga di JSON valido (i non finiti diventano null)."""
    return json.dumps({k: _clean(row.get(k)) for k in KIN_COLUMNS}, separators=(",", ":"))


def _cell(v) -> str:
    """Cella CSV con 8 cifre significative (w_full e' ~1e-2..1e-4: i 4 decimali di s1_report non bastano)."""
    v = _clean(v)
    if v is None:
        return ""
    return f"{v:.8g}" if isinstance(v, float) else str(v)


def csv_cells(row: dict) -> list:
    return [_cell(row.get(c)) for c in KIN_COLUMNS]


def write_outputs(row: dict, outdir, csv_path=None):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "kin_metrics.json").write_text(json_line(row) + "\n")
    with open(outdir / "kin_metrics.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(KIN_COLUMNS)
        w.writerow(csv_cells(row))
    if csv_path:
        path = Path(csv_path)
        new = not path.exists() or path.stat().st_size == 0
        with open(path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(KIN_COLUMNS)
            w.writerow(csv_cells(row))


def render_summary(row: dict) -> str:
    def n(v, f="{:.4f}"):
        v = _clean(v)
        return "n/d" if v is None else f.format(v)
    L = [f"S1 KIN tag={row.get('tag')} k={row.get('k')} theta*={n(row.get('theta_star_deg'), '{:.2f}')} deg",
         f"w_full a AT_END = {n(row['w_full_end'], '{:.6f}')} (+1/+2/+3 s: {n(row['w_full_end_p1s'], '{:.6f}')}/"
         f"{n(row['w_full_end_p2s'], '{:.6f}')}/{n(row['w_full_end_p3s'], '{:.6f}')})"
         + (f" | previsto {n(row['w_full_pred'], '{:.6f}')} | scarto rel a AT_END {n(row['w_full_end_rel_err'], '{:+.2%}')}"
            if row.get("w_full_pred") is not None else ""),
         f"w_full al contatto ({n(row.get('t_contact_rel_s'), '{:+.2f}')} s da AT_END) = {n(row['w_full_contact'], '{:.6f}')}"
         " (descrittivo: non confrontare col planner)",
         "q a AT_END [rad] = " + " ".join(n(row[f'q_end_j{j}'], '{:.4f}') for j in range(1, N_JOINTS + 1)),
         f"errore di posa a AT_END vs goal: {n(row['pose_err_end_pos_mm'], '{:.3f}')} mm, "
         f"{n(row['pose_err_end_ori_deg'], '{:.3f}')} deg",
         f"w_full min (approccio+follow) = {n(row['w_full_min'], '{:.6f}')} a {n(row['w_full_min_t_rel_s'], '{:+.2f}')} s | "
         f"a fine follow = {n(row['w_full_follow_end'], '{:.6f}')}"]
    for ph in PHASES:
        L.append(f"qdot {ph}: |qdot| picco {n(row[f'qdot_norm_peak_{ph}'], '{:.3f}')} rad/s, rms "
                 f"{n(row[f'qdot_norm_rms_{ph}'], '{:.3f}')} | picco/limite max {n(row[f'qdot_ratio_max_{ph}'], '{:.3f}')} "
                 f"(giunto {row.get(f'qdot_ratio_worst_joint_{ph}')})")
    L.append(f"max err TCP-target [0,+15] s = {n(row['max_err_tgt_0_15s_mm'], '{:.3f}')} mm")
    L.append(f"margine giunto min misurato = {n(row['margin_min_deg'], '{:.2f}')} deg (giunto {row['margin_min_joint']}, "
             f"{n(row['margin_min_t_rel_s'], '{:+.2f}')} s) | al contatto {n(row['margin_contact_deg'], '{:.2f}')} | "
             f"previsto dal planner {n(row['margin_pred_deg'], '{:.2f}')}")
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("bag_dir")
    ap.add_argument("--executor-log", help="default: <bag_dir>/../executor.log")
    ap.add_argument("--meta", help="default: <bag_dir>/../run_meta.json se esiste")
    ap.add_argument("--outdir", help="default: <bag_dir>/..")
    ap.add_argument("--csv", help="aggiunge la riga a questo CSV aggregato")
    ap.add_argument("--urdf", default=str(s1_report.URDF_PATH))
    ap.add_argument("--tag")
    ap.add_argument("--predicted-w-full", type=float, default=None,
                    help="w_full previsto dal planner: stampa e salva lo scarto relativo (misurato-previsto)/previsto")
    args = ap.parse_args(argv)

    bag_dir = Path(args.bag_dir).resolve()
    parent = bag_dir.parent
    elog = Path(args.executor_log) if args.executor_log else parent / "executor.log"
    meta_path = Path(args.meta) if args.meta else parent / "run_meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    if args.tag:
        meta["tag"] = args.tag
    meta.setdefault("tag", parent.name)
    try:
        if not elog.exists():
            raise s1_report.ReportError(f"executor log non trovato: {elog}")
        row = analyze(bag_dir, elog, meta, args.urdf, args.predicted_w_full)
        write_outputs(row, args.outdir or parent, args.csv)
    except (s1_report.ReportError, OSError, ImportError, ValueError) as e:
        print(f"s1_kinematic_metrics FAIL: {e}", file=sys.stderr)
        return 1
    print(render_summary(row), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
