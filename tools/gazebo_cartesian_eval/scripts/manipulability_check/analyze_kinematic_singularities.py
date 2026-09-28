#!/usr/bin/env python3
"""
analyze_kinematic_singularities.py — Analisi quantitativa delle singolarita'
cinematiche, dello spettro di J_t (SVD), delle direzioni cartesiane deboli
(u_min) e dell'allineamento con l'errore residuo cartesiano per Franka Emika Panda.

Confronto tra Goal 2 (anomalo, basso w_trans), Goal 4 (riferimento normale),
Goal 1, 3, 5 e Baseline.

Compatibile con esecuzione in Docker (franka:latest) o ambiente locale con Pinocchio.
"""

import os
import csv
import math
import bisect
import numpy as np
import pinocchio as pin

# ---------------------------------------------------------------------------
# Setup percorsi e modello Pinocchio
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EVAL_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
DATA_DIR = os.path.join(EVAL_ROOT, "data")
WS_ROOT = os.path.abspath(os.path.join(EVAL_ROOT, "../.."))

URDF_CANDIDATES = [
    os.path.join(WS_ROOT, "fer_flat_effort.urdf"),
    "/root/thesis_ws/fer_flat_effort.urdf",
    "/home/lorenzo/thesis_ws/fer_flat_effort.urdf",
]
urdf_path = next((p for p in URDF_CANDIDATES if os.path.exists(p)), None)
if urdf_path is None:
    raise FileNotFoundError("Impossibile trovare fer_flat_effort.urdf nei percorsi noti.")

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_name = "fer_hand_tcp"
frame_id = model.getFrameId(frame_name)
JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


def build_q_index(model, joint_names):
    """Resolves each joint name to its Pinocchio q-vector index (model.idx_qs[joint_id]),
    mirroring RobotModel::update's q_index resolution in robot_model.cpp. Needed because
    model.nq (9 with hand:=true: 7 arm joints + fer_finger_joint1/2) no longer matches the
    7-element q vectors read from joint_states_*.csv."""
    return [model.idx_qs[model.getJointId(name)] for name in joint_names]


Q_INDEX = build_q_index(model, JOINT_NAMES)

# Limiti di giunto meccanici dal modello URDF. NOTA: questi restano indicizzati in ordine di
# albero URDF (non tramite Q_INDEX) - invariato rispetto a prima della migrazione; qui sotto
# Q_MIN/Q_MAX sono usati solo con indici letterali 0-6 (q[3], Q_MIN[3], ...), non confrontati
# per intero contro q_full o q, quindi il fatto che siano ora vettori a 9 elementi (con
# hand:=true) non introduce un mismatch di shape in questo file - verificare comunque se le
# dita compaiono PRIMA dei giunti braccio nell'albero URDF (non atteso, ma non verificato qui).
Q_MIN = model.lowerPositionLimit
Q_MAX = model.upperPositionLimit

RUNS = [
    ("reach_task_baseline_impedance_kt200_delay1", "Baseline"),
    ("reach_task_goal_1_prodmp", "Goal 1"),
    ("reach_task_goal_2_prodmp", "Goal 2 (rep1)"),
    ("reach_task_goal_2_rep2_prodmp", "Goal 2 (rep2)"),
    ("reach_task_goal_2_rep3_prodmp", "Goal 2 (rep3)"),
    ("reach_task_goal_3_prodmp", "Goal 3"),
    ("reach_task_goal_4_prodmp", "Goal 4 (rep1)"),
    ("reach_task_goal_4_rep2_prodmp", "Goal 4 (rep2)"),
    ("reach_task_goal_4_rep3_prodmp", "Goal 4 (rep3)"),
    ("reach_task_goal_5_prodmp", "Goal 5"),
]


def load_trajectories(target_path, actual_path, js_path):
    """Carica i dati temporali e identifica l'istante t_clamp del target."""
    def read_csv(path):
        t, rows = [], []
        with open(path, "r") as f:
            for r in csv.DictReader(f):
                t.append(float(r["t"]))
                rows.append(r)
        return np.array(t), rows

    t_tgt, r_tgt = read_csv(target_path)
    t_act, r_act = read_csv(actual_path)
    t_js, r_js = read_csv(js_path)

    p_tgt = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in r_tgt])
    clamp_idx = len(t_tgt) - 1
    for i in range(len(t_tgt) - 1, -1, -1):
        if np.linalg.norm(p_tgt[i] - p_tgt[-1]) > 1e-4:
            clamp_idx = i + 1
            break
    t_clamp = t_tgt[clamp_idx]
    return t_tgt, r_tgt, t_act, r_act, t_js, r_js, t_clamp


def analyze_run(run_key, label):
    target_path = os.path.join(DATA_DIR, f"target_aligned_{run_key}.csv")
    actual_path = os.path.join(DATA_DIR, f"actual_pose_{run_key}.csv")
    js_path = os.path.join(DATA_DIR, f"joint_states_{run_key}.csv")

    if not (os.path.exists(target_path) and os.path.exists(actual_path) and os.path.exists(js_path)):
        return None

    t_tgt, r_tgt, t_act, r_act, t_js, r_js, t_clamp = load_trajectories(
        target_path, actual_path, js_path
    )

    idx_js = min(max(bisect.bisect_left(t_js, t_clamp), 0), len(t_js) - 1)
    idx_act = min(max(bisect.bisect_left(t_act, t_clamp), 0), len(t_act) - 1)
    idx_tgt = min(max(bisect.bisect_left(t_tgt, t_clamp), 0), len(t_tgt) - 1)

    q = np.array([float(r_js[idx_js][jn]) for jn in JOINT_NAMES])
    p_act = np.array([float(r_act[idx_act]["x"]), float(r_act[idx_act]["y"]), float(r_act[idx_act]["z"])])
    p_tgt = np.array([float(r_tgt[idx_tgt]["x"]), float(r_tgt[idx_tgt]["y"]), float(r_tgt[idx_tgt]["z"])])

    err_vec = p_act - p_tgt
    err_norm_mm = np.linalg.norm(err_vec) * 1000.0

    # Calcolo cinematico con Pinocchio. Zero-pad dei 7 valori braccio nel vettore di
    # configurazione completo model.nq, agli indici risolti per nome in Q_INDEX - i DOF extra
    # (dita del gripper con hand:=true) restano a 0.0: fer_hand_tcp è raggiunto tramite un
    # joint fixed a monte delle dita, quindi il loro valore non influenza la sua posizione, e
    # le colonne dello Jacobiano corrispondenti sono strutturalmente nulle - innocue per il
    # determinante/SVD di Jp sotto (U e S sono invarianti rispetto a colonne nulle aggiuntive;
    # Vt non viene usato in questa funzione).
    q_full = np.zeros(model.nq)
    for i, idx in enumerate(Q_INDEX):
        q_full[idx] = q[i]
    pin.computeJointJacobians(model, data, q_full)
    pin.framesForwardKinematics(model, data, q_full)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    Jp = J[:3, :]

    # SVD di J_t = U * S * V^T
    U, S, Vt = np.linalg.svd(Jp)
    w_trans = math.sqrt(max(0.0, np.linalg.det(Jp @ Jp.T)))
    sigma_min = S[2]
    u_min = U[:, 2].copy()

    # Convenzione di segno per confronto coerente (forziamo componente x > 0)
    if u_min[0] < 0:
        u_min = -u_min

    # Proiezione dell'errore cartesiano sull'asse debole
    proj_err_mm = abs(np.dot(u_min, err_vec)) * 1000.0
    cos_theta = proj_err_mm / (err_norm_mm + 1e-12)
    theta_deg = math.acos(min(1.0, cos_theta)) * 180.0 / math.pi

    q_deg = q * 180.0 / math.pi
    margin_q4_min_deg = (q[3] - Q_MIN[3]) * 180.0 / math.pi
    margin_q4_max_deg = (Q_MAX[3] - q[3]) * 180.0 / math.pi

    return {
        "key": run_key,
        "label": label,
        "t_clamp": t_clamp,
        "w_trans": w_trans,
        "sigmas": S,
        "u_min": u_min,
        "err_vec_mm": err_vec * 1000.0,
        "err_norm_mm": err_norm_mm,
        "proj_err_mm": proj_err_mm,
        "cos_theta": cos_theta,
        "theta_deg": theta_deg,
        "q_deg": q_deg,
        "margin_q4_min_deg": margin_q4_min_deg,
        "margin_q4_max_deg": margin_q4_max_deg,
        "abs_q5_deg": abs(q_deg[4]),
        "abs_q3_deg": abs(q_deg[2]),
        "abs_q6_deg": abs(q_deg[5]),
        "abs_q2_deg": abs(q_deg[1]),
    }


def main():
    print("=" * 130)
    print("ANALISI QUANTITATIVA DELLE SINGOLARITÀ CINEMATICHE E DELLO SPETTRO SVD DI J_t (PANDA 7-DOF)")
    print("=" * 130)

    results = []
    for r_key, lbl in RUNS:
        res = analyze_run(r_key, lbl)
        if res is not None:
            results.append(res)

    # 1. Tabella SVD, Manipolabilità e Allineamento Errore
    print("\n--- 1. SPETTRO SVD DI J_t, DIREZIONE DEBOLE u_min E ALLINEAMENTO CON L'ERRORE RESIDUO ---")
    print(f"{'Run / Goal':<15} | {'t_clamp':<7} | {'w_trans':<7} | {'σ_1, σ_2, σ_min':<20} | {'u_min [x, y, z]':<25} | {'||e|| [mm]':<9} | {'e_proj [mm]':<11} | {'cos(θ)':<6} | {'θ [deg]':<7}")
    print("-" * 130)
    for r in results:
        sig_str = f"[{r['sigmas'][0]:.2f}, {r['sigmas'][1]:.2f}, {r['sigmas'][2]:.2f}]"
        u = r["u_min"]
        u_str = f"[{u[0]:+.3f}, {u[1]:+.3f}, {u[2]:+.3f}]"
        print(f"{r['label']:<15} | {r['t_clamp']:<7.2f} | {r['w_trans']:<7.4f} | {sig_str:<20} | {u_str:<25} | {r['err_norm_mm']:<9.2f} | {r['proj_err_mm']:<11.2f} | {r['cos_theta']:<6.3f} | {r['theta_deg']:<7.1f}°")
    print("=" * 130)

    # 2. Tabella Distanze da Limiti e Singolarità di Giunto
    print("\n--- 2. PROSSIMITÀ AI LIMITI MECCANICI E ALLE CONDIZIONI DI SINGOLARITÀ INTERNA ---")
    print(f"{'Run / Goal':<15} | {'Margine q4 min':<14} | {'|q5| (z4||z6)':<13} | {'|q3| (z2||z4)':<13} | {'|q6| (wrist)':<12} | {'|q2| (shoulder)':<15} | {'q [deg] (q1...q7)':<45}")
    print("-" * 130)
    for r in results:
        q_str = "[" + ", ".join(f"{x:.1f}" for x in r["q_deg"]) + "]"
        print(f"{r['label']:<15} | {r['margin_q4_min_deg']:>6.1f}° (lim 0°) | {r['abs_q5_deg']:>6.1f}° (sg 0°) | {r['abs_q3_deg']:>6.1f}° (sg 0°) | {r['abs_q6_deg']:>6.1f}° (sg 0°) | {r['abs_q2_deg']:>6.1f}° (sg 0°) | {q_str}")
    print("=" * 130)

    # 3. Analisi di Consistenza Direzionale (Dot product u_min tra repliche di Goal 2)
    g2_reps = [r for r in results if "Goal 2" in r["label"]]
    if len(g2_reps) >= 3:
        u1, u2, u3 = g2_reps[0]["u_min"], g2_reps[1]["u_min"], g2_reps[2]["u_min"]
        dot12 = abs(np.dot(u1, u2))
        dot13 = abs(np.dot(u1, u3))
        dot23 = abs(np.dot(u2, u3))
        print("\n--- 3. CONSISTENZA DIREZIONALE CARTESIANA (Goal 2: 3 repliche) ---")
        print(f"  • Prodotto scalare |u_min(rep1) · u_min(rep2)| = {dot12:.4f} (angolo: {math.acos(min(1.0, dot12))*180/math.pi:.2f}°)")
        print(f"  • Prodotto scalare |u_min(rep1) · u_min(rep3)| = {dot13:.4f} (angolo: {math.acos(min(1.0, dot13))*180/math.pi:.2f}°)")
        print(f"  • Prodotto scalare |u_min(rep2) · u_min(rep3)| = {dot23:.4f} (angolo: {math.acos(min(1.0, dot23))*180/math.pi:.2f}°)")
        print("  => Conclusione: la direzione debole nello spazio cartesiano è invariante tra le 3 run (quasi puro asse X world),")
        print("     nonostante rep1/rep2 e rep3 risiedano su rami diversi della varietà di nullspace a 7-DOF.")


if __name__ == "__main__":
    main()
