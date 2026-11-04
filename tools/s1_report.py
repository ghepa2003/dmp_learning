#!/usr/bin/env python3
"""
tools/s1_report.py - analisi di UN bag S1 gia' registrato (nessun processo viene lanciato).

Estratto e parametrizzato da analysis_s1_1015/analyze_final2.py (che ha prodotto report_final2.txt).
Importabile (analyze / render_txt / csv_row / append_csv) e da CLI:

    python3 tools/s1_report.py BAG_DIR --executor-log executor.log [--meta run_meta.json]
                               [--outdir DIR] [--csv results.csv] [--criteria tools/s1_success.yaml]

Produce DIR/report.json e DIR/report.txt (max 60 righe, sezioni sempre uguali: selezione, lancio,
istanti, criteri di presa) e, con --csv, aggiunge una riga a colonne fisse (CSV_COLUMNS).

Dipendenze: numpy; rclpy + rosidl_runtime_py (lettura bag) e pinocchio (cinematica) solo quando
si chiama analyze(): l'import del modulo non richiede ROS.

Convenzioni delle colonne (unita' tra parentesi):
  theta_star (deg), d_pos (mm), wait_s = provisional_delay_s del GraspCommand (s, STIMA del planner,
  significato invariato), wait_s_actual = TRIGGER - pubblicazione del GraspCommand (s sim, misurato:
  launch.wait_actual_s del JSON),
  stop_cube (0/1, run con --stop-cube), wz_after_stop_degps (deg/s, wz del cubo 1 s sim dopo lo Twist nullo;
  vuoto se stop_cube=0 o non misurato),
  tcp_cube_var_{close,tang,axial}_3_8s (mm): come tcp_cube_var_3_8s ma per componente, max |proiezione di
  TCP_cubo(t) - TCP_cubo(+3 s)| nella finestra [+3, +8] s. Assi nel frame del cubo, fissati dal TCP a +3 s:
  axial = +x del cubo (asse del collare, lo stesso di face_normal_body); close = direzione RADIALE rispetto
  a quell'asse, cioe' quella lungo cui si calcolano i raggi L/R dei pad (hypot(y, z) nel frame del cubo),
  (0, y, z)/|(0, y, z)| del TCP; tang = axial x close (tangenziale alla parete). Il gioco dei pad e' sul
  close, lo slittamento sul tang/axial,
  f1_f2_plateau_min_10_20s (mm) = min(media f1, media f2) in [+10, +20] s,
  wz_cube_mean_4_8s (deg/s) = pendenza del fit lineare dello yaw (= velocita' media, stessa convenzione
  di segno di omega) in [t_follow_end + 4, t_follow_end + 8] s, t_follow_end = AT_END + follow_s del meta
  (3.0 se assente): il log "follow: ended" non ha timestamp, e il follow termina al primo tick da 10 Hz
  dopo AT_END + follow_s (differenza <= 0.1 s),
  slip_tang_axial_3_15s = max(tang_3_15, axial_3_15) (mm, criterio); f_sum_plateau_10_20s = media di f1+f2 in
  [+10, +20] s (mm; NaN se il bag finisce prima di +20 s); pad_straddle_5s / _15s (0/1); i criteri sono in
  tools/s1_success.yaml,
  fondo finestra (rel AT_END, ultimo campione utile del bag): window_end_rel_s, gap_end_mean_last10s (media
  f1+f2 negli ultimi 10 s, mm), pad_straddle_end (0/1), slip_tang_axial_3_end (max tang/axial di
  |TCP-cubo(t) - TCP-cubo(+3 s)|, t in [+3 s, fine], mm); NaN se window_end_rel_s < min_window_after_at_end_s
  (yaml, --min-window-s),
  min_joint_margin_deg / _joint / _t_rel_s: margine di giunto minimo misurato (deg), min sui 7 giunti di
  min(q - inf, sup - q) con limiti del URDF, nella finestra [TRIGGER, AT_END + follow_s]; giunto (1..7) e
  istante rel AT_END (s). NaN se il bag finisce prima di AT_END + follow_s. Criterio 11 (19/10/2026),
  success = AND dei criteri 1-10 (esito principale, 02/11/2026); criterion_11_ok = criterio 11 (SEPARATE_CRITERIA,
  riportato a parte); success_1_11 = AND di tutti e 11 (l'esito unico delle analisi precedenti); le soglie
  restano quelle di tools/s1_success.yaml,
  start_goal_deg (deg) = angolo start-goal del ramo scelto dalla riga "selected" del planner.log
  (start_goal_deg[A|B] secondo wrist_flip; vuoto se il log o i campi mancano); sopra ROT_VERIFIED_MAX_DEG
  il report.txt stampa un AVVISO (il successo non cambia),
  timeseries.csv (scritto da write_outputs): passo 0.5 s da AT_END a fine bag.
  yaw_trig_err / yaw_end_err (deg, misurato - atteso), f_sum_* / min_f_5s (mm),
  tcp_cube_var_3_X (mm): max |TCP_cubo(t) - TCP_cubo(+3 s)| per t in [+3, +X] s,
  yaw_slope_6_12s (deg/s), max_err_tgt_0_15s / cube_disp_max (mm), tilt_max (deg),
  sin_q6_min = min sin(fer_joint6) sui joint_states in [TRIGGER, AT_END + 15 s],
  pad_straddle_2s (0/1), success (0/1).
Tutti i tempi sono tempo simulato (/clock, header.stamp), t=0 dei "+N s" e' AT_END.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sqlite3
import sys
from pathlib import Path

import numpy as np

WS_DIR = Path(os.environ.get("THESIS_WS", Path(__file__).resolve().parents[1]))
URDF_PATH = WS_DIR / "fer_flat_effort.urdf"
SUCCESS_YAML = Path(__file__).with_name("s1_success.yaml")

JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]

# Parete del collare nel frame del cubo (raggio interno / esterno, mm)
COLLAR_R_INNER_MM = 63.5
COLLAR_R_OUTER_MM = 72.5
PAD_HALF_Y_FALLBACK_M = 0.0076

# Soglie del "primo contatto" (come analyze_final2.py)
CONTACT_SLOPE_TOL_DEG_S = 0.05
CONTACT_DISP_MM = 0.3
CONTACT_TILT_DEG = 0.05

MAX_TXT_LINES = 60

# Dominio di rotazione start-goal verificato (deg): oltre, solo un AVVISO nel report
ROT_VERIFIED_MAX_DEG = 58.1
ROT_WARNING = f"rotazione start-goal oltre il dominio verificato ({ROT_VERIFIED_MAX_DEG:g} gradi)"
MARGIN_STEP_S = 0.01   # passo della griglia del margine (come s1_kinematic_metrics.STEP_S)

# Attesa pubblicazione GraspCommand -> TRIGGER: min_delay_s del runner (5 s) e al piu' un giro del cubo
WAIT_MIN_S = 5.0
WAIT_FULL_TURN_DEG = 360.0
WAIT_WARNING = "attesa fuori intervallo: lo yaw di spawn non è coerente con omega"
OMEGA_META_TOL_DEG_S = 0.1   # |omega del meta - omega del GraspCommand| oltre questo => avviso

CSV_COLUMNS = [
    "tag", "depth", "kt", "follow_s", "hold_s", "squeeze", "spawn_yaw",
    "k", "theta_star", "d_pos", "cost", "wait_s", "yaw_trig_err", "yaw_end_err",
    "pad_straddle_2s", "f_sum_2s", "f_sum_5s", "f_sum_15s", "min_f_5s",
    "tcp_cube_var_3_8s", "tcp_cube_var_3_15s", "yaw_slope_6_12s", "max_err_tgt_0_15s",
    "cube_disp_max", "tilt_max", "sin_q6_min", "success",
    # in fondo: le colonne precedenti (e i golden) non cambiano
    "stop_cube", "wz_after_stop_degps", "wait_s_actual",
    "tcp_cube_var_close_3_8s", "tcp_cube_var_tang_3_8s", "tcp_cube_var_axial_3_8s",
    "f1_f2_plateau_min_10_20s", "wz_cube_mean_4_8s",
    "tcp_cube_var_tang_3_15s", "tcp_cube_var_axial_3_15s", "pad_straddle_5s", "pad_straddle_15s",
    "f_sum_plateau_10_20s", "tcp_cube_var_close_3_15s",
    # criteri sul fondo della finestra registrata (rel AT_END)
    "window_end_rel_s", "gap_end_mean_last10s", "pad_straddle_end", "slip_tang_axial_3_end",
    # campagna omega / centro / braccio (s1_run.py --omega-deg-s / --center-xyz / --arm)
    "omega_deg_s", "center_xyz", "arm",
    # criterio 11 (19/10/2026) e rotazione start-goal del ramo scelto
    "min_joint_margin_deg", "min_joint_margin_joint", "min_joint_margin_t_rel_s", "start_goal_deg",
    # 02/11/2026: "success" = criteri 1-10; criterio 11 a parte; success_1_11 = tutti e 11 (esito unico precedente)
    "criterion_11_ok", "success_1_11",
]

TIMESERIES_COLUMNS = ["t_rel", "f1_mm", "f2_mm", "yaw_deg", "yaw_rate_degps",
                      "d_close_mm", "d_tang_mm", "d_axial_mm"]
TIMESERIES_STEP_S = 0.5

# Parametri (non criteri) di s1_success.yaml
MIN_WINDOW_KEY = "min_window_after_at_end_s"
MIN_WINDOW_DEFAULT_S = 40.0
PARAM_KEYS = (MIN_WINDOW_KEY,)


class ReportError(RuntimeError):
    """Bag o log incompleti: l'analisi non e' possibile."""


# ------------------------------------------------------------------ utilita' pure

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def nearest_idx(ts: np.ndarray, t: float) -> int:
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return 0
    if i >= len(ts):
        return len(ts) - 1
    return i if abs(ts[i] - t) < abs(ts[i - 1] - t) else i - 1


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_from_meta(meta: dict):
    """(asse a, R_ref) da run_meta.json ("axis", "q_ref" xyzw). None se assenti o asse z con q_ref identita'
    (comportamento storico: yaw = arctan2(R[1,0], R[0,0]))."""
    ax, q = (meta or {}).get("axis"), (meta or {}).get("q_ref")
    if ax is None or q is None:
        return None
    ax, q = [float(v) for v in ax], [float(v) for v in q]
    if ax == [0.0, 0.0, 1.0] and q == [0.0, 0.0, 0.0, 1.0]:
        return None
    a = np.array(ax) / np.linalg.norm(ax)
    return a, quat_to_matrix(q)


def cube_angles(oR: np.ndarray, tilt=None) -> np.ndarray:
    """Angolo del cubo attorno all'asse [rad, (-pi, pi]]. tilt None: arctan2(R[1,0], R[0,0]) (asse z, q_ref = I).
    Altrimenti theta dell'angolo di R_rel = oR . R_ref^T attorno a a: con e = componente di R_ref.x ortogonale
    ad a, theta = atan2(a.(e x e'), e.e'), e' = R_rel e (R_rel = Rot(a, theta) per un moto rigido attorno a a)."""
    if tilt is None:
        return np.arctan2(oR[:, 1, 0], oR[:, 0, 0])
    a, R_ref = tilt
    e = R_ref[:, 0] - (R_ref[:, 0] @ a) * a
    e = e / np.linalg.norm(e)
    e1 = oR @ (R_ref.T @ e)   # (oR R_ref^T) e, shape (N, 3)
    return np.arctan2(np.cross(e, e1) @ a, e1 @ e)


def _jsonable(o):
    if isinstance(o, dict):
        return {k: _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


# ------------------------------------------------------------------ criteri di successo

def load_criteria(path=None) -> dict:
    path = Path(path) if path else SUCCESS_YAML
    text = path.read_text()
    try:
        import yaml
        return yaml.safe_load(text) or {}
    except ImportError:  # fallback minimale per il formato di s1_success.yaml
        out, cur = {}, None
        for raw in text.splitlines():
            line = raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            key, _, val = line.strip().partition(":")
            val = val.strip()
            if raw.startswith(" "):
                out[cur][key] = float(val)
            elif val == "":
                cur = key
                out[cur] = {}
            else:
                low = val.lower()
                if low in ("true", "false"):
                    out[key] = low == "true"
                else:
                    try:
                        out[key] = float(val)
                    except ValueError:
                        out[key] = val
        return out


def wait_range_s(omega_deg_s: float):
    """Intervallo ammesso di wait_s_actual [s]: [5, 5 + 360/|omega|]; None se omega e' nullo/non finito."""
    if omega_deg_s is None or not math.isfinite(omega_deg_s) or omega_deg_s == 0.0:
        return None
    return WAIT_MIN_S, WAIT_MIN_S + WAIT_FULL_TURN_DEG / abs(omega_deg_s)


def wait_out_of_range(wait_s: float, omega_deg_s: float) -> bool:
    """True se wait_s_actual e' fuori da wait_range_s(omega). Non influisce sul successo."""
    rng = wait_range_s(omega_deg_s)
    if rng is None or wait_s is None or not math.isfinite(wait_s):
        return False
    return not (rng[0] <= wait_s <= rng[1])


def run_warnings(meta: dict, omega_gc_deg_s: float, wait_s: float) -> list:
    """Avvisi di report.txt (non cambiano il successo)."""
    out = []
    if wait_out_of_range(wait_s, omega_gc_deg_s):
        lo, hi = wait_range_s(omega_gc_deg_s)
        out.append(f"{WAIT_WARNING} (attesa {wait_s:.2f} s, ammessa [{lo:.1f}, {hi:.1f}] s)")
    om = meta.get("omega_deg_s")
    if om is not None and abs(float(om) - omega_gc_deg_s) > OMEGA_META_TOL_DEG_S:
        out.append(f"omega del meta ({float(om):.4f} deg/s) != omega del GraspCommand ({omega_gc_deg_s:.4f} deg/s)")
    return out


def rotation_warnings(start_goal_deg) -> list:
    """Avviso se l'angolo start-goal supera il dominio verificato (None/NaN: nessun avviso)."""
    if start_goal_deg is None or not math.isfinite(float(start_goal_deg)):
        return []
    return [ROT_WARNING] if float(start_goal_deg) > ROT_VERIFIED_MAX_DEG else []


def parse_urdf_limits(urdf):
    """(lower, upper) [rad] dei <limit> di fer_joint1..7 (copia locale: niente import di s1_kinematic_metrics)."""
    import xml.etree.ElementTree as ET
    lims = {}
    for j in ET.parse(str(urdf)).getroot().iter("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("name", "") in JOINT_NAMES:
            lims[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    missing = [nm for nm in JOINT_NAMES if nm not in lims]
    if missing:
        raise ReportError(f"limiti URDF mancanti per {missing}")
    arr = np.array([lims[nm] for nm in JOINT_NAMES])
    return arr[:, 0], arr[:, 1]


def min_joint_margin(js_t, js_q, t_trig, t_end, follow_s, lower, upper, step: float = MARGIN_STEP_S) -> dict:
    """Margine di giunto minimo misurato: min su griglia [t_trig, t_end + follow_s] e sui 7 giunti di
    min(q - lower, upper - q), q interpolata linearmente. Ritorna deg, joint (1..7), t_rel_s (rel t_end).
    NaN se il bag finisce prima di t_end + follow_s (niente clamp all'ultimo campione)."""
    nan = float("nan")
    js_t, js_q = np.asarray(js_t, dtype=float), np.asarray(js_q, dtype=float)
    t_fend = t_end + follow_s
    if len(js_t) < 2 or not (t_fend >= t_trig) or js_t[-1] < t_fend - 1e-9:
        return {"deg": nan, "joint": None, "t_rel_s": nan}
    grid = np.append(np.arange(t_trig, t_fend, step), t_fend)
    q = np.stack([np.interp(grid, js_t, js_q[:, c]) for c in range(js_q.shape[1])], axis=1)
    mg = np.minimum(q - np.asarray(lower), np.asarray(upper) - q)
    i = int(np.argmin(mg.min(axis=1)))
    return {"deg": math.degrees(float(mg[i].min())), "joint": int(np.argmin(mg[i])) + 1,
            "t_rel_s": float(grid[i] - t_end)}


_RE_SELECTED = re.compile(r"grasp_planner_node: selected\b.*")

# Riga del nodo "grasp_planner_node: s1 timing: total_s=.. scan_ms=.. ... finalists=<n>" (grasp_planner_node.cpp, accanto a "s1 status=").
# Ogni stadio e' la SOMMA dei tempi di quello stadio sui finalisti esaminati (il flusso pigro intercala gli stadi); wall-clock.
_RE_TIMING = re.compile(r"grasp_planner_node: s1 timing:(.*)")
TIMING_KEYS = ("scan_ms", "rollout_ms", "clearance_ms", "hold_end_ms", "dynamic_ms", "posthoc_ms", "other_ms")


def parse_selection_timing(path) -> dict:
    """Campi di tempo della (ultima) riga "s1 timing:" del planner.log: TIMING_KEYS (ms) e finalists (int). Tutti None se il
    file o la riga mancano (log vecchi) o un campo e' assente/illeggibile."""
    out = {k: None for k in TIMING_KEYS}
    out["finalists"] = None
    try:
        text = Path(path).read_text(errors="replace")
    except (OSError, TypeError):
        return out
    body = None
    for m in _RE_TIMING.finditer(text):
        body = m.group(1)
    if body is None:
        return out
    kv = dict(re.findall(r"(\w+)=(\S+)", body))
    for k in TIMING_KEYS:
        try:
            out[k] = float(kv[k])
        except (KeyError, ValueError):
            pass
    try:
        out["finalists"] = int(kv["finalists"])
    except (KeyError, ValueError):
        pass
    return out


def parse_selected_start_goal(path) -> float | None:
    """start_goal_deg del ramo scelto dalla (ultima) riga "selected" del planner.log: [B] se wrist_flip=1,
    altrimenti [A]. None se il file/la riga/i campi mancano (log vecchi)."""
    try:
        text = Path(path).read_text(errors="replace")
    except (OSError, TypeError):
        return None
    line = None
    for m in _RE_SELECTED.finditer(text):
        line = m.group(0)
    if line is None:
        return None
    flip = re.search(r"wrist_flip=(\d)", line)
    num = r"(-?[0-9.]+(?:[eE][-+]?\d+)?)"
    a = re.search(r"start_goal_deg\[A\]=" + num, line)
    b = re.search(r"start_goal_deg\[B\]=" + num, line)
    if flip is None or a is None or b is None:
        return None
    return float((b if flip.group(1) == "1" else a).group(1))


def apply_interrupt(success: bool, meta: dict) -> bool:
    """Un run interrotto non e' mai un successo."""
    return bool(success) and not meta.get("interrupted")


def interrupt_line(meta: dict):
    """Riga di report.txt per un run interrotto (None se il run non e' stato interrotto)."""
    if not meta.get("interrupted"):
        return None
    rel = meta.get("interrupted_rel_s")
    return ("run interrotto a +{:.1f} s (dopo AT_END): successo = NO".format(rel) if rel is not None
            else "run interrotto prima di AT_END: successo = NO")


def evaluate_criteria(values: dict, criteria: dict):
    """values: nome criterio -> valore misurato. Ritorna (dict nome->{value,rule,ok}, success)."""
    res = {}
    for name, rule in criteria.items():
        if name in PARAM_KEYS:
            continue
        v = values.get(name)
        if isinstance(rule, bool):
            ok = v is not None and bool(v) == rule
            txt = f"== {str(rule).lower()}"
        else:
            finite = v is not None and math.isfinite(float(v))
            ok = finite
            parts = []
            if "min" in rule:
                ok = ok and float(v) >= rule["min"]
                parts.append(f">= {rule['min']:g}")
            if "max" in rule:
                ok = ok and float(v) <= rule["max"]
                parts.append(f"<= {rule['max']:g}")
            if "lt" in rule:
                ok = ok and float(v) < rule["lt"]
                parts.append(f"< {rule['lt']:g}")
            txt = " e ".join(parts)
        res[name] = {"value": v, "rule": txt, "ok": bool(ok)}
    return res, all(r["ok"] for r in res.values())


# Criteri riportati a parte: NON entrano nell'esito principale "success" (decisione di Lorenzo, 02/11/2026:
# successo = criteri 1-10; criterio 11 = margine di giunto misurato >= soglia, riportato separatamente).
# La soglia resta in tools/s1_success.yaml e non cambia.
SEPARATE_CRITERIA = ("min_joint_margin_deg",)


def split_success(crit: dict):
    """crit: primo valore di evaluate_criteria. Ritorna (success_1_10, criterion_11_ok, success_1_11):
    success_1_10 = AND dei criteri non in SEPARATE_CRITERIA; criterion_11_ok = AND dei criteri in SEPARATE_CRITERIA
    (None se nessuno e' presente); success_1_11 = AND di tutti (come il vecchio esito unico)."""
    main = [r["ok"] for n, r in crit.items() if n not in SEPARATE_CRITERIA]
    sep = [r["ok"] for n, r in crit.items() if n in SEPARATE_CRITERIA]
    success_1_10 = all(main)
    return success_1_10, (all(sep) if sep else None), success_1_10 and all(sep)


# ------------------------------------------------------------------ lettura log / bag

_RE_TRIGGER = re.compile(r"continuous TRIGGER at node clock t=([0-9.]+) s")
_RE_AT_END = re.compile(r"continuous AT_END at node clock t=([0-9.]+) s")
_RE_CLOSE_START = re.compile(r"gripper: CLOSE start t=([0-9.]+)")
_RE_CLOSE_DONE = re.compile(r"gripper: CLOSE done t=([0-9.]+)")


def parse_executor_log(path) -> dict:
    out = {"t_trigger": None, "t_at_end": None, "t_close_start": None, "t_close_done": None}
    pats = {"t_trigger": _RE_TRIGGER, "t_at_end": _RE_AT_END,
            "t_close_start": _RE_CLOSE_START, "t_close_done": _RE_CLOSE_DONE}
    with open(path, "r", errors="replace") as f:
        for line in f:
            for key, pat in pats.items():
                m = pat.search(line)
                if m:
                    out[key] = float(m.group(1))   # l'ultima occorrenza vince (come analyze_final2)
    return out


def read_bag(bag_dir) -> dict:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    db_files = sorted(glob.glob(os.path.join(str(bag_dir), "*.db3")))
    if not db_files:
        raise ReportError(f"nessun .db3 in {bag_dir}")
    conn = sqlite3.connect(db_files[0])
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topic_id = {name: tid for tid, name in cur.fetchall()}

    def rows(topic):
        if topic not in topic_id:
            return []
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
                    (topic_id[topic],))
        return cur.fetchall()

    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    Clock = get_message("rosgraph_msgs/msg/Clock")
    clocks = []
    for ts, raw in rows("/clock"):
        m = deserialize_message(raw, Clock)
        clocks.append((ts * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))

    gc, t_pub_wall = None, None
    GraspCommand = get_message("satellite_grasp_msgs/msg/GraspCommand")
    for ts, raw in rows("/grasp_command")[:1]:
        t_pub_wall = ts * 1e-9
        gc = deserialize_message(raw, GraspCommand)

    Odometry = get_message("nav_msgs/msg/Odometry")
    odo_t, odo_p, odo_R = [], [], []
    for _, raw in rows("/free_target_object/odometry"):
        m = deserialize_message(raw, Odometry)
        o = m.pose.pose
        odo_t.append(stamp(m.header))
        odo_p.append([o.position.x, o.position.y, o.position.z])
        odo_R.append(quat_to_matrix([o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w]))

    PoseStamped = get_message("geometry_msgs/msg/PoseStamped")
    tgt_t, tgt_p = [], []
    for _, raw in rows("/target_pose"):
        m = deserialize_message(raw, PoseStamped)
        tgt_t.append(stamp(m.header))
        tgt_p.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    JointState = get_message("sensor_msgs/msg/JointState")
    js_t, js_q, js_f1, js_f2 = [], [], [], []
    for _, raw in rows("/joint_states"):
        m = deserialize_message(raw, JointState)
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINT_NAMES):
            js_t.append(stamp(m.header))
            js_q.append([d[j] for j in JOINT_NAMES])
            js_f1.append(d.get("fer_finger_joint1", np.nan))
            js_f2.append(d.get("fer_finger_joint2", d.get("fer_finger_joint2_mimic", np.nan)))
    conn.close()

    return {
        "clocks": np.array(clocks), "grasp_cmd": gc, "t_pub_wall": t_pub_wall,
        "odo_t": np.array(odo_t), "odo_p": np.array(odo_p), "odo_R": np.array(odo_R),
        "tgt_t": np.array(tgt_t), "tgt_p": np.array(tgt_p),
        "js_t": np.array(js_t), "js_q": np.array(js_q),
        "js_f1": np.array(js_f1), "js_f2": np.array(js_f2),
    }


# ------------------------------------------------------------------ cinematica

class _Kin:
    """Pinocchio: TCP e facce interne dei pad (stessa costruzione di analyze_final2.py)."""

    def __init__(self, urdf):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.cmodel = pin.buildGeomFromUrdf(self.model, str(urdf), pin.GeometryType.COLLISION)
        self.cdata = self.cmodel.createData()
        self.tcp_id = self.model.getFrameId("fer_hand_tcp")
        names = [g.name for g in self.cmodel.geometryObjects]
        self.il = names.index("fer_leftfinger_3")
        self.ir = names.index("fer_rightfinger_3")
        try:
            self.pad_half_y = float(self.cmodel.geometryObjects[self.il].geometry.halfSide[1])
        except Exception:
            self.pad_half_y = PAD_HALF_Y_FALLBACK_M

    def fk(self, q7, f1, f2):
        pin = self.pin
        q = np.zeros(self.model.nq)
        q[:7] = q7
        q[7], q[8] = f1, f2
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.updateGeometryPlacements(self.model, self.data, self.cmodel, self.cdata)
        p_tcp = self.data.oMf[self.tcp_id].translation.copy()
        Ml, Mr = self.cdata.oMg[self.il], self.cdata.oMg[self.ir]
        in_l = Ml.translation - Ml.rotation @ np.array([0.0, self.pad_half_y, 0.0])
        in_r = Mr.translation + Mr.rotation @ np.array([0.0, self.pad_half_y, 0.0])
        return p_tcp, in_l, in_r


# ------------------------------------------------------------------ analisi

def analyze(bag_dir, executor_log, meta: dict | None = None, criteria: dict | None = None,
            urdf=None, min_window_s: float | None = None, planner_log=None) -> dict:
    meta = dict(meta or {})
    criteria = criteria if criteria is not None else load_criteria()
    if min_window_s is None:
        min_window_s = float(criteria.get(MIN_WINDOW_KEY, MIN_WINDOW_DEFAULT_S))
    ev = parse_executor_log(executor_log)
    t_trig, t_end = ev["t_trigger"], ev["t_at_end"]
    if t_trig is None or t_end is None:
        raise ReportError("executor log senza TRIGGER/AT_END")

    bag = read_bag(bag_dir)
    gc = bag["grasp_cmd"]
    if gc is None:
        raise ReportError("bag senza /grasp_command")
    for key in ("odo_t", "js_t", "tgt_t", "clocks"):
        if len(bag[key]) < 3:
            raise ReportError(f"bag con troppo pochi campioni su {key}")
    if np.isnan(bag["js_f1"]).any() or np.isnan(bag["js_f2"]).any():
        raise ReportError("joint_states senza giunti delle dita")

    kin = _Kin(urdf or URDF_PATH)

    th_star = gc.theta_star_rad
    omega = gc.omega_rad_s
    tau = gc.contact_time_s
    t_pub_sim = gc.header.stamp.sec + gc.header.stamp.nanosec * 1e-9

    # --- clock / RTF
    t_wall_c, t_sim_c = bag["clocks"][:, 0], bag["clocks"][:, 1]
    i_pub, i_trg = nearest_idx(t_sim_c, t_pub_sim), nearest_idx(t_sim_c, t_trig)
    dw = t_wall_c[i_trg] - t_wall_c[i_pub]
    rtf_pub_trig = (t_sim_c[i_trg] - t_sim_c[i_pub]) / dw if dw > 0 else float("nan")
    dw_tot = t_wall_c[-1] - t_wall_c[0]
    rtf_total = (t_sim_c[-1] - t_sim_c[0]) / dw_tot if dw_tot > 0 else float("nan")

    # --- odometria
    ot, op, oR = bag["odo_t"], bag["odo_p"], bag["odo_R"]
    tilt = tilt_from_meta(meta)   # None (asse z, q_ref = I): formula storica
    yaws = cube_angles(oR, tilt)   # theta attorno all'asse (a alpha = 0 e' lo yaw)
    yaws_unw = np.unwrap(yaws)
    p0, z0 = op[0], oR[0][:, 2]

    th_trig_exp = wrap_pi(th_star - omega * tau)
    yaw_trig_meas = wrap_pi(yaws[nearest_idx(ot, t_trig)])
    yaw_trig_err = math.degrees(wrap_pi(yaw_trig_meas - th_trig_exp))
    i_end = nearest_idx(ot, t_end)
    yaw_end_meas = wrap_pi(yaws[i_end])
    yaw_end_err = math.degrees(wrap_pi(yaw_end_meas - th_star))

    def yaw_slope(t_lo, t_hi):
        m = (ot >= t_lo) & (ot <= t_hi)
        if m.sum() < 3:
            return float("nan")
        return math.degrees(np.polyfit(ot[m], yaws_unw[m], 1)[0])

    def slope_1s(t):
        s = yaw_slope(t - 0.5, t + 0.5)
        return math.degrees(omega) if math.isnan(s) else s

    # --- primo contatto
    t_contact = t_end
    for t_s in np.arange(t_trig, t_end + 10.0, 0.05):
        i = nearest_idx(ot, t_s)
        disp = np.linalg.norm(op[i] - p0) * 1000.0
        tilt = math.degrees(math.acos(np.clip(np.dot(oR[i][:, 2], z0), -1.0, 1.0)))
        if (abs(slope_1s(t_s) - math.degrees(omega)) > CONTACT_SLOPE_TOL_DEG_S
                or disp > CONTACT_DISP_MM or tilt > CONTACT_TILT_DEG):
            t_contact = float(t_s)
            break

    # --- interpolatori (campionamento lineare, clamp ai bordi)
    jt = bag["js_t"]

    def q_at(t):
        return np.array([np.interp(t, jt, bag["js_q"][:, c]) for c in range(7)])

    def f_at(t):
        return float(np.interp(t, jt, bag["js_f1"])), float(np.interp(t, jt, bag["js_f2"]))

    def tgt_at(t):
        return np.array([np.interp(t, bag["tgt_t"], bag["tgt_p"][:, c]) for c in range(3)])

    def tcp_world(t):
        f1, f2 = f_at(t)
        return kin.fk(q_at(t), f1, f2)

    def tcp_cube(t):
        p_tcp = tcp_world(t)[0]
        i = nearest_idx(ot, t)
        return oR[i].T @ (p_tcp - op[i]) * 1000.0

    # --- dita
    fvals = {}
    for lbl, off in [("0", 0.0), ("1", 1.0), ("2", 2.0), ("5", 5.0), ("15", 15.0)]:
        f1, f2 = (v * 1000.0 for v in f_at(t_end + off))
        fvals[lbl] = {"t": t_end + off, "f1": f1, "f2": f2, "sum": f1 + f2, "min": min(f1, f2)}
    mpl = (jt >= t_end + 10.0) & (jt <= t_end + 20.0)
    if mpl.any():
        f1 = float(np.mean(bag["js_f1"][mpl])) * 1000.0
        f2 = float(np.mean(bag["js_f2"][mpl])) * 1000.0
        fvals["plateau"] = {"f1": f1, "f2": f2, "sum": f1 + f2, "min": min(f1, f2)}
    else:
        fvals["plateau"] = {"f1": np.nan, "f2": np.nan, "sum": np.nan, "min": np.nan}

    # --- pad vs parete
    pads = {}
    for lbl, off in [("0", 0.0), ("2", 2.0), ("5", 5.0), ("15", 15.0)]:
        t = t_end + off
        _, in_l, in_r = tcp_world(t)
        i = nearest_idx(ot, t)
        r = []
        for p_in in (in_l, in_r):
            c = oR[i].T @ (p_in - op[i])
            r.append(math.hypot(c[1], c[2]) * 1000.0)
        straddle = min(r) < COLLAR_R_INNER_MM and max(r) > COLLAR_R_OUTER_MM
        pads[lbl] = {"r_left_mm": r[0], "r_right_mm": r[1], "straddle": bool(straddle)}

    # --- inseguimento del cubo: TCP nel frame del cubo
    ref = tcp_cube(t_end + 3.0)

    def tcp_var(t_stop):
        ts = np.arange(t_end + 3.0, t_end + t_stop + 0.05, 0.1)
        return max(float(np.linalg.norm(tcp_cube(t) - ref)) for t in ts)

    var_3_8, var_3_15 = tcp_var(8.0), tcp_var(15.0)

    # --- scomposizione dello scostamento TCP-cubo (frame del cubo, assi fissati dal TCP a +3 s)
    e_ax = np.array([1.0, 0.0, 0.0])
    rad = np.array([0.0, ref[1], ref[2]])
    if np.linalg.norm(rad) > 1e-6:
        e_close = rad / np.linalg.norm(rad)
        e_tang = np.cross(e_ax, e_close)
    else:
        e_close = e_tang = None

    def comp(t):
        d = tcp_cube(t) - ref
        if e_close is None:
            return float("nan"), float("nan"), float(d @ e_ax)
        return float(d @ e_close), float(d @ e_tang), float(d @ e_ax)

    n8 = len(np.arange(t_end + 3.0, t_end + 8.0 + 0.05, 0.1))   # prefisso della griglia +3..+15 s
    ts_dec = np.arange(t_end + 3.0, t_end + 15.0 + 0.05, 0.1)
    dec = np.array([comp(t) for t in ts_dec])

    def comp_max(k, n):
        col = np.abs(dec[:n, k])
        return float(np.nanmax(col)) if np.isfinite(col).any() else float("nan")

    var_close, var_tang, var_axial = (comp_max(k, n8) for k in range(3))
    var_close_15, var_tang_15, var_axial_15 = (comp_max(k, len(dec)) for k in range(3))
    # NaN se una delle due componenti non e' definita (TCP sull'asse): il criterio non e' soddisfatto
    slip_15 = float("nan") if (math.isnan(var_tang_15) or math.isnan(var_axial_15)) else max(var_tang_15, var_axial_15)
    t_follow_end = t_end + float(meta.get("follow_s") if meta.get("follow_s") is not None else 3.0)
    wz_mean_4_8 = yaw_slope(t_follow_end + 4.0, t_follow_end + 8.0)

    # --- serie temporale (passo 0.5 s da AT_END a fine bag)
    t_last = min(float(ot[-1]), float(jt[-1]), float(t_sim_c[-1]))
    timeseries = []
    for t in np.arange(t_end, t_last + 1e-9, TIMESERIES_STEP_S):
        f1, f2 = f_at(t)
        dc, dt_, da = comp(t)
        timeseries.append({
            "t_rel": float(t - t_end), "f1_mm": f1 * 1000.0, "f2_mm": f2 * 1000.0,
            "yaw_deg": math.degrees(wrap_pi(float(yaws[nearest_idx(ot, t)]))),
            "yaw_rate_degps": yaw_slope(t - 0.5, t + 0.5),
            "d_close_mm": dc, "d_tang_mm": dt_, "d_axial_mm": da})
    slope_6_12 = yaw_slope(t_end + 6.0, t_end + 12.0)

    # --- errore TCP - target
    ts_err = np.arange(t_end, t_end + 15.0 + 0.02, 0.05)
    max_err_tgt = max(float(np.linalg.norm((tcp_world(t)[0] - tgt_at(t)) * 1000.0)) for t in ts_err)

    # --- cubo
    disp_all = np.linalg.norm(op - p0, axis=1) * 1000.0
    tilt_all = np.degrees(np.arccos(np.clip(oR[:, :, 2] @ z0, -1.0, 1.0)))
    pre = ot <= t_end
    disp_pre_end = float(disp_all[pre].max()) if pre.any() else float("nan")

    # --- sin(q6)
    mq = (jt >= t_trig) & (jt <= t_end + 15.0)
    sin_q6_min = float(np.min(np.sin(bag["js_q"][mq, 5]))) if mq.any() else float("nan")

    # plateau +10..+20 s: serve il bag fino a +20 s (tolleranza 0.1 s), altrimenti il criterio e' NaN
    t_bag_end = min(float(jt[-1]), float(ot[-1]))
    window_short = t_bag_end < t_end + 20.0 - 0.1
    f_sum_plateau = float("nan") if window_short else fvals["plateau"]["sum"]
    min_f_plateau = float("nan") if window_short else fvals["plateau"]["min"]

    # --- fondo della finestra registrata (rel AT_END): NaN/None se la finestra e' piu' corta del minimo
    window_end_rel = t_bag_end - t_end
    end_short = window_end_rel < min_window_s
    gap_end = slip_end = float("nan")
    pad_end = None
    if not end_short:
        m10 = (jt >= t_bag_end - 10.0) & (jt <= t_bag_end)
        if m10.any():
            gap_end = float(np.mean(bag["js_f1"][m10] + bag["js_f2"][m10])) * 1000.0
        _, in_l, in_r = tcp_world(t_bag_end)
        i = nearest_idx(ot, t_bag_end)
        r_end = [math.hypot(*(oR[i].T @ (p_in - op[i]))[1:]) * 1000.0 for p_in in (in_l, in_r)]
        pad_end = bool(min(r_end) < COLLAR_R_INNER_MM and max(r_end) > COLLAR_R_OUTER_MM)
        ts_end = np.arange(t_end + 3.0, t_bag_end + 1e-9, 0.1)
        dec_end = np.array([comp(t) for t in ts_end])
        tang_end, axial_end = np.abs(dec_end[:, 1]), np.abs(dec_end[:, 2])
        if np.isfinite(tang_end).all() and np.isfinite(axial_end).all():
            slip_end = float(max(tang_end.max(), axial_end.max()))
    # --- criterio 11: margine di giunto minimo misurato in [TRIGGER, AT_END + follow_s]
    lim_lo, lim_hi = parse_urdf_limits(urdf or URDF_PATH)
    mgn = min_joint_margin(jt, bag["js_q"], t_trig, t_end, t_follow_end - t_end, lim_lo, lim_hi)
    if planner_log is None:
        planner_log = Path(executor_log).parent / "planner.log"
    start_goal_deg = parse_selected_start_goal(planner_log)
    selection_timing = parse_selection_timing(planner_log)   # None se il planner.log non ha la riga "s1 timing:" (log vecchi)
    crit_values = {
        "min_joint_margin_deg": mgn["deg"],
        "gap_end_mean_last10s": gap_end,
        "pad_straddle_end": pad_end,
        "slip_tang_axial_3_end": slip_end,
        "f_sum_plateau_10_20s": f_sum_plateau,
        "min_f_plateau_10_20s": min_f_plateau,
        "slip_tang_axial_3_15s": slip_15,
        "pad_straddle_2s": pads["2"]["straddle"],
        "pad_straddle_5s": pads["5"]["straddle"],
        "pad_straddle_15s": pads["15"]["straddle"],
        "cube_disp_pre_end_mm": disp_pre_end,
    }
    crit, _success_all = evaluate_criteria(crit_values, criteria)
    success, criterion_11_ok, success_1_11 = split_success(crit)
    success = apply_interrupt(success, meta)           # esito principale: criteri 1-10
    success_1_11 = apply_interrupt(success_1_11, meta)  # confronto con le analisi precedenti (11 criteri)

    res = {
        "meta": meta,
        "selection": {
            "evaluations": int(gc.evaluations), "selection_elapsed_s": gc.selection_elapsed_s,
            "k": int(gc.k), "theta_star_deg": math.degrees(th_star), "d_pos_mm": gc.d_pos_m * 1000.0,
            "cost": gc.cost, "w_hat": gc.w_hat,
            "min_joint_margin_deg": math.degrees(gc.min_joint_margin_rad),
            "wrist_flip": bool(gc.wrist_flip), "provisional_delay_s": gc.provisional_delay_s,
            "start_goal_deg": start_goal_deg,
            **selection_timing,
        },
        "launch": {
            "snapshot_theta_deg": math.degrees(gc.snapshot_theta_rad), "snapshot_t_s": gc.snapshot_t_s,
            "t_pub_sim_s": t_pub_sim, "wait_actual_s": t_trig - t_pub_sim,
            "omega_deg_s": math.degrees(omega),   # omega del GraspCommand = quello del run (non -2 fisso)
            "yaw_trig_meas_deg": math.degrees(yaw_trig_meas), "yaw_trig_exp_deg": math.degrees(th_trig_exp),
            "yaw_trig_err_deg": yaw_trig_err,
            "yaw_end_meas_deg": math.degrees(yaw_end_meas), "yaw_end_err_deg": yaw_end_err,
            "rtf_pub_trigger": rtf_pub_trig, "rtf_total": rtf_total,
        },
        "instants": {
            "bag_start_s": float(t_sim_c[0]), "trigger_s": t_trig, "first_contact_s": t_contact,
            "at_end_s": t_end, "close_start_s": ev["t_close_start"], "close_done_s": ev["t_close_done"],
        },
        "grasp": {
            "fingers_mm": fvals, "pads": pads,
            "tcp_cube_var_3_8s_mm": var_3_8, "tcp_cube_var_3_15s_mm": var_3_15,
            "yaw_slope_6_12s_deg_s": slope_6_12, "max_err_tgt_0_15s_mm": max_err_tgt,
            "cube_disp_max_mm": float(disp_all.max()), "cube_disp_pre_end_mm": disp_pre_end,
            "tilt_max_deg": float(tilt_all.max()), "sin_q6_min": sin_q6_min,
            "tcp_cube_var_close_3_8s_mm": var_close, "tcp_cube_var_tang_3_8s_mm": var_tang,
            "tcp_cube_var_axial_3_8s_mm": var_axial,
            "f1_f2_plateau_min_10_20s_mm": fvals["plateau"]["min"],
            "wz_cube_mean_4_8s_degps": wz_mean_4_8, "t_follow_end_s": t_follow_end,
            "tcp_cube_var_close_3_15s_mm": var_close_15, "tcp_cube_var_tang_3_15s_mm": var_tang_15,
            "tcp_cube_var_axial_3_15s_mm": var_axial_15, "slip_tang_axial_3_15s_mm": slip_15,
            "f_sum_plateau_10_20s_mm": f_sum_plateau,
            "window_end_rel_s": window_end_rel, "gap_end_mean_last10s_mm": gap_end,
            "pad_straddle_end": pad_end, "slip_tang_axial_3_end_mm": slip_end,
            "min_joint_margin_deg": mgn["deg"], "min_joint_margin_joint": mgn["joint"],
            "min_joint_margin_t_rel_s": mgn["t_rel_s"],
        },
        "criteria": crit,
        "warnings": run_warnings(meta, math.degrees(omega), t_trig - t_pub_sim) + rotation_warnings(start_goal_deg),
        "window_short": bool(window_short), "bag_end_rel_s": t_bag_end - t_end,
        "end_window_short": bool(end_short), "min_window_after_at_end_s": float(min_window_s),
        "success": bool(success),
        "criterion_11_ok": None if criterion_11_ok is None else bool(criterion_11_ok),
        "success_1_11": bool(success_1_11),
        "timeseries": timeseries,
    }
    res["row"] = csv_row(res)
    return _jsonable(res)


# ------------------------------------------------------------------ uscite

def csv_row(res: dict) -> dict:
    m, s, la, g = res["meta"], res["selection"], res["launch"], res["grasp"]
    fm, pads = g["fingers_mm"], g["pads"]
    return {
        "tag": m.get("tag"), "depth": m.get("depth"), "kt": m.get("kt"),
        "follow_s": m.get("follow_s"), "hold_s": m.get("hold_s"), "squeeze": m.get("squeeze"),
        "spawn_yaw": m.get("spawn_yaw_deg"),
        "k": s["k"], "theta_star": s["theta_star_deg"], "d_pos": s["d_pos_mm"], "cost": s["cost"],
        "wait_s": s["provisional_delay_s"],
        "yaw_trig_err": la["yaw_trig_err_deg"], "yaw_end_err": la["yaw_end_err_deg"],
        "pad_straddle_2s": int(pads["2"]["straddle"]),
        "f_sum_2s": fm["2"]["sum"], "f_sum_5s": fm["5"]["sum"], "f_sum_15s": fm["15"]["sum"],
        "min_f_5s": fm["5"]["min"],
        "tcp_cube_var_3_8s": g["tcp_cube_var_3_8s_mm"], "tcp_cube_var_3_15s": g["tcp_cube_var_3_15s_mm"],
        "yaw_slope_6_12s": g["yaw_slope_6_12s_deg_s"], "max_err_tgt_0_15s": g["max_err_tgt_0_15s_mm"],
        "cube_disp_max": g["cube_disp_max_mm"], "tilt_max": g["tilt_max_deg"],
        "sin_q6_min": g["sin_q6_min"], "success": int(res["success"]),
        "stop_cube": int(bool(m.get("stop_cube", False))),
        "wz_after_stop_degps": m.get("wz_after_stop_degps"),
        "wait_s_actual": la["wait_actual_s"],
        "tcp_cube_var_close_3_8s": g["tcp_cube_var_close_3_8s_mm"],
        "tcp_cube_var_tang_3_8s": g["tcp_cube_var_tang_3_8s_mm"],
        "tcp_cube_var_axial_3_8s": g["tcp_cube_var_axial_3_8s_mm"],
        "f1_f2_plateau_min_10_20s": g["f1_f2_plateau_min_10_20s_mm"],
        "wz_cube_mean_4_8s": g["wz_cube_mean_4_8s_degps"],
        "tcp_cube_var_tang_3_15s": g["tcp_cube_var_tang_3_15s_mm"],
        "tcp_cube_var_axial_3_15s": g["tcp_cube_var_axial_3_15s_mm"],
        "pad_straddle_5s": int(pads["5"]["straddle"]),
        "pad_straddle_15s": int(pads["15"]["straddle"]),
        "f_sum_plateau_10_20s": g["f_sum_plateau_10_20s_mm"],
        "tcp_cube_var_close_3_15s": g["tcp_cube_var_close_3_15s_mm"],
        "window_end_rel_s": g["window_end_rel_s"],
        "gap_end_mean_last10s": g["gap_end_mean_last10s_mm"],
        "pad_straddle_end": None if g["pad_straddle_end"] is None else int(g["pad_straddle_end"]),
        "slip_tang_axial_3_end": g["slip_tang_axial_3_end_mm"],
        "omega_deg_s": m["omega_deg_s"] if m.get("omega_deg_s") is not None else la.get("omega_deg_s"),
        "center_xyz": _center_cell(m.get("center_xyz")),
        "arm": m.get("arm"),
        "min_joint_margin_deg": g.get("min_joint_margin_deg"),
        "min_joint_margin_joint": g.get("min_joint_margin_joint"),
        "min_joint_margin_t_rel_s": g.get("min_joint_margin_t_rel_s"),
        "start_goal_deg": s.get("start_goal_deg"),
        "criterion_11_ok": None if res.get("criterion_11_ok") is None else int(res["criterion_11_ok"]),
        "success_1_11": None if res.get("success_1_11") is None else int(res["success_1_11"]),
    }


def _center_cell(c):
    """center_xyz del meta (lista di 3 numeri) -> 'X,Y,Z' (None se assente)."""
    if c is None:
        return None
    return ",".join(f"{float(v):g}" for v in c)


def _fmt_cell(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return "" if not math.isfinite(v) else f"{v:.4f}"
    return str(v)


def append_csv(path, row: dict):
    path = Path(path)
    new = not path.exists() or path.stat().st_size == 0
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(CSV_COLUMNS)
        w.writerow([_fmt_cell(row.get(c)) for c in CSV_COLUMNS])


def render_txt(res: dict) -> str:
    """Report testuale, stesse sezioni sempre, <= MAX_TXT_LINES righe. [M]=misurato [C]=calcolato."""
    m, s, la, ins, g = res["meta"], res["selection"], res["launch"], res["instants"], res["grasp"]

    def n(v, fmt="{:.3f}"):
        return "n/d" if v is None or (isinstance(v, float) and not math.isfinite(v)) else fmt.format(v)

    L = []
    L.append(f"S1 REPORT tag={m.get('tag')}   [M]=misurato [C]=calcolato   tempi = sim /clock")
    L.append(f"run: depth={m.get('depth')} kt={m.get('kt')} follow_s={m.get('follow_s')} "
             f"hold_s={m.get('hold_s')} squeeze={m.get('squeeze')} spawn_yaw={m.get('spawn_yaw_deg')} deg "
             f"omega={n(la['omega_deg_s'], '{:.3f}')} deg/s center={_center_cell(m.get('center_xyz')) or 'n/d'} "
             f"arm={m.get('arm') or 'n/d'}")
    if m.get("planner_param"):
        L.append("planner-param: " + " ".join(f"{k}={v}" for k, v in m["planner_param"].items()))
    L.append("== 1 SELEZIONE (GraspCommand) [M] ==")
    L.append(f"evaluations={s['evaluations']}  selection_elapsed={n(s['selection_elapsed_s'])} s  "
             f"k={s['k']} (psi={s['k'] * 90.0:.1f} deg)  wrist_flip={str(s['wrist_flip']).lower()}")
    L.append(f"theta*={n(s['theta_star_deg'], '{:.4f}')} deg  d_pos={n(s['d_pos_mm'], '{:.4f}')} mm  "
             f"cost={n(s['cost'], '{:.6f}')}  w_hat={n(s['w_hat'], '{:.4f}')}")
    L.append(f"margine giunti min={n(s['min_joint_margin_deg'])} deg  "
             f"provisional_delay (wait_s)={n(s['provisional_delay_s'])} s")
    L.append("== 2 LANCIO ==")
    L.append(f"snapshot [M]: theta={n(la['snapshot_theta_deg'], '{:.4f}')} deg a t={n(la['snapshot_t_s'], '{:.4f}')} s; "
             f"GraspCommand pubblicato a t={n(la['t_pub_sim_s'], '{:.4f}')} s")
    L.append(f"attesa pub->TRIGGER [C]: {n(la['wait_actual_s'])} s   "
             f"RTF pub->TRIGGER={n(la['rtf_pub_trigger'], '{:.4f}')}  RTF totale={n(la['rtf_total'], '{:.4f}')}")
    L.append(f"yaw al TRIGGER [M odom]: {n(la['yaw_trig_meas_deg'], '{:.4f}')} deg, atteso wrapPi(theta*-omega*tau) "
             f"{n(la['yaw_trig_exp_deg'], '{:.4f}')} -> err {n(la['yaw_trig_err_deg'], '{:.4f}')} deg")
    L.append(f"yaw a AT_END [M odom]: {n(la['yaw_end_meas_deg'], '{:.4f}')} deg -> err vs theta* "
             f"{n(la['yaw_end_err_deg'], '{:.4f}')} deg")
    L.append("== 3 ISTANTI (sim t [s]; rel = rispetto ad AT_END) ==")
    L.append(f"inizio bag {n(ins['bag_start_s'], '{:.3f}')} | TRIGGER {n(ins['trigger_s'], '{:.3f}')} | "
             f"primo contatto [C] {n(ins['first_contact_s'], '{:.3f}')} (rel {n(ins['first_contact_s'] - ins['at_end_s'])})")
    L.append(f"AT_END {n(ins['at_end_s'], '{:.3f}')} (rollout {n(ins['at_end_s'] - ins['trigger_s'])} s) | "
             f"CLOSE start {n(ins['close_start_s'], '{:.3f}')} | CLOSE done {n(ins['close_done_s'], '{:.3f}')}")
    L.append("== 4 CRITERI DI PRESA ==")
    L.append("dita [M], rel AT_END:     f1[mm]   f2[mm]  f1+f2[mm]  min[mm]")
    for lbl, name in [("0", "+0 s"), ("1", "+1 s"), ("2", "+2 s"), ("5", "+5 s"), ("15", "+15 s"),
                      ("plateau", "plateau +10..+20 s")]:
        f = g["fingers_mm"][lbl]
        L.append(f"  {name:<20} {n(f['f1'], '{:8.3f}')} {n(f['f2'], '{:8.3f}')} "
                 f"{n(f['sum'], '{:9.3f}')} {n(f['min'], '{:8.3f}')}")
    pr = g["pads"]
    L.append("raggio facce interne pad [C] (parete [63.5, 72.5] mm), L / R / a cavallo:")
    L.append("  " + " | ".join(
        f"+{k} s: {n(pr[k]['r_left_mm'], '{:.2f}')}/{n(pr[k]['r_right_mm'], '{:.2f}')}/"
        f"{'SI' if pr[k]['straddle'] else 'NO'}" for k in ("0", "2", "5", "15")))
    L.append(f"TCP nel frame cubo [C]: max|d| in [+3,+8] s={n(g['tcp_cube_var_3_8s_mm'])} mm, "
             f"in [+3,+15] s={n(g['tcp_cube_var_3_15s_mm'])} mm")
    L.append(f"TCP-cubo per asse [C]: close [+3,+8]/[+3,+15]={n(g['tcp_cube_var_close_3_8s_mm'], '{:.3f}')}/"
             f"{n(g['tcp_cube_var_close_3_15s_mm'], '{:.3f}')} mm (solo riportato) | tang [+3,+8]/[+3,+15]="
             f"{n(g['tcp_cube_var_tang_3_8s_mm'], '{:.3f}')}/{n(g['tcp_cube_var_tang_3_15s_mm'], '{:.3f}')} mm | "
             f"axial={n(g['tcp_cube_var_axial_3_8s_mm'], '{:.3f}')}/{n(g['tcp_cube_var_axial_3_15s_mm'], '{:.3f}')} mm")
    L.append(f"yaw slope [+6,+12] s = {n(g['yaw_slope_6_12s_deg_s'], '{:.4f}')} deg/s | "
             f"wz cubo medio [+4,+8] s dopo follow={n(g['wz_cube_mean_4_8s_degps'], '{:.4f}')} deg/s (solo riportati)")
    L.append(f"max err TCP-target [+0,+15] s={n(g['max_err_tgt_0_15s_mm'])} mm | "
             f"cubo: disp max={n(g['cube_disp_max_mm'], '{:.4f}')} mm (prima di AT_END {n(g['cube_disp_pre_end_mm'], '{:.4f}')}), "
             f"tilt max={n(g['tilt_max_deg'], '{:.4f}')} deg | sin(q6) min={n(g['sin_q6_min'], '{:.6f}')}")
    L.append("criteri (soglie fissate il 15/10/2026, criterio 11 del 19/10/2026: tools/s1_success.yaml):")
    if res.get("window_short"):
        L.append(f"  finestra registrata troppo corta: bag fino a +{n(res.get('bag_end_rel_s'))} s < +20 s "
                 "(criteri sul plateau = NaN, run NON riuscito)")
    if res.get("end_window_short"):
        L.append(f"  finestra registrata troppo corta: +{n(res.get('bag_end_rel_s'))} s < "
                 f"{n(res.get('min_window_after_at_end_s'), '{:.0f}')} s dopo AT_END "
                 "(criteri sul fondo = NaN, run NON riuscito)")
    for name, c in res["criteria"].items():
        v = c["value"]
        vs = ("SI" if v else "NO") if isinstance(v, bool) else n(v, "{:.4f}")
        extra = ""
        if name == "min_joint_margin_deg":
            extra = f" [giunto {g.get('min_joint_margin_joint')}, {n(g.get('min_joint_margin_t_rel_s'), '{:+.2f}')} s da AT_END]"
        L.append(f"  [{'OK' if c['ok'] else 'KO'}] {name} = {vs}{extra}  (richiesto {c['rule']})")
    for w in res.get("warnings", []):
        L.append(f"AVVISO: {w}")
    if interrupt_line(m):
        L.append(interrupt_line(m))
    # una sola riga (MAX_TXT_LINES): esito principale = criteri 1-10, criterio 11 a parte, esito 1-11 per confronto
    c11, s111 = res.get("criterion_11_ok"), res.get("success_1_11")
    L.append(f"SUCCESSO (criteri 1-10): {'SI' if res['success'] else 'NO'} | criterio 11 (a parte): "
             f"{'n/d' if c11 is None else ('OK' if c11 else 'KO')} | SUCCESSO 1-11: "
             f"{'n/d' if s111 is None else ('SI' if s111 else 'NO')}")
    if len(L) > MAX_TXT_LINES:
        raise ReportError(f"report di {len(L)} righe > {MAX_TXT_LINES}")
    return "\n".join(L) + "\n"


def write_outputs(res: dict, outdir, csv_path=None):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    res = dict(res)
    ts = res.pop("timeseries", None)   # niente serie nel JSON: va in timeseries.csv
    (outdir / "report.json").write_text(json.dumps(res, indent=2))
    if ts:
        with open(outdir / "timeseries.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(TIMESERIES_COLUMNS)
            for row in ts:
                w.writerow([_fmt_cell(row.get(c)) for c in TIMESERIES_COLUMNS])
    (outdir / "report.txt").write_text(render_txt(res))
    if csv_path:
        append_csv(csv_path, res["row"])


# ------------------------------------------------------------------ CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1], formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("bag_dir")
    ap.add_argument("--executor-log", help="default: <bag_dir>/../executor.log")
    ap.add_argument("--meta", help="JSON con tag/depth/kt/follow_s/hold_s/squeeze/spawn_yaw_deg "
                                   "(default: <bag_dir>/../run_meta.json se esiste)")
    ap.add_argument("--criteria", default=str(SUCCESS_YAML))
    ap.add_argument("--outdir", help="default: <bag_dir>/..")
    ap.add_argument("--csv", help="aggiunge la riga a questo CSV")
    ap.add_argument("--urdf", default=str(URDF_PATH))
    ap.add_argument("--planner-log", help="default: <executor.log dir>/planner.log (start_goal_deg del ramo scelto)")
    ap.add_argument("--tag")
    ap.add_argument("--min-window-s", type=float, default=None,
                    help=f"sovrascrive {MIN_WINDOW_KEY} del yaml (per rianalizzare bag vecchi piu' corti)")
    args = ap.parse_args(argv)

    bag_dir = Path(args.bag_dir).resolve()
    parent = bag_dir.parent
    elog = Path(args.executor_log) if args.executor_log else parent / "executor.log"
    meta_path = Path(args.meta) if args.meta else parent / "run_meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    if args.tag:
        meta["tag"] = args.tag
    meta.setdefault("tag", bag_dir.parent.name)
    try:
        if not elog.exists():
            raise ReportError(f"executor log non trovato: {elog}")
        res = analyze(bag_dir, elog, meta, load_criteria(args.criteria), args.urdf,
                      min_window_s=args.min_window_s, planner_log=args.planner_log)
        write_outputs(res, args.outdir or parent, args.csv)
    except (ReportError, OSError, ImportError) as e:
        print(f"s1_report FAIL: {e}", file=sys.stderr)
        return 1
    print(render_txt(res), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
