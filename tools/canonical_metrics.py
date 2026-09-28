#!/usr/bin/env python3
"""
canonical_metrics.py - metriche canoniche per run Gazebo (posizione, w_trans, regressione).

Nessuno script/extractor esistente viene modificato o importato: le definizioni sotto
riproducono ESPLICITAMENTE quelle degli script di analisi, ognuna con un nome proprio.

DEFINIZIONI CONGELATE
---------------------
Serie: target = target_aligned_<run>.csv, actual = actual_pose_<run>.csv,
       js = joint_states_<run>.csv. Colonna "t" = receive time del bag in secondi, con
       origine PROPRIA di ogni file (target e actual condividono t0; js ha il suo).

Istanti (indici sul target):
  t_settle_1e4 : t[k], k = primo indice da cui TUTTE le righe successive del target sono a
                 distanza <= 1e-4 m dall'ULTIMA riga (k=0 se nessuna riga la supera).
                 Uguale a find_clamp_index/find_clamp_time di compute_scatter_analysis,
                 run_satellite_rotation_experiment, format_both_tables, ecc. (i+1).
  t_step_1e5   : t[i], i = massimo indice con |p[i]-p[i-1]| >= 1e-5 m (0 se non esiste).
                 Uguale a find_clamp_index di evaluate_cartesian_tracking_headless.py (i).
  t_last       : t dell'ultima riga del target.

Errore di posizione (mm):
  err_paired[i] = |actual_pos[i] - target_pos[j(i)]|, j(i) = searchsorted(target.t, actual.t[i],
                  'left') limitato a [0, n_target-1] (== bisect_left + clip degli script).
  *_paired_at_<istante>: err_paired[j], j = searchsorted(actual.t, t_istante, 'left'); errore se
                  t_istante e' oltre l'ultimo t di actual (nessun clip silenzioso).
  *_to_target_at_<istante>: |actual_pos[j] - target_pos[idx_istante]| (definizione di
                  compute_scatter_analysis.eval_at_tau; e' la y della regressione).
  err_pos_mean_all_mm    : media di err_paired su TUTTE le righe di actual.
  err_pos_mean_cruise_mm : media di err_paired sulle righe con 2.0 <= actual.t <= actual.t[-1]-3.0.
  err_pos_max_mm / err_pos_last_mm : max / ultimo valore di err_paired.

Manipolabilita' traslazionale: w = sqrt(det(Jv Jv^T)) = prodotto dei valori singolari di Jv
  (3x7, punto di riferimento = origine del frame, assi della base). Frame:
  link8 = origine di fer_link8; tcp = link8 + R_link8 @ [0, 0, 0.1034].

Asse dei tempi per w (js):
  as_scripts : tau_js = t_settle_1e4 (asse target) usato cosi' com'e' sull'asse js.
  aligned    : tau_js = t_settle_1e4 + (js.t[-1] - target.t[-1]). APPROSSIMAZIONE: presume che
               l'ultimo messaggio di js e l'ultimo del target siano simultanei (NON VERIFICATO).
               Se in futuro esiste una colonna t_abs comune, va usata al posto di questa.
  Istanti di w: pre1s_mean = media di w sulle righe js con tau_js-1 <= t <= tau_js (errore se
               vuota); at_t_settle = w alla prima riga js con t >= tau_js (errore se oltre la
               fine); last_row = w all'ultima riga js.

Variante sim_time (richiede la chiave `bag` nel manifest; legge header.stamp dai .db3 con
  bag_time_probe.py, senza ROS). Le CSV sono verificate riga per riga contro i timestamp di
  ricezione del bag (verify_csv_alignment): se non coincidono -> errore. Tutti i tempi sono gli
  stamp assoluti (sim) dei messaggi, comuni ai tre topic (nessun riallineamento):
  t_settle_sim = stamp del messaggio target di indice settle (settle_index e' calcolato sulle
  posizioni, come sempre). Errore: primo messaggio actual con stamp >= t_settle_sim (errore se
  oltre la fine). w: finestre pre1s/pre2s = righe js con t_settle_sim-W <= stamp <= t_settle_sim
  (errore se vuote o se la finestra esce dal range di js); at_t_settle = riga js con lo stamp
  MINIMO tra quelle con stamp >= t_settle_sim; last_row = ultima riga js in ordine di ricezione
  (errore se il suo stamp e' epoch-like). target e actual: stamp non monotoni -> errore.
  joint_states: gli stamp dei bag esistenti NON sono monotoni (sequenze mescolate nello stesso
  topic): finestre e at_t_settle sono definiti per stamp, senza richiedere l'ordine; le anomalie
  sono contate nelle colonne js_stamp_n_*.

Regressione: y = err_pos_to_target_at_t_settle_1e4_mm (per sim_time: la stessa quantita' con il
  messaggio actual scelto per stamp, colonna ..._sim_mm), x = una colonna w, sui run
  family == regression_goal. r2_logo = R^2 leave-one-GOAL-out: ad ogni fold si escludono TUTTI i
  run dello stesso `group` (Goal 1 = 2 run, Goal 2 = 3, Goal 3 = 1, Goal 4 = 3, Goal 5 = 1: 5 fold)
  e si predicono; 1 - sum((y-yhat_cv)^2)/sum((y-mean(y))^2).

Uso:
  python3 tools/canonical_metrics.py [--manifest tools/runs_manifest.yaml] [--out tools/canonical_out]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))
import bag_time_probe as btp  # noqa: E402  (lettura header.stamp dai .db3, senza ROS)

WS_ROOT = TOOLS_DIR.parent
URDF_PATH = WS_ROOT / "fer_flat_effort.urdf"
DEFAULT_MANIFEST = TOOLS_DIR / "runs_manifest.yaml"
DEFAULT_OUT = TOOLS_DIR / "canonical_out"
DEFAULT_DATA_DIR = TOOLS_DIR / "gazebo_cartesian_eval" / "data"
DEFAULT_BAGS_DIR = TOOLS_DIR / "gazebo_cartesian_eval" / "bags"

TCP_OFFSET_M = (0.0, 0.0, 0.1034)
TOL_SETTLE_M = 1e-4
TOL_STEP_M = 1e-5
PRE_WINDOW_S = 1.0
CRUISE_START_S = 2.0
CRUISE_END_MARGIN_S = 3.0

FRAMES = ("link8", "tcp")
AXIS_VARIANTS = ("as_scripts", "aligned")
W_INSTANTS = ("pre1s_mean", "at_t_settle", "last_row")
SIM_VARIANT = "sim_time"
SIM_WINDOWS_S = (1.0, 2.0)
SIM_W_INSTANTS = ("pre1s_mean", "pre2s_mean", "at_t_settle", "last_row")
Y_COL = "err_pos_to_target_at_t_settle_1e4_mm"
Y_COL_SIM = "err_pos_to_target_at_t_settle_1e4_sim_mm"
JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]

FAMILIES = ("regression_goal", "satellite_phase", "baseline")
KT_SOURCES = ("yaml_commit", "script_argument", "inferred_from_error", "unknown", "not_applicable")
FRAMES_AT_RECORDING = ("link8", "tcp")
FILE_ROLES = (("target_aligned", "target_aligned"), ("actual_pose", "actual_pose"),
              ("joint_states", "joint_states"))
_ENTRY_KEYS = {"run_id", "family", "group", "phase_deg", "files", "kt", "kt_source", "kt_note",
               "recorded_utc", "frame_at_recording", "frame_source", "bag"}
_ENTRY_REQUIRED = {"run_id", "family", "files", "kt", "kt_source", "recorded_utc",
                   "frame_at_recording"}


# --------------------------------------------------------------------------------------
# Cinematica (numpy). Catena dai giunti fer_joint1..7 + fer_joint8 dell'URDF.
# --------------------------------------------------------------------------------------
def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin_transform(xyz, rpy) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = _rpy_matrix(*rpy)
    M[:3, 3] = xyz
    return M


class PandaKinematics:
    """FK e Jacobiano traslazionale del Panda (URDF fer_flat_effort.urdf), frame link8 e tcp."""

    def __init__(self, urdf_path: Path = URDF_PATH):
        urdf_path = Path(urdf_path)
        if not urdf_path.is_file():
            raise FileNotFoundError(f"URDF non trovato: {urdf_path}")
        joints = {j.get("name"): j for j in ET.parse(urdf_path).getroot().findall("joint")}
        self._fixed = []
        for i in range(1, 8):
            j = self._joint(joints, f"fer_joint{i}")
            axis = j.find("axis")
            if axis is None or [float(v) for v in axis.get("xyz").split()] != [0.0, 0.0, 1.0]:
                raise ValueError(f"fer_joint{i}: asse diverso da [0,0,1] (non supportato)")
            self._fixed.append(self._origin(j, f"fer_joint{i}"))
        self._t8 = self._origin(self._joint(joints, "fer_joint8"), "fer_joint8")
        self._tcp = np.array(TCP_OFFSET_M, dtype=float)

    @staticmethod
    def _joint(joints, name):
        if name not in joints:
            raise ValueError(f"giunto {name} assente nell'URDF")
        return joints[name]

    @staticmethod
    def _origin(joint, name):
        o = joint.find("origin")
        if o is None or o.get("xyz") is None or o.get("rpy") is None:
            raise ValueError(f"{name}: <origin xyz rpy> mancante (nessun default)")
        return _origin_transform([float(v) for v in o.get("xyz").split()],
                                 [float(v) for v in o.get("rpy").split()])

    def _chain(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (7,):
            raise ValueError(f"q deve avere 7 elementi, ricevuti shape {q.shape}")
        M = np.eye(4)
        zs = np.empty((7, 3))
        ps = np.empty((7, 3))
        for i in range(7):
            M = M @ self._fixed[i]
            zs[i] = M[:3, 2]
            ps[i] = M[:3, 3]
            c, s = math.cos(q[i]), math.sin(q[i])
            Rz = np.eye(4)
            Rz[0, 0], Rz[0, 1], Rz[1, 0], Rz[1, 1] = c, -s, s, c
            M = M @ Rz
        M = M @ self._t8
        return M[:3, :3], M[:3, 3].copy(), zs, ps

    def _ee_position(self, R8, p8, ee):
        if ee == "link8":
            return p8
        if ee == "tcp":
            return p8 + R8 @ self._tcp
        raise ValueError(f"frame sconosciuto: {ee!r} (ammessi {FRAMES})")

    def position(self, q, ee: str) -> np.ndarray:
        R8, p8, _, _ = self._chain(q)
        return self._ee_position(R8, p8, ee)

    def jacobian_v(self, q, ee: str) -> np.ndarray:
        R8, p8, zs, ps = self._chain(q)
        pe = self._ee_position(R8, p8, ee)
        return np.cross(zs, pe - ps).T  # 3x7, colonna i = z_i x (p_ee - p_i)

    @staticmethod
    def w_from_jv(jv: np.ndarray) -> float:
        # sqrt(det(Jv Jv^T)) == prodotto dei valori singolari di Jv (3x7)
        return math.sqrt(max(0.0, float(np.linalg.det(jv @ jv.T))))

    def w_trans(self, q, ee: str) -> float:
        return self.w_from_jv(self.jacobian_v(q, ee))

    def w_trans_both(self, q) -> tuple[float, float]:
        R8, p8, zs, ps = self._chain(q)
        return (self.w_from_jv(np.cross(zs, p8 - ps).T),
                self.w_from_jv(np.cross(zs, self._ee_position(R8, p8, "tcp") - ps).T))


# --------------------------------------------------------------------------------------
# Istanti
# --------------------------------------------------------------------------------------
def settle_index(p: np.ndarray, tol: float = TOL_SETTLE_M) -> int:
    """Primo indice k da cui tutte le righe sono entro tol dall'ultima riga."""
    p = _check_positions(p)
    bad = np.nonzero(np.linalg.norm(p - p[-1], axis=1) > tol)[0]
    return 0 if bad.size == 0 else int(bad[-1] + 1)


def step_index(p: np.ndarray, tol: float = TOL_STEP_M) -> int:
    """Massimo indice i con |p[i]-p[i-1]| >= tol (0 se non esiste)."""
    p = _check_positions(p)
    big = np.nonzero(np.linalg.norm(np.diff(p, axis=0), axis=1) >= tol)[0]
    return 0 if big.size == 0 else int(big[-1] + 1)


def _check_positions(p) -> np.ndarray:
    p = np.asarray(p, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or p.shape[0] < 2:
        raise ValueError(f"posizioni: attesa shape (n>=2, 3), ricevuta {p.shape}")
    return p


def paired_error_mm(a_t, a_p, t_t, t_p) -> np.ndarray:
    idx = np.clip(np.searchsorted(t_t, a_t, side="left"), 0, len(t_t) - 1)
    return np.linalg.norm(a_p - t_p[idx], axis=1) * 1000.0


def _index_at(t_axis: np.ndarray, t_value: float, what: str) -> int:
    j = int(np.searchsorted(t_axis, t_value, side="left"))
    if j >= len(t_axis):
        raise ValueError(f"{what}: t={t_value:.6f} oltre l'ultimo istante ({t_axis[-1]:.6f}); "
                         "nessun clip silenzioso")
    return j


# --------------------------------------------------------------------------------------
# Manifest
# --------------------------------------------------------------------------------------
def load_manifest(path) -> list[dict]:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"manifest non trovato: {path}")
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or doc.get("schema_version") != 1:
        raise ValueError("manifest: schema_version deve essere 1")
    for key in ("data_dir", "runs"):
        if key not in doc:
            raise ValueError(f"manifest: chiave obbligatoria mancante: {key}")
    data_dir = (path.parent / doc["data_dir"]).resolve()
    if not data_dir.is_dir():
        raise FileNotFoundError(f"data_dir non trovata: {data_dir}")
    runs = doc["runs"]
    if not isinstance(runs, list) or not runs:
        raise ValueError("manifest: 'runs' deve essere una lista non vuota")

    seen_ids: set[str] = set()
    seen_files: dict[Path, str] = {}
    out = []
    for e in runs:
        if not isinstance(e, dict):
            raise ValueError(f"manifest: voce non valida: {e!r}")
        missing = _ENTRY_REQUIRED - set(e)
        unknown = set(e) - _ENTRY_KEYS
        if missing:
            raise ValueError(f"run {e.get('run_id')!r}: chiavi mancanti {sorted(missing)}")
        if unknown:
            raise ValueError(f"run {e['run_id']!r}: chiavi sconosciute {sorted(unknown)}")
        rid = e["run_id"]
        if rid in seen_ids:
            raise ValueError(f"run_id duplicato: {rid}")
        seen_ids.add(rid)
        if e["family"] not in FAMILIES:
            raise ValueError(f"{rid}: family {e['family']!r} non in {FAMILIES}")
        if e["family"] == "regression_goal" and not e.get("group"):
            raise ValueError(f"{rid}: 'group' obbligatorio per family regression_goal")
        if e["kt_source"] not in KT_SOURCES:
            raise ValueError(f"{rid}: kt_source {e['kt_source']!r} non in {KT_SOURCES}")
        if (e["kt"] is None) != (e["kt_source"] in ("unknown", "not_applicable")):
            raise ValueError(f"{rid}: kt deve essere null se e solo se kt_source e' "
                             "unknown/not_applicable")
        if e["kt"] is not None and not isinstance(e["kt"], (int, float)):
            raise ValueError(f"{rid}: kt non numerico")
        if e["frame_at_recording"] not in FRAMES_AT_RECORDING:
            raise ValueError(f"{rid}: frame_at_recording {e['frame_at_recording']!r} non in "
                             f"{FRAMES_AT_RECORDING}")
        try:
            _dt.datetime.fromisoformat(str(e["recorded_utc"]))
        except ValueError as exc:
            raise ValueError(f"{rid}: recorded_utc non ISO-8601 ({exc})") from exc
        files = e["files"]
        if not isinstance(files, dict) or set(files) != {r for r, _ in FILE_ROLES}:
            raise ValueError(f"{rid}: 'files' deve avere esattamente le chiavi "
                             f"{[r for r, _ in FILE_ROLES]}")
        resolved = {}
        for role, prefix in FILE_ROLES:
            fpath = (data_dir / files[role]).resolve()
            if files[role] != f"{prefix}_{rid}.csv":
                raise ValueError(f"{rid}: file {role} = {files[role]!r}, atteso "
                                 f"'{prefix}_{rid}.csv'")
            if not fpath.is_file():
                raise FileNotFoundError(f"{rid}: file mancante ({role}): {fpath}")
            if fpath in seen_files:
                raise ValueError(f"file usato da due run_id: {fpath.name} "
                                 f"({seen_files[fpath]} e {rid})")
            seen_files[fpath] = rid
            resolved[role] = fpath
        entry = dict(e)
        entry["paths"] = resolved
        entry["bag_path"] = None
        if e.get("bag") is not None:
            bag_path = (path.parent / e["bag"]).resolve()
            if not (bag_path / "metadata.yaml").is_file():
                raise FileNotFoundError(f"{rid}: bag mancante (metadata.yaml assente): {bag_path}")
            entry["bag_path"] = bag_path
        out.append(entry)
    return out


# --------------------------------------------------------------------------------------
# Metriche per run
# --------------------------------------------------------------------------------------
def _read_csv(path: Path, columns: list[str], what: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    absent = [c for c in columns if c not in df.columns]
    if absent:
        raise ValueError(f"{path.name}: colonne mancanti {absent}")
    if len(df) < 2:
        raise ValueError(f"{path.name}: meno di 2 righe")
    df = df[columns]
    if df.isna().any().any():
        raise ValueError(f"{path.name}: valori NaN")
    if (np.diff(df["t"].to_numpy()) < 0).any():
        raise ValueError(f"{path.name}: colonna t non monotona ({what})")
    return df


def compute_run_metrics(kin: PandaKinematics, entry: dict, sim_time: bool = False) -> dict:
    P = entry["paths"]
    T = _read_csv(P["target_aligned"], ["t", "x", "y", "z"], "target")
    A = _read_csv(P["actual_pose"], ["t", "x", "y", "z"], "actual")
    JS = _read_csv(P["joint_states"], ["t"] + JOINT_NAMES, "joint_states")
    t_t, t_p = T["t"].to_numpy(), T[["x", "y", "z"]].to_numpy()
    a_t, a_p = A["t"].to_numpy(), A[["x", "y", "z"]].to_numpy()
    js_t, js_q = JS["t"].to_numpy(), JS[JOINT_NAMES].to_numpy()

    i_settle, i_step, i_last = settle_index(t_p), step_index(t_p), len(t_t) - 1
    t_settle, t_step, t_last = float(t_t[i_settle]), float(t_t[i_step]), float(t_t[i_last])

    err = paired_error_mm(a_t, a_p, t_t, t_p)
    cruise = (a_t >= CRUISE_START_S) & (a_t <= a_t[-1] - CRUISE_END_MARGIN_S)
    if not cruise.any():
        raise ValueError(f"{entry['run_id']}: finestra crociera [2 s, fine-3 s] vuota")

    row: dict = {
        "run_id": entry["run_id"], "family": entry["family"], "group": entry.get("group", ""),
        "kt": entry["kt"], "kt_source": entry["kt_source"],
        "recorded_utc": str(entry["recorded_utc"]), "frame_at_recording": entry["frame_at_recording"],
        "n_target": len(t_t), "n_actual": len(a_t), "n_joint_states": len(js_t),
        "idx_settle_1e4": i_settle, "idx_step_1e5": i_step,
        "t_settle_1e4_s": t_settle, "t_step_1e5_s": t_step, "t_last_s": t_last,
        "actual_t_last_s": float(a_t[-1]), "js_t_last_s": float(js_t[-1]),
        "axis_shift_aligned_s": float(js_t[-1] - t_t[-1]),
    }
    for name, idx, t_inst in (("t_settle_1e4", i_settle, t_settle), ("t_step_1e5", i_step, t_step)):
        j = _index_at(a_t, t_inst, f"{entry['run_id']} actual @ {name}")
        row[f"err_pos_paired_at_{name}_mm"] = float(err[j])
        row[f"err_pos_to_target_at_{name}_mm"] = float(np.linalg.norm(a_p[j] - t_p[idx]) * 1000.0)
    row["err_pos_mean_all_mm"] = float(err.mean())
    row["err_pos_mean_cruise_mm"] = float(err[cruise].mean())
    row["err_pos_max_mm"] = float(err.max())
    row["err_pos_last_mm"] = float(err[-1])
    for axis_name, val in zip("xyz", t_p[0]):
        row[f"target_first_{axis_name}_m"] = float(val)
    for axis_name, val in zip("xyz", t_p[-1]):
        row[f"target_final_{axis_name}_m"] = float(val)
    for axis_name, val in zip("xyz", a_p[0]):
        row[f"actual_first_{axis_name}_m"] = float(val)

    cache: dict[int, tuple[float, float]] = {}

    def w_row(i: int) -> tuple[float, float]:
        if i not in cache:
            cache[i] = kin.w_trans_both(js_q[i])
        return cache[i]

    shifts = {"as_scripts": 0.0, "aligned": float(js_t[-1] - t_t[-1])}
    for variant in AXIS_VARIANTS:
        tau_js = t_settle + shifts[variant]
        mask = np.nonzero((js_t >= tau_js - PRE_WINDOW_S) & (js_t <= tau_js))[0]
        if mask.size == 0:
            raise ValueError(f"{entry['run_id']}: finestra pre-t_settle [{tau_js - PRE_WINDOW_S:.3f}, "
                             f"{tau_js:.3f}] vuota sull'asse js ({variant})")
        j_settle = _index_at(js_t, tau_js, f"{entry['run_id']} js @ t_settle ({variant})")
        pre = np.array([w_row(int(i)) for i in mask])
        at = w_row(j_settle)
        last = w_row(len(js_t) - 1)
        for k, frame in enumerate(FRAMES):
            row[f"w_{frame}_pre1s_mean_{variant}"] = float(pre[:, k].mean())
            row[f"w_{frame}_at_t_settle_{variant}"] = float(at[k])
            row[f"w_{frame}_last_row_{variant}"] = float(last[k])
        row[f"n_js_rows_pre1s_{variant}"] = int(mask.size)
    if sim_time:
        row.update(_sim_time_metrics(entry, t_t, t_p, a_t, a_p, js_t, i_settle, w_row))
    return row


def _stamps_s(bag, role: str, rid: str, require_monotonic: bool = True) -> np.ndarray:
    if role not in bag:
        raise ValueError(f"{rid}: topic {role} assente nel bag")
    if not bag[role].stamp_verified:
        raise ValueError(f"{rid}: header.stamp di {role} NON VERIFICATO ({bag[role].note})")
    st = bag[role].stamp_ns
    n_bad = int((np.diff(st) < 0).sum())
    if require_monotonic and n_bad:
        raise ValueError(f"{rid}: {n_bad} stamp non monotoni in {role}")
    return st * 1e-9


EPOCH_LIKE_S = 1e8          # stamp >= 1e8 s: non e' un tempo di simulazione
LAG_TOL_S = 0.05            # "sotto il massimo corrente" di piu' di 50 ms


def _sim_time_metrics(entry, t_t, t_p, a_t, a_p, js_t, i_settle, w_row) -> dict:
    """Variante sim_time: vedi docstring del modulo."""
    rid = entry["run_id"]
    if entry.get("bag_path") is None:
        raise ValueError(f"{rid}: la variante sim_time richiede la chiave 'bag' nel manifest")
    bag = btp.read_bag(entry["bag_path"])
    btp.verify_csv_alignment(bag, {"target_aligned": t_t, "actual_pose": a_t, "joint_states": js_t})
    st_t = _stamps_s(bag, "target_pose_aligned", rid)
    st_a = _stamps_s(bag, "actual_pose", rid)
    # joint_states: gli stamp NON sono monotoni nei bag esistenti (sequenze mescolate nello stesso topic).
    # Le definizioni sotto non richiedono monotonia; le anomalie sono contate in colonne dedicate.
    st_j = _stamps_s(bag, "joint_states", rid, require_monotonic=False)
    tau = float(st_t[i_settle])
    j_act = _index_at(st_a, tau, f"{rid} actual @ t_settle (sim)")
    k_tgt = min(int(np.searchsorted(st_t, st_a[j_act], side="left")), len(st_t) - 1)
    out = {
        "t_settle_1e4_sim_s": tau - float(st_t[0]),
        "t_settle_1e4_sim_abs_s": tau,
        "err_pos_paired_at_t_settle_1e4_sim_mm": float(np.linalg.norm(a_p[j_act] - t_p[k_tgt]) * 1000.0),
        Y_COL_SIM: float(np.linalg.norm(a_p[j_act] - t_p[i_settle]) * 1000.0),
        "sim_js_minus_target_last_s": float(st_j[-1] - st_t[-1]),
        "sim_js_max_stamp_minus_target_last_s": float(st_j[st_j < EPOCH_LIKE_S].max() - st_t[-1]),
        "sim_js_minus_target_first_s": float(st_j[0] - st_t[0]),
        "js_stamp_n_non_monotonic": int((np.diff(st_j) < 0).sum()),
        "js_stamp_n_below_running_max_gt_0p05s": int(((np.maximum.accumulate(st_j) - st_j) > LAG_TOL_S).sum()),
        "js_stamp_n_epoch_like": int((st_j >= EPOCH_LIKE_S).sum()),
        "js_stamp_last_row_is_epoch_like": bool(st_j[-1] >= EPOCH_LIKE_S),
        "rtf_target_overall": float((st_t[-1] - st_t[0]) / ((bag["target_pose_aligned"].recv_ns[-1]
                                                              - bag["target_pose_aligned"].recv_ns[0]) * 1e-9)),
    }
    if out["js_stamp_last_row_is_epoch_like"]:
        raise ValueError(f"{rid}: l'ultima riga di joint_states ha stamp epoch-like: NON VERIFICATO")
    # at_t_settle: tra le righe js con stamp >= t_settle_sim, quella con lo stamp MINIMO (a parita' la
    # prima in ordine di ricezione). Coincide con "prima riga con stamp >= tau" se gli stamp sono
    # monotoni; non dipende dall'ordine se non lo sono.
    cand = np.nonzero(st_j >= tau)[0]
    if cand.size == 0:
        raise ValueError(f"{rid}: nessuna riga js con stamp >= t_settle_sim {tau:.3f}")
    j_js = int(cand[np.argmin(st_j[cand])])
    at, last = w_row(j_js), w_row(len(st_j) - 1)
    for width in SIM_WINDOWS_S:
        if tau - width < st_j.min():
            raise ValueError(f"{rid}: finestra {width:g} s sim esce dall'inizio di joint_states")
        mask = np.nonzero((st_j >= tau - width) & (st_j <= tau))[0]
        if mask.size == 0:
            raise ValueError(f"{rid}: finestra {width:g} s sim vuota su joint_states")
        pre = np.array([w_row(int(i)) for i in mask])
        for k, frame in enumerate(FRAMES):
            out[f"w_{frame}_pre{width:g}s_mean_{SIM_VARIANT}"] = float(pre[:, k].mean())
        out[f"n_js_rows_pre{width:g}s_{SIM_VARIANT}"] = int(mask.size)
    for k, frame in enumerate(FRAMES):
        out[f"w_{frame}_at_t_settle_{SIM_VARIANT}"] = float(at[k])
        out[f"w_{frame}_last_row_{SIM_VARIANT}"] = float(last[k])
    return out


# --------------------------------------------------------------------------------------
# Regressione
# --------------------------------------------------------------------------------------
def linfit(x, y) -> tuple[float, float, float]:
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3:
        raise ValueError("regressione: servono almeno 3 punti")
    slope, intercept = np.polyfit(x, y, 1)
    r = float(np.corrcoef(x, y)[0, 1])
    return float(intercept), float(slope), r


def logo_folds(groups) -> dict:
    """Fold leave-one-GOAL-out: {gruppo: indici dei run esclusi insieme}."""
    groups = np.asarray(groups)
    return {str(g): np.nonzero(groups == g)[0] for g in np.unique(groups)}


def r2_leave_one_group_out(x, y, groups) -> float:
    x, y, groups = np.asarray(x, float), np.asarray(y, float), np.asarray(groups)
    pred = np.empty_like(y)
    for g in np.unique(groups):
        held = groups == g
        if (~held).sum() < 3:
            raise ValueError(f"LOGO: meno di 3 punti rimasti tolto il gruppo {g}")
        b0, b1, _ = linfit(x[~held], y[~held])
        pred[held] = b0 + b1 * x[held]
    return float(1.0 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2))


def regression_table(df: pd.DataFrame, y_col: str = Y_COL) -> pd.DataFrame:
    reg = df[df["family"] == "regression_goal"]
    if reg.empty:
        raise ValueError("nessun run con family == regression_goal")
    specs = [(v, W_INSTANTS, y_col) for v in AXIS_VARIANTS]
    if f"w_link8_pre1s_mean_{SIM_VARIANT}" in df.columns:
        specs.append((SIM_VARIANT, SIM_W_INSTANTS, Y_COL_SIM))
    rows = []
    for excluded in ("none", "Goal 2"):
        sub = reg if excluded == "none" else reg[reg["group"] != excluded]
        for variant, instants, ycol in specs:
            for frame in FRAMES:
                for instant in instants:
                    col = f"w_{frame}_{instant}_{variant}"
                    x, y = sub[col].to_numpy(), sub[ycol].to_numpy()
                    b0, b1, r = linfit(x, y)
                    rows.append({"excluded_group": excluded, "axis_variant": variant,
                                 "frame": frame, "w_instant": instant, "y_col": ycol, "n": len(sub),
                                 "intercept": b0, "slope": b1, "r": r, "r2": r * r,
                                 "r2_logo": r2_leave_one_group_out(x, y, sub["group"].to_numpy())})
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------------------
# Esecuzione completa
# --------------------------------------------------------------------------------------
def compute_all(manifest_path=DEFAULT_MANIFEST, kin: PandaKinematics | None = None,
                sim_time: bool = True) -> pd.DataFrame:
    kin = kin or PandaKinematics()
    entries = load_manifest(manifest_path)
    return pd.DataFrame([compute_run_metrics(kin, e, sim_time=sim_time) for e in entries])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args(argv)
    out = Path(args.out)
    df = compute_all(args.manifest)
    (out / "per_run").mkdir(parents=True, exist_ok=True)
    for _, r in df.iterrows():
        r.to_frame().T.to_csv(out / "per_run" / f"{r['run_id']}.csv", index=False)
    df.to_csv(out / "all_runs.csv", index=False)
    tab = regression_table(df)
    tab.to_csv(out / "regression_table.csv", index=False)
    with pd.option_context("display.width", 200, "display.max_rows", 200,
                           "display.float_format", "{:.4f}".format):
        print(tab.to_string(index=False))
    print(f"\nscritti: {out}/per_run/*.csv, {out}/all_runs.csv, {out}/regression_table.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
