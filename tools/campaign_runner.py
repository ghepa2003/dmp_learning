#!/usr/bin/env python3
"""
tools/campaign_runner.py - lancia in Gazebo, UNA ALLA VOLTA, le corse della campagna (tools/s1_run.py), una per braccio necessario.

    python3 tools/campaign_runner.py --manifest manifest.csv --dryrun dryrun.csv --demo-map demos.yaml \
        --campaign-tag camp1 [--outdir-root s1_runs/camp1] [--resume] [--print] [--limit N] [--stop-file PATH]

Da lanciare nel container con ROS (come s1_run.py). Per ogni scenario del manifesto (ordine del manifesto) legge run_needed dal CSV
del dry-run (tools/campaign_dryrun.py): "A" = UNA corsa condivisa, braccio A; "B" = una corsa, braccio B; "both" = due corse (A poi B);
"none"/"error" = nessuna corsa (stato NON_ESEGUITA, motivo dal dry-run: il runner non esclude scenari di sua iniziativa).
NON_ESEGUITA vale anche per CONFIG e STOP (vedi sotto). Un rifiuto del selettore A RUNTIME (il planner non trova candidati) NON e'
NON_ESEGUITA: ha lo stato proprio RIFIUTO_SELETTORE.
--arms dryrun (default): come sopra. --arms both: A e B per OGNI scenario, ignorando run_needed; un braccio senza vincitore (o senza yaw)
nel dry-run non si puo' lanciare (manca la fase di spawn): resta una riga NON_ESEGUITA per quel braccio, con il motivo.
--arms A: solo il braccio A per OGNI scenario, ignorando run_needed; uno scenario senza vincitore (o senza yaw) A nel dry-run resta una
riga NON_ESEGUITA per il braccio A, con il motivo.
Il nodo planner sceglie da solo (nessun candidato forzato). Il profilo di corsa e' RUN_PROFILE (stesso di results/val_C0, vedi sotto).

--demo-map: file con righe `demo_id: cartella_dei_pesi` (demo_id = nome della cartella nella colonna `demo` del manifesto).
--resume: salta le corse (scenario, braccio) gia' presenti nel CSV con stato OK, FALLITA, RIFIUTO_SELETTORE o STARTUP_FAIL_RIPETUTA.
--print: stampa i comandi senza lanciare niente (nessun controllo dei processi). --limit N: al piu' N corse (una per braccio).
--stop-file: se esiste prima di iniziare una corsa, il runner si ferma (la corsa in corso finisce).

CSV delle corse (--status-csv, default <outdir-root>/campaign_runs.csv): vedi COLUMNS. Il vincitore scelto dal nodo e' letto da
report.json["selection"] (k, wrist_flip, theta_star_deg), che s1_report ricava dal GraspCommand pubblicato (s1_report.py:728-731); la fase
al trigger da report.json["launch"] (yaw_trig_exp_deg atteso, yaw_trig_meas_deg misurato dall'odometria: s1_report.py:738); i criteri 1-11
da report.json["criteria"].

Guasti di avvio. s1_run esce con 1 (fallito), 3 (processi residui dopo il run) o 130 (interrotto) e scrive UNA riga "S1_RUN FAIL: <motivo>":
nessun codice distingue il guasto di avvio. Con 4 (EXIT_SELECTOR_REFUSAL) esce per il rifiuto del selettore (riga "S1_RUN RIFIUTO_SELETTORE: ...",
run_meta.json con outcome="RIFIUTO_SELETTORE"). Il runner usa la tabella di classify_failure():
  RIFIUTO_SELETTORE  rc 4: il planner non ha trovato candidati ammissibili (GraspCommand status != SELECTED). Esito corretto e distinto, NON un guasto:
           niente ripetizione, mai STARTUP; riga di CSV con stato RIFIUTO_SELETTORE, motivo, selection_elapsed_s, selector_status e rifiuto_nota
           (letti da run_meta.json); nessun report;
  CONFIG   errore deterministico dei parametri / dell'ambiente (messaggi prima di qualunque processo): NON_ESEGUITA, nessuna ripetizione;
  STARTUP  guasto prima del TRIGGER di presa (nessuna riga "continuous TRIGGER at node clock" in executor.log, s1_report.py:366) per i
           messaggi della fase di avvio, e per i messaggi "ambigui" se il trigger NON c'e' stato: si ripete UNA volta;
  FALLITA  guasto dopo il TRIGGER: e' un risultato della corsa (non si ripete, resta registrato con il report se esiste);
  STOP     processi residui (prima/dopo il run) o corsa interrotta: il runner si ferma.
Il primo tentativo con guasto di avvio si rinomina <tag>_attempt1_STARTUP_FAIL (mai sovrascrivere: se il nome esiste si aggiunge _2, _3...);
se anche il secondo fallisce lo stato e' STARTUP_FAIL_RIPETUTA (motivo = secondo messaggio).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS_DIR = Path(os.environ.get("THESIS_WS", HERE.parent))
S1_RUN = HERE / "s1_run.py"

# Profilo validato delle celle (results/val_C0/run_meta.json: kt 2000.0, depth 0.0125, follow_s 3.0, hold_s 120.0, squeeze -0.05,
# stop_cube true, stop_cube_delay_s 0.0, post_s 45.0, sim_timeout 500.0; comando: docs/RUNBOOK_SIMULAZIONE.md:236-238; opzioni:
# s1_run.py:1105-1117 e :1162). Passati tutti esplicitamente: non dipende dai default di s1_run.
RUN_PROFILE = ["--kt", "2000", "--depth", "0.0125", "--follow-s", "3.0", "--hold-s", "120.0", "--squeeze", "-0.05",
               "--stop-cube", "--stop-cube-delay-s", "0.0", "--post-s", "45.0", "--sim-timeout", "500.0"]
RUN_PROFILE_STR = " ".join(RUN_PROFILE)

EXIT_SELECTOR_REFUSAL = 4   # = s1_run.EXIT_SELECTOR_REFUSAL (copiato: s1_run importa numpy/s1_report; i test li confrontano)
STATUS_REFUSAL = "RIFIUTO_SELETTORE"
# ASSUNTO (non misurato): scarto attesa-pubblicazione->TRIGGER oltre il quale si segna lost_phase_turn=1, cioe' il
# trigger e' avvenuto piu' tardi di quanto stimato dal planner (wait_s): il candidato non era pronto prima dell'incrocio
# di fase previsto e il TRIGGER aspetta un giro del satellite. wait_s e' una stima diagnostica (GraspCommand.msg), wait_s_actual
# e' misurato in tempo simulato: la soglia 1.0 s e' una scelta, da ricalibrare sui primi run.
LOST_PHASE_TURN_THRESHOLD_S = 1.0

# Tempi della selezione per stadio [ms, wall-clock], dalla riga "s1 timing:" del planner.log (via report.json["selection"] o, per un
# rifiuto, dal planner.log). Ogni stadio e' la SOMMA sui finalisti esaminati (il flusso pigro intercala gli stadi); other_ms = totale -
# somma degli stadi. Vuote se la riga manca (log vecchi). Stessi nomi di s1_report.TIMING_KEYS.
TIMING_COLUMNS = ["scan_ms", "rollout_ms", "clearance_ms", "hold_end_ms", "dynamic_ms", "posthoc_ms", "other_ms"]

CRITERIA_KEYS = ["f_sum_plateau_10_20s", "min_f_plateau_10_20s", "slip_tang_axial_3_15s", "pad_straddle_2s", "pad_straddle_5s",
                 "pad_straddle_15s", "cube_disp_pre_end_mm", "gap_end_mean_last10s", "pad_straddle_end", "slip_tang_axial_3_end",
                 "min_joint_margin_deg"]   # results/val_C0/report.json["criteria"]
COLUMNS = (["scenario_id", "arm", "tag", "run_needed", "predicted_k", "predicted_branch", "predicted_theta_deg", "chosen_k",
            "chosen_branch", "chosen_theta_deg", "same_winner", "spawn_yaw_deg", "yaw_trig_exp_deg", "yaw_trig_meas_deg",
            "yaw_trig_err_deg", "wait_s", "wait_actual_s", "wait_err_s", "lost_phase_turn", "selection_elapsed_s",
            *TIMING_COLUMNS,
            "success", "criterion_11_ok", "success_1_11", "min_joint_margin_meas_deg", "min_joint_margin_pred_deg"] +
           [f"crit_{k}" for k in CRITERIA_KEYS] +
           ["status", "motivo", "selector_status", "rifiuto_nota", "attempts", "wall_s", "outdir",
            "loadavg_1m_start", "loadavg_5m_start", "loadavg_15m_start"])
# success = criteri 1-10 (esito principale); criterion_11_ok = margine di giunto misurato >= soglia, a parte;
# success_1_11 = tutti e 11. min_joint_margin_meas_deg = grasp.min_joint_margin_deg (misurato), _pred_ = selection.min_joint_margin_deg (previsto).
DONE_STATES = ("OK", "FALLITA", STATUS_REFUSAL, "STARTUP_FAIL_RIPETUTA")
THETA_TOL_DEG = 0.06
TRIGGER_RE = re.compile(r"continuous TRIGGER at node clock t=([0-9.]+) s")   # s1_report.py:366
RESIDUE_RE = re.compile(r"\b(ros2|gz|gzserver|gzclient|colcon|grasp_planner|prodmp|ign gazebo|parameter_bridge|"
                        r"robot_state_publisher|controller_manager|ros_gz)")
ROS2_DAEMON_MARK = "ros2cli.daemon"   # demone ros2 avviato da `ros2 pkg prefix` ecc.: infrastruttura persistente di ROS 2, non un residuo della corsa
CONCLUDED_CLASSES = ("OK", "FALLITA", STATUS_REFUSAL)   # corsa arrivata a un esito: dei residui dopo la corsa si annota solo il motivo dello STOP
EXIT_INTERRUPTED = 130   # s1_run.py:867

# ---- classificazione dei messaggi "S1_RUN FAIL:" di s1_run.py (riga = punto in cui e' sollevato) ----
# CONFIG: 155 omega non coerente; 281/283 --axis-tilt/lean fuori intervallo; 298, 345-355 coerenza asse/normale/q_ref; 396, 405, 415, 417
# --arm; 566 --planner-param ripetuto; 795 parametri di tilt scritti dal runner; 1098 file dei pesi mancante; 1184-1201 argomenti
# (omega, stop-cube, post-s, stop-cube-delay); 1208 bag gia' esistente; 123/130 translational_stiffness non trovato nel YAML dei guadagni.
CONFIG_PREFIXES = (
    "omega non coerente", "--axis-tilt-deg", "--axis-lean-deg", "normale del collare", "asse non coerente", "q_ref non coerente",
    "face_normal_body del planner", "|n.a|", "--arm", "--planner-param", "con --axis-tilt-deg > 0", "file mancante",
    "--omega-deg-s", "--stop-cube richiede", "--post-s", "--stop-cube-delay-s", "translational_stiffness non trovato",
)
CONFIG_CONTAINS = ("esiste gia' (usa --overwrite",)
# STARTUP sempre (fase prima del planner: 461 residui "prima del run", 526-542 preflight, 925 bringup, 932 controller, 948 spawn,
# 960 topic cmd_vel, 975/977 executor, 984 executor non pronto). "processi residui prima del run" e' STOP (vedi sotto).
STARTUP_PREFIXES = (
    "preflight:", "executor senza follow limitato", "bringup terminato", "controller non attivo", "spawn del cubo non confermato",
    "--stop-cube: nessun topic", "executor terminato prematuramente", "eccezione nel log dell'executor", "executor non pronto",
    "/clock non pubblicato",   # 995
)
# AMBIGUI (decide il trigger): 1003 timeout simulato senza AT_END; 1006 /clock fermo; 1009 executor terminato prima di AT_END; 710 goal
# 3D mancante; 1028 gate goal fallito; 1044 gate goal mai eseguito. Con il trigger => FALLITA; senza => STARTUP.
# POST-trigger per costruzione (FALLITA): 1050 /clock fermo post AT_END; 878/880 bag non chiuso; "report:" (1246).
POST_PREFIXES = ("/clock fermo durante la registrazione", "bag non chiuso", "bag chiuso ma", "report:")


def classify_failure(rc: int, msg: str, trigger_seen: bool):
    """(classe, motivo). classe in OK, CONFIG, STARTUP, FALLITA, STOP, RIFIUTO_SELETTORE. msg = testo dopo "S1_RUN FAIL: " (o vuoto).
    Il rifiuto del selettore e' riconosciuto dal solo codice di uscita (4, distinto da 3 e 130 di STOP) e prima di ogni
    regola sul messaggio: mai STARTUP. I residui dopo il run (main) lo sovrascrivono con STOP, come per ogni classe."""
    if rc == 0:
        return "OK", ""
    if rc == EXIT_SELECTOR_REFUSAL:
        return STATUS_REFUSAL, msg or "rifiuto del selettore (codice 4)"
    if rc == EXIT_INTERRUPTED:
        return "STOP", "corsa interrotta (codice 130)"
    if rc == 3 or "processi residui dopo il run" in msg:
        return "STOP", msg or "processi residui dopo il run (codice 3)"
    if "processi residui prima del run" in msg:
        return "STOP", msg
    if msg.startswith(CONFIG_PREFIXES) or any(c in msg for c in CONFIG_CONTAINS):
        return "CONFIG", msg
    if msg.startswith(POST_PREFIXES):
        return "FALLITA", msg
    if msg.startswith(STARTUP_PREFIXES):
        return "STARTUP", msg
    # ambigui e messaggi sconosciuti: decide il trigger
    return ("FALLITA" if trigger_seen else "STARTUP"), (msg or f"codice di uscita {rc} senza riga S1_RUN FAIL")


def last_fail_message(stderr_text: str) -> str:
    msg = ""
    for line in stderr_text.splitlines():
        if line.startswith("S1_RUN FAIL:"):
            msg = line[len("S1_RUN FAIL:"):].strip()
    return msg


def trigger_seen(outdir: Path) -> bool:
    p = outdir / "executor.log"
    return p.is_file() and TRIGGER_RE.search(p.read_text(errors="replace")) is not None


def residual_processes() -> list:
    """Righe `ps` dei processi che corrispondono a RESIDUE_RE, escluso il runner stesso (e i suoi genitori)."""
    out = subprocess.run(["ps", "-eo", "pid=,ppid=,args="], capture_output=True, text=True).stdout
    procs = {}
    for ln in out.splitlines():
        parts = ln.split(None, 2)
        if len(parts) == 3 and parts[0].isdigit() and parts[1].isdigit():
            procs[int(parts[0])] = (int(parts[1]), parts[2])
    mine, pid = set(), os.getpid()
    while pid in procs and pid not in mine:
        mine.add(pid)
        pid = procs[pid][0]
    return [f"{p} {a}" for p, (_pp, a) in sorted(procs.items())
            if p not in mine and RESIDUE_RE.search(a) and not a.startswith("ps ") and "campaign_runner.py" not in a
            and ROS2_DAEMON_MARK not in a]


def read_csv(path) -> list:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def read_demo_map(path) -> dict:
    m = {}
    for line in Path(path).read_text().splitlines():
        body = line.split("#", 1)[0].strip()
        k, sep, v = body.partition(":")
        if sep and k.strip() and v.strip():
            m[k.strip()] = v.strip().strip("'\"")
    return m


def demo_id(demo: str) -> str:
    return Path(demo.rstrip("/")).name


def plan_runs(manifest: list, dryrun: list, demo_map: dict, arms: str = "dryrun") -> list:
    """Corse nell'ordine del manifesto: dict(scenario, arm, run_needed, predicted, spawn_yaw, weights_dir). Errore se manca qualcosa.
    arms = "dryrun": i bracci indicati da run_needed; "both": A e B per ogni scenario, run_needed ignorato (un braccio senza vincitore
    o yaw nel dry-run diventa una voce con "skip"); "A": solo il braccio A per ogni scenario, run_needed ignorato (stesso "skip" se A
    non ha vincitore o yaw)."""
    if arms not in ("dryrun", "both", "A"):
        raise SystemExit(f"--arms '{arms}' sconosciuto (dryrun | both | A)")
    dr = {r["scenario_id"]: r for r in dryrun}
    runs = []
    for sc in manifest:
        d = dr.get(sc["scenario_id"])
        if d is None:
            raise SystemExit(f"{sc['scenario_id']}: assente dal CSV del dry-run")
        for k in ("omega_deg_s", "alpha_deg", "phi_deg", "cx", "cy", "cz", "demo"):
            if sc[k] != d[k]:
                raise SystemExit(f"{sc['scenario_id']}: {k} differisce tra manifesto ({sc[k]}) e dry-run ({d[k]})")
        need = d["run_needed"]
        wd = None
        if arms == "dryrun":
            if need in ("none", "error", ""):
                runs.append({"scenario": sc, "arm": "", "run_needed": need, "skip": f"dry-run: run_needed={need or 'vuoto'}"})
                continue
            arm_list = {"A": ["A"], "B": ["B"], "both": ["A", "B"]}.get(need)
            if arm_list is None:
                raise SystemExit(f"{sc['scenario_id']}: run_needed '{need}' sconosciuto")
            wd = demo_map.get(demo_id(sc["demo"]))
            if wd is None:
                raise SystemExit(f"{sc['scenario_id']}: demo '{demo_id(sc['demo'])}' non in --demo-map")
        else:
            arm_list = ["A"] if arms == "A" else ["A", "B"]
        for arm in arm_list:
            pred = (d[f"winner_{arm}_k"], d[f"winner_{arm}_branch"], d[f"winner_{arm}_theta_deg"])
            yaw = d[f"spawn_yaw_deg_{arm}"]
            if "" in pred or yaw == "":
                if arms in ("both", "A"):
                    runs.append({"scenario": sc, "arm": arm, "run_needed": need,
                                 "skip": f"dry-run: nessun vincitore o yaw per il braccio {arm} (run_needed={need or 'vuoto'})"})
                    continue
                raise SystemExit(f"{sc['scenario_id']}: vincitore o yaw del braccio {arm} vuoti nel dry-run (run_needed={need})")
            if wd is None:
                wd = demo_map.get(demo_id(sc["demo"]))
                if wd is None:
                    raise SystemExit(f"{sc['scenario_id']}: demo '{demo_id(sc['demo'])}' non in --demo-map")
            runs.append({"scenario": sc, "arm": arm, "run_needed": need, "predicted": pred, "spawn_yaw": yaw, "weights_dir": wd})
    return runs


def build_cmd(run: dict, tag: str, outdir: Path) -> list:
    sc = run["scenario"]
    return [sys.executable, str(S1_RUN), "--weights-dir", run["weights_dir"], "--tag", tag, "--outdir", str(outdir),
            "--arm", run["arm"], *RUN_PROFILE, "--spawn-yaw-deg", run["spawn_yaw"], "--omega-deg-s", sc["omega_deg_s"],
            "--center-xyz", f"{sc['cx']},{sc['cy']},{sc['cz']}", "--axis-tilt-deg", sc["alpha_deg"], "--axis-lean-deg", sc["phi_deg"]]


def unique_name(path: Path) -> Path:
    if not path.exists():
        return path
    i = 2
    while Path(f"{path}_{i}").exists():
        i += 1
    return Path(f"{path}_{i}")


def run_once(cmd, logbase: Path, timeout_s: float):
    """Lancia s1_run, scrive stdout/stderr in <logbase>.out/.err. Ritorna (rc, stderr, timed_out)."""
    with open(f"{logbase}.out", "w") as fo, open(f"{logbase}.err", "w") as fe:
        p = subprocess.Popen(cmd, stdout=fo, stderr=fe)
        try:
            rc = p.wait(timeout=timeout_s)
            timed_out = False
        except subprocess.TimeoutExpired:
            timed_out = True
            p.send_signal(signal.SIGINT)   # s1_run chiude il bag e ripulisce (KeyboardInterrupt)
            try:
                rc = p.wait(timeout=180)
            except subprocess.TimeoutExpired:
                p.terminate()
                rc = p.wait()
    return rc, Path(f"{logbase}.err").read_text(errors="replace"), timed_out


def read_report(outdir: Path):
    p = outdir / "report.json"
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text())
    except ValueError:
        return None


def read_run_meta(outdir: Path):
    p = outdir / "run_meta.json"
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text())
    except ValueError:
        return None


def wait_columns(wait_s, wait_actual_s) -> dict:
    """wait_err_s = wait_s_actual - wait_s; lost_phase_turn = 1 se wait_err_s > LOST_PHASE_TURN_THRESHOLD_S (ASSUNTO).
    Valori mancanti o non finiti: celle vuote."""
    try:
        err = float(wait_actual_s) - float(wait_s)
    except (TypeError, ValueError):
        return {}
    if err != err or err in (float("inf"), float("-inf")):
        return {}
    return {"wait_err_s": f"{err:.4f}", "lost_phase_turn": int(err > LOST_PHASE_TURN_THRESHOLD_S)}


def timing_cells(sel: dict) -> dict:
    """Celle TIMING_COLUMNS da un dict con le chiavi di s1_report.TIMING_KEYS (None/assente -> cella vuota)."""
    return {k: f"{sel[k]:.3f}" for k in TIMING_COLUMNS if sel.get(k) is not None}


def planner_log_timing_cells(outdir: Path) -> dict:
    """Celle TIMING_COLUMNS dal planner.log di una corsa senza report (rifiuto del selettore). {} se manca la riga."""
    import s1_report   # import locale: numpy serve solo qui
    return timing_cells(s1_report.parse_selection_timing(outdir / "planner.log"))


def refusal_fields(meta: dict | None) -> dict:
    """Celle della riga CSV per un rifiuto del selettore, da run_meta.json (chiavi scritte da s1_run.selector_refusal_meta)."""
    meta = meta or {}
    f = {"selector_status": meta.get("selector_status_name", ""), "rifiuto_nota": meta.get("selector_note", "")}
    if meta.get("selection_elapsed_s") is not None:
        f["selection_elapsed_s"] = f"{meta['selection_elapsed_s']:.4f}"
    return f


def refusal_motivo(meta: dict | None, fallback: str) -> str:
    meta = meta or {}
    if "selector_status" not in meta:
        return fallback
    return f"{meta.get('selector_status_name', '')} (status={meta['selector_status']}), nessun candidato ammissibile"


def fill_from_report(row: dict, rep: dict, pred):
    sel, la, crit = rep.get("selection", {}), rep.get("launch", {}), rep.get("criteria", {})
    g = rep.get("grasp", {})
    if sel.get("selection_elapsed_s") is not None:
        row["selection_elapsed_s"] = f"{sel['selection_elapsed_s']:.4f}"
    row.update(timing_cells(sel))
    if sel.get("provisional_delay_s") is not None:
        row["wait_s"] = f"{sel['provisional_delay_s']:.4f}"
    row.update(wait_columns(sel.get("provisional_delay_s"), la.get("wait_actual_s")))
    if g.get("min_joint_margin_deg") is not None:
        row["min_joint_margin_meas_deg"] = f"{g['min_joint_margin_deg']:.4f}"
    if sel.get("min_joint_margin_deg") is not None:
        row["min_joint_margin_pred_deg"] = f"{sel['min_joint_margin_deg']:.4f}"
    if sel.get("k") is not None:
        row["chosen_k"] = sel["k"]
        row["chosen_branch"] = "B" if sel.get("wrist_flip") else "A"
        row["chosen_theta_deg"] = f"{sel['theta_star_deg']:.4f}" if sel.get("theta_star_deg") is not None else ""
        try:
            row["same_winner"] = int(str(sel["k"]) == str(pred[0]) and row["chosen_branch"] == pred[1]
                                     and abs(float(sel["theta_star_deg"]) - float(pred[2])) <= THETA_TOL_DEG)
        except (TypeError, ValueError):
            row["same_winner"] = ""
    for key, col in (("yaw_trig_exp_deg", "yaw_trig_exp_deg"), ("yaw_trig_meas_deg", "yaw_trig_meas_deg"),
                     ("yaw_trig_err_deg", "yaw_trig_err_deg"), ("wait_actual_s", "wait_actual_s")):
        if la.get(key) is not None:
            row[col] = f"{la[key]:.4f}"
    for k in CRITERIA_KEYS:
        if k in crit:
            row[f"crit_{k}"] = int(bool(crit[k].get("ok")))
    if "success" in rep:
        row["success"] = int(bool(rep["success"]))   # criteri 1-10
    if rep.get("criterion_11_ok") is not None:
        row["criterion_11_ok"] = int(bool(rep["criterion_11_ok"]))
    if "success_1_11" in rep:
        row["success_1_11"] = int(bool(rep["success_1_11"]))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--dryrun", required=True, help="CSV di campaign_dryrun.py")
    ap.add_argument("--demo-map", required=True)
    ap.add_argument("--campaign-tag", required=True, help="prefisso dei tag: <campaign-tag>_<scenario_id>_<braccio>")
    ap.add_argument("--outdir-root", default=None, help="default: s1_runs/<campaign-tag>")
    ap.add_argument("--status-csv", default=None, help="default: <outdir-root>/campaign_runs.csv")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--print", dest="print_only", action="store_true", help="stampa i comandi, non lancia niente")
    ap.add_argument("--limit", type=int, default=None, help="al piu' N corse (una per braccio)")
    ap.add_argument("--stop-file", default=None)
    ap.add_argument("--arms", choices=("dryrun", "both", "A"), default="dryrun",
                    help="dryrun (default): i bracci indicati da run_needed del dry-run; both: A e B per OGNI scenario, run_needed ignorato; "
                         "A: solo il braccio A per OGNI scenario, run_needed ignorato")
    ap.add_argument("--run-timeout-s", type=float, default=3600.0, help="timeout wall di una corsa (3600)")
    a = ap.parse_args(argv)

    manifest, dryrun = read_csv(a.manifest), read_csv(a.dryrun)
    info = Path(str(a.dryrun) + ".json")
    if info.is_file():
        import hashlib
        want = json.loads(info.read_text()).get("manifest_sha256")
        got = hashlib.sha256(Path(a.manifest).read_bytes()).hexdigest()
        if want and want != got:
            raise SystemExit(f"il manifesto ({got}) non e' quello del dry-run ({want})")
    runs = plan_runs(manifest, dryrun, read_demo_map(a.demo_map), a.arms)
    outroot = Path(a.outdir_root) if a.outdir_root else WS_DIR / "s1_runs" / a.campaign_tag
    status_csv = Path(a.status_csv) if a.status_csv else outroot / "campaign_runs.csv"
    done = set()
    if a.resume and status_csv.is_file():
        done = {(r["scenario_id"], r["arm"]) for r in read_csv(status_csv) if r["status"] in DONE_STATES}

    if not a.print_only:
        outroot.mkdir(parents=True, exist_ok=True)
        new_file = not (a.resume and status_csv.is_file())
        fo = open(status_csv, "w" if new_file else "a", newline="")
        w = csv.DictWriter(fo, fieldnames=COLUMNS, lineterminator="\n")
        if new_file:
            w.writeheader()
    n_launched = 0
    try:
        for run in runs:
            sc, arm = run["scenario"], run["arm"]
            sid = sc["scenario_id"]
            if (sid, arm) in done:
                continue
            row = {c: "" for c in COLUMNS}
            row.update(scenario_id=sid, arm=arm, run_needed=run["run_needed"])
            if run.get("skip"):
                row.update(status="NON_ESEGUITA", motivo=run["skip"], attempts=0)
                if a.print_only:
                    print(f"# {sid}: {run['skip']}")
                else:
                    w.writerow(row)
                    fo.flush()
                continue
            if a.limit is not None and n_launched >= a.limit:
                break
            tag = f"{a.campaign_tag}_{sid}_{arm}"
            outdir = outroot / tag
            cmd = build_cmd(run, tag, outdir)
            if a.print_only:
                print(" ".join(cmd))
                n_launched += 1
                continue
            if a.stop_file and Path(a.stop_file).exists():
                print(f"stop-file {a.stop_file} presente: mi fermo prima di {tag}")
                break
            if outdir.exists():
                raise SystemExit(f"{outdir} esiste gia' e {sid}/{arm} non e' tra le corse concluse: non sovrascrivo")
            left = residual_processes()
            if left:
                raise SystemExit("processi residui prima di " + tag + ":\n  " + "\n  ".join(left))
            row.update(tag=tag, predicted_k=run["predicted"][0], predicted_branch=run["predicted"][1],
                       predicted_theta_deg=run["predicted"][2], spawn_yaw_deg=run["spawn_yaw"], outdir=str(outdir))
            load_1m, load_5m, load_15m = os.getloadavg()
            row.update(loadavg_1m_start=f"{load_1m:.2f}", loadavg_5m_start=f"{load_5m:.2f}", loadavg_15m_start=f"{load_15m:.2f}")
            n_launched += 1
            stop_all = False
            t_start = time.time()
            for attempt in (1, 2):
                row["attempts"] = attempt
                print(f"[{sid}/{arm}] tentativo {attempt}: {tag}", flush=True)
                logbase = outroot / "_runner_logs" / f"{tag}_attempt{attempt}"
                logbase.parent.mkdir(parents=True, exist_ok=True)
                rc, err, timed_out = run_once(cmd, logbase, a.run_timeout_s)
                msg = last_fail_message(err)
                cls, motivo = classify_failure(rc, msg, trigger_seen(outdir))
                if timed_out:
                    cls, motivo = "STOP", f"timeout del runner ({a.run_timeout_s:g} s)"
                left = residual_processes()
                stop_note = ""
                if left:
                    residue_msg = "processi residui dopo la corsa: " + "; ".join(left[:3])
                    if cls in CONCLUDED_CLASSES:
                        stop_note = "STOP: " + residue_msg   # l'esito reale resta; la nota va in `motivo`
                    else:
                        cls, motivo = "STOP", residue_msg
                rep = read_report(outdir)
                if cls == "STARTUP":
                    if attempt == 1:
                        dest = unique_name(Path(f"{outdir}_attempt1_STARTUP_FAIL"))
                        if outdir.exists():
                            outdir.rename(dest)
                        print(f"  guasto di avvio ({motivo}); rinominato in {dest.name}; riprovo", flush=True)
                        time.sleep(10.0)
                        continue
                    dest = unique_name(Path(f"{outdir}_attempt2_STARTUP_FAIL"))
                    if outdir.exists():
                        outdir.rename(dest)
                    row.update(status="STARTUP_FAIL_RIPETUTA", motivo=motivo, outdir=str(dest))
                elif cls == "OK":
                    row["status"] = "OK"
                    if rep is None:
                        row.update(status="FALLITA", motivo="report.json assente con codice 0")
                elif cls == STATUS_REFUSAL:
                    # esito corretto e distinto: nessuna ripetizione, nessun report; i dati vengono da run_meta.json
                    meta = read_run_meta(outdir)
                    row.update(status=STATUS_REFUSAL, motivo=refusal_motivo(meta, motivo), **refusal_fields(meta),
                               **planner_log_timing_cells(outdir))
                elif cls == "FALLITA":
                    row.update(status="FALLITA", motivo=motivo)
                elif cls == "CONFIG":
                    row.update(status="NON_ESEGUITA", motivo="config: " + motivo)
                else:   # STOP
                    row.update(status=("OK" if rep is not None and rc == 3 else "NON_ESEGUITA"), motivo=motivo)
                    stop_all = True
                if rep is not None and row["status"] in ("OK", "FALLITA"):
                    fill_from_report(row, rep, run["predicted"])
                if stop_note:
                    row["motivo"] = "; ".join(x for x in (row["motivo"], stop_note) if x)
                    stop_all = True
                break
            row["wall_s"] = f"{time.time() - t_start:.0f}"
            w.writerow(row)
            fo.flush()
            print(f"  -> {row['status']} {row['motivo']}", flush=True)
            if stop_all:
                print("STOP: " + row["motivo"])
                return 3
    finally:
        if not a.print_only:
            fo.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
