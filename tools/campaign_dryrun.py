#!/usr/bin/env python3
"""
tools/campaign_dryrun.py - dry run OFFLINE dei due bracci della campagna (nessun Gazebo): per ogni scenario del manifesto
(tools/campaign_manifest.py) usa s3_landscape --all-candidates e scrive un CSV con vincitori, margini L2, clearance, tempi di
selezione e il numero di corse Gazebo necessarie.

    python3 tools/campaign_dryrun.py --manifest manifest.csv --out dryrun.csv [--mode fast|prod] [--bin <s3_landscape>] [--urdf <urdf>]

--mode prod: DUE chiamate di s3_landscape per scenario, con config/campaign_A.yaml e con config/campaign_B.yaml (descrizione sotto).
--mode fast (default): UNA sola chiamata per scenario, con campaign_B.yaml; i vincitori si derivano dalle righe del CSV --all-candidates
(colonne v1_ok, v2_ok, v1p_ok, l2_diverged, l2_total_deg, margin_delta_deg, J_s, rank, error):
  - B = riga con J_s minimo (a parita': rank minimo) tra quelle con v1_ok = v2_ok = v1p_ok = 1, senza errore, l2_diverged != 1 e
    l2_total_deg >= margin_delta_deg (stesso criterio di S3FinalistRecord::dyn_rejected, grasp_s3_selection.cpp:902);
  - A = lo stesso senza il vincolo L2 (campaign_A.yaml differisce da campaign_B.yaml solo in s1.use_dynamic_follow; entrambi hanno
    use_margin e posthoc_velocity_check falsi, quindi i vincoli di A sono V1, V2, V1').
  In fast selection_ms_A resta vuoto (non misurato) e selection_ms_B e' il tempo della selezione di produzione con L2 attivo;
  il vincitore di produzione di B (lazy_outcome == winner) e' confrontato con quello derivato e, se diverge, scritto in `note`.

Per scenario (descrizione di prod):
  - corsa A: s3_landscape --params campaign_A.yaml --all-candidates ...  -> vincitore A (riga con lazy_outcome == "winner"), clearance,
    selection_ms (tempo della selezione di produzione, <out>.meta);
  - corsa B: lo stesso con campaign_B.yaml (L2 attivo) -> vincitore B, e per OGNI finalista il margine L2 totale (l2_total_deg): il margine
    L2 del vincitore di A si legge nella riga di B con lo stesso (k, ramo, theta), perche' A non valuta L2.
  - opzioni dello scenario: --weights-dir <demo>, --omega-deg-s, --center, e (solo con alpha > 0) --axis-tilt-deg/--axis-lean-deg;
    --theta-snap-rad, --t-snap-s, --depth, --min-delay-s, --q-start restano quelli del binario / del YAML (campagna precedente:
    docs/RUNBOOK_SIMULAZIONE.md, procedura (a): fase di snapshot fissa, yaw di spawn scelto dopo da theta*).
  - run_needed: i vincitori coincidono -> "A" (UNA corsa, condivisa dai due bracci); un solo braccio ha un vincitore -> quel braccio
    (UNA corsa); nessuno -> "none"; vincitori diversi -> "both" (DUE corse). Se uno dei due bracci da' errore o timeout: "error".
  - spawn_yaw_deg_X = wrap(theta*_X - omega * T_total) in (-180, 180] (s1_run.auto_spawn_yaw_deg senza il riferimento fisso), da usare
    come --spawn-yaw-deg della corsa Gazebo; T_total = tau_demo + (45.2 - 24.474) (s1_run.t_total_for_tau), con tau_demo = demo.tau_s di
    <demo>/weights_demo_params.yaml (45.2 per D1).
Default SEQUENZIALE (selection_ms e' tempo reale). Con --jobs N > 1 gli scenari girano in parallelo (un processo s3_landscape per
arm e scenario, file per corsa distinti in <out>.runs/): il CSV resta nell'ordine del manifesto, ma selection_ms_A/B cresce con il
carico della macchina e non e' confrontabile con una corsa sequenziale. File per corsa in <out>.runs/.
Accanto al CSV: <out>.json con sha256 di manifesto, dei due YAML e del binario, e le righe di comando.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import s1_run  # noqa: E402  (solo le funzioni pure: wrap_deg_half_open, t_total_for_tau, read_demo_tau_s)

WS_DIR = HERE.parent
CONFIG_DIR = WS_DIR / "src/satellite_grasp_planner/config"
BIN_REL = "install/satellite_grasp_planner/lib/satellite_grasp_planner/s3_landscape"
STATUS_NAMES = {0: "selected", 1: "no_candidate", 2: "posthoc_failed"}   # S3Result::Status
THETA_TOL_DEG = 1e-6
COLUMNS = [
    "scenario_id", "demo", "omega_deg_s", "alpha_deg", "phi_deg", "cx", "cy", "cz",
    "status_A", "status_B",
    "winner_A_k", "winner_A_branch", "winner_A_theta_deg", "winner_B_k", "winner_B_branch", "winner_B_theta_deg", "same_winner",
    "l2_margin_total_deg_winner_A", "l2_margin_total_deg_winner_B", "clearance_mm_winner_A", "clearance_mm_winner_B",
    "selection_ms_A", "selection_ms_B", "run_needed", "n_runs", "spawn_yaw_deg_A", "spawn_yaw_deg_B", "note",
]


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve_demo(demo: str) -> Path:
    p = Path(demo)
    return p if p.is_absolute() or p.exists() else WS_DIR / p


def read_meta(path: Path) -> dict:
    out = {}
    if path.is_file():
        for line in path.read_text(errors="replace").splitlines():
            k, sep, v = line.partition("=")
            if sep:
                out[k] = v          # l'ultima occorrenza vince
    return out


def scenario_args(sc: dict) -> list:
    args = ["--weights-dir", str(resolve_demo(sc["demo"])), "--omega-deg-s", sc["omega_deg_s"],
            "--center", f"{sc['cx']},{sc['cy']},{sc['cz']}"]
    if float(sc["alpha_deg"]) > 0.0:
        args += ["--axis-tilt-deg", sc["alpha_deg"], "--axis-lean-deg", sc["phi_deg"]]
    return args


def run_arm(a, sc: dict, arm: str, workdir: Path) -> dict:
    """Una corsa di s3_landscape. Ritorna {status, selection_ms, rows, cmd, rc, err}."""
    tag = f"{sc['scenario_id']}_{arm}"
    out_csv = workdir / f"{tag}.csv"
    for stale in (out_csv, Path(str(out_csv) + ".meta")):
        if stale.exists():
            stale.unlink()
    cmd = [str(a.bin), "--params", str(CONFIG_DIR / f"campaign_{arm}.yaml"), "--all-candidates", "--out", str(out_csv)]
    cmd += scenario_args(sc)
    if a.urdf:
        cmd += ["--urdf", a.urdf]
    res = {"cmd": cmd, "rows": [], "selection_ms": "", "rc": None, "err": ""}
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=a.timeout_s)
    except subprocess.TimeoutExpired:
        res["status"], res["err"] = "timeout", f"oltre {a.timeout_s:g} s"
        return res
    res["rc"] = p.returncode
    (workdir / f"{tag}.stderr").write_text(p.stderr)
    if p.returncode != 0 or not out_csv.is_file():
        last = [ln for ln in p.stderr.splitlines() if ln.strip()]
        res["status"], res["err"] = "error", (last[-1] if last else f"rc={p.returncode}")
        return res
    meta = read_meta(Path(str(out_csv) + ".meta"))
    try:
        res["status"] = STATUS_NAMES[int(meta["status"])]
    except (KeyError, ValueError):
        res["status"], res["err"] = "error", "status assente in .meta"
        return res
    res["selection_ms"] = meta.get("selection_ms", "")
    with open(out_csv, newline="") as f:
        res["rows"] = list(csv.DictReader(f))
    return res


def winner_row(res: dict):
    """La riga con lazy_outcome == winner (esattamente una se status == selected)."""
    w = [r for r in res["rows"] if r.get("lazy_outcome") == "winner"]
    return w[0] if len(w) == 1 else None


def same_candidate(r1: dict, r2: dict) -> bool:
    return (r1["k"] == r2["k"] and r1["branch"] == r2["branch"]
            and abs(float(r1["theta_deg"]) - float(r2["theta_deg"])) < THETA_TOL_DEG)


def find_row(rows: list, ref: dict):
    for r in rows:
        if same_candidate(r, ref):
            return r
    return None


def spawn_yaw(row, omega: float, t_total_s: float) -> str:
    if row is None:
        return ""
    return f"{s1_run.wrap_deg_half_open(float(row['theta_deg']) - omega * t_total_s):.3f}"


def _num(row: dict, key: str) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return float("nan")


def passes_static_constraints(r: dict) -> bool:
    """V1, V2, V1' superati (colonne v1_ok, v2_ok, v1p_ok = "1"; vuota = non calcolata = non superata) e nessun errore di riga."""
    return r.get("error", "") == "" and all(r.get(k) == "1" for k in ("v1_ok", "v2_ok", "v1p_ok"))


def passes_l2(r: dict) -> bool:
    """Come dyn_rejected (grasp_s3_selection.cpp:902): non divergente e l2_total_deg >= margin_delta_deg (NaN/vuoto -> non superato)."""
    total, delta = _num(r, "l2_total_deg"), _num(r, "margin_delta_deg")
    return r.get("l2_diverged") != "1" and total == total and delta == delta and total >= delta


def best_by_j_static(rows: list):
    """Riga con J_s minimo; a parita' il rank minore. None se non ce ne sono (J_s non finito esclusa)."""
    ok = [r for r in rows if _num(r, "J_s") == _num(r, "J_s")]
    if not ok:
        return None
    return min(ok, key=lambda r: (_num(r, "J_s"), _num(r, "rank")))


def derive_winners(rows: list):
    """(vincitore A, vincitore B) dalle righe di UNA corsa --all-candidates con la configurazione di B (vedi docstring del modulo)."""
    static_ok = [r for r in rows if passes_static_constraints(r)]
    return best_by_j_static(static_ok), best_by_j_static([r for r in static_ok if passes_l2(r)])


def run_needed(res_a, res_b, wa, wb, same: bool):
    if res_a["status"] in ("error", "timeout") or res_b["status"] in ("error", "timeout"):
        return "error", 0
    if wa is None and wb is None:
        return "none", 0
    if wa is None:
        return "B", 1
    if wb is None:
        return "A", 1
    return ("A", 1) if same else ("both", 2)


def scenario_row(a, sc: dict, workdir: Path) -> dict:
    row = {c: "" for c in COLUMNS}
    row.update({k: sc[k] for k in ("scenario_id", "demo", "omega_deg_s", "alpha_deg", "phi_deg", "cx", "cy", "cz")})
    notes = []
    if not (resolve_demo(sc["demo"]) / "weights_demo_params.yaml").is_file():
        row.update(status_A="error", status_B="error", run_needed="error", n_runs=0,
                   note="weights_demo_params.yaml mancante in " + str(resolve_demo(sc["demo"])))
        return row
    try:
        t_total = s1_run.t_total_for_tau(s1_run.read_demo_tau_s(resolve_demo(sc["demo"])))
    except s1_run.RunError as e:
        row.update(status_A="error", status_B="error", run_needed="error", n_runs=0, note=str(e))
        return row
    if a.mode == "prod":
        res_a, res_b = run_arm(a, sc, "A", workdir), run_arm(a, sc, "B", workdir)
        wa = winner_row(res_a) if res_a["status"] == "selected" else None
        wb = winner_row(res_b) if res_b["status"] == "selected" else None
    else:
        res_b = run_arm(a, sc, "B", workdir)
        res_a = {"cmd": res_b["cmd"], "rows": [], "selection_ms": "", "rc": res_b["rc"], "err": ""}   # A derivato da B (fast)
        if res_b["status"] in ("error", "timeout"):
            res_a["status"] = res_b["status"]
            wa = wb = None
        else:
            wa, wb = derive_winners(res_b["rows"])
            res_a["status"] = "selected" if wa is not None else "no_candidate"
            res_b_prod = winner_row(res_b) if res_b["status"] == "selected" else None
            if (res_b_prod is None) != (wb is None) or (wb is not None and not same_candidate(res_b_prod, wb)):
                notes.append("fast: vincitore B derivato diverso dal winner di produzione di B")
            res_b["status"] = "selected" if wb is not None else "no_candidate"
    row["status_A"], row["status_B"] = res_a["status"], res_b["status"]
    row["selection_ms_A"], row["selection_ms_B"] = res_a["selection_ms"], res_b["selection_ms"]
    for arm, res in (("A", res_a), ("B", res_b)):
        if res["err"]:
            notes.append(f"{arm}: {res['err']}")
    if a.mode == "prod":
        for arm, res, w in (("A", res_a, wa), ("B", res_b, wb)):
            if res["status"] == "selected" and w is None:
                notes.append(f"{arm}: selected ma nessuna riga winner nel CSV")
    for arm, w in (("A", wa), ("B", wb)):
        if w is not None:
            row[f"winner_{arm}_k"], row[f"winner_{arm}_branch"], row[f"winner_{arm}_theta_deg"] = w["k"], w["branch"], w["theta_deg"]
            row[f"clearance_mm_winner_{arm}"] = w.get("clearance_mm", "")
    row["l2_margin_total_deg_winner_B"] = wb.get("l2_total_deg", "") if wb is not None else ""
    if wa is not None and res_b["rows"]:
        rb = find_row(res_b["rows"], wa)
        if rb is None:
            notes.append("vincitore di A non tra i finalisti di B")
        else:
            row["l2_margin_total_deg_winner_A"] = rb.get("l2_total_deg", "")
    same = wa is not None and wb is not None and same_candidate(wa, wb)
    row["same_winner"] = (1 if same else 0) if (wa is not None and wb is not None) else ""
    row["run_needed"], row["n_runs"] = run_needed(res_a, res_b, wa, wb, same)
    omega = float(sc["omega_deg_s"])
    row["spawn_yaw_deg_A"], row["spawn_yaw_deg_B"] = spawn_yaw(wa, omega, t_total), spawn_yaw(wb, omega, t_total)
    row["note"] = " | ".join(notes)
    row["_cmds"] = [res_a["cmd"], res_b["cmd"]]
    return row


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--mode", choices=("fast", "prod"), default="fast",
                    help="fast (default): UNA corsa di s3_landscape per scenario (campaign_B.yaml) e vincitori A/B derivati dalle righe\n"
                         "--all-candidates; prod: DUE corse (campaign_A.yaml e campaign_B.yaml), vincitori letti da lazy_outcome == winner")
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bin", default=os.environ.get("S3_LANDSCAPE_BIN", str(WS_DIR / BIN_REL)),
                    help=f"binario s3_landscape (default $S3_LANDSCAPE_BIN o {BIN_REL})")
    ap.add_argument("--urdf", default=None, help="--urdf di s3_landscape (default del binario: $HOME/thesis_ws/fer_flat_effort.urdf)")
    ap.add_argument("--timeout-s", type=float, default=1800.0, help="timeout wall per corsa (1800)")
    ap.add_argument("--limit", type=int, default=None, help="solo i primi N scenari del manifesto")
    ap.add_argument("--jobs", type=int, default=1, help="scenari in parallelo (processi); il CSV resta nell'ordine del manifesto (1)")
    ap.add_argument("--resume", action="store_true", help="salta gli scenari gia' presenti in --out (e appende)")
    a = ap.parse_args(argv)
    a.bin = Path(a.bin)
    if a.jobs < 1:
        raise SystemExit("--jobs deve essere >= 1")
    if not a.bin.is_file():
        raise SystemExit(f"binario non trovato: {a.bin} (compilare con -DBUILD_ANALYSIS_TOOLS=ON o passare --bin)")
    for arm in ("A", "B"):
        if not (CONFIG_DIR / f"campaign_{arm}.yaml").is_file():
            raise SystemExit(f"manca {CONFIG_DIR / f'campaign_{arm}.yaml'}")
    with open(a.manifest, newline="") as f:
        scenarios = list(csv.DictReader(f))
    if a.limit is not None:
        scenarios = scenarios[: a.limit]
    out = Path(a.out)
    workdir = Path(str(out) + ".runs")
    workdir.mkdir(parents=True, exist_ok=True)
    done = set()
    if a.resume and out.is_file():
        with open(out, newline="") as f:
            done = {r["scenario_id"] for r in csv.DictReader(f)}
    new_file = not (a.resume and out.is_file())
    cmds = {}
    info_path = Path(str(out) + ".json")
    if a.resume and info_path.is_file():
        cmds = json.loads(info_path.read_text()).get("commands", {})
    with open(out, "w" if new_file else "a", newline="") as fo:
        w = csv.DictWriter(fo, fieldnames=COLUMNS, lineterminator="\n")
        if new_file:
            w.writeheader()
        index = {sc["scenario_id"]: i for i, sc in enumerate(scenarios, 1)}
        todo = [sc for sc in scenarios if sc["scenario_id"] not in done]

        def emit(sc, row):
            cmds[sc["scenario_id"]] = row.pop("_cmds", None)
            w.writerow(row)
            fo.flush()
            print(f"[{index[sc['scenario_id']]}/{len(scenarios)}] {sc['scenario_id']}: A={row['status_A']} B={row['status_B']} "
                  f"run_needed={row['run_needed']}", flush=True)

        if a.jobs == 1:
            for sc in todo:
                emit(sc, scenario_row(a, sc, workdir))
        else:
            # map() restituisce i risultati nell'ordine di `todo` (= ordine del manifesto) qualunque sia l'ordine di fine dei processi
            with ProcessPoolExecutor(max_workers=a.jobs) as pool:
                for sc, row in zip(todo, pool.map(scenario_row, [a] * len(todo), todo, [workdir] * len(todo))):
                    emit(sc, row)
    info = {"manifest": str(a.manifest), "manifest_sha256": sha256_file(a.manifest),
            "config_A_sha256": sha256_file(CONFIG_DIR / "campaign_A.yaml"), "config_B_sha256": sha256_file(CONFIG_DIR / "campaign_B.yaml"),
            "binary": str(a.bin), "binary_sha256": sha256_file(a.bin), "urdf": a.urdf, "jobs": a.jobs, "mode": a.mode, "commands": cmds}
    info_path.write_text(json.dumps(info, indent=2))
    print(f"scritto {out} e {out}.json; sha256 CSV {sha256_file(out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
