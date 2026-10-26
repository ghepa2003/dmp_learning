#!/usr/bin/env python3
"""
tools/s1_campaign.py - griglia di run S1: per ogni combinazione chiama tools/s1_run.py, appende la
riga del report al CSV dei risultati e continua se un run fallisce.

Griglia da CLI e/o da yaml (la CLI ha la precedenza); chiavi: depth, kt, spawn_yaw, follow_s,
hold_s, squeeze, stop_cube (bool; richiede follow_s > 0) (squeeze: "default" o null = parametro non passato),
omega_deg_s (deg/s con segno; spawn_yaw di default "auto" se omega non e' solo -2), center_xyz ("X,Y,Z" o
[X,Y,Z]; default 0.70,0.0,0.77), arm (d_only | v2; default: non passato). spawn_yaw accetta anche
"auto" (yaw0 = theta_star_ref - omega*T_total, vedi s1_run.auto_spawn_yaw_deg). Esempio:

    python3 tools/s1_campaign.py --weights-dir runs/<run> --outroot s1_runs/camp1 \\
        --depth 0.0 0.0125 --kt 2000 --spawn-yaw -73 --squeeze default 0.002

    # grid.yaml:  depth: [0.0, 0.0125]   kt: [2000]   squeeze: [null, 0.002]
    python3 tools/s1_campaign.py --weights-dir runs/<run> --outroot s1_runs/camp2 --dry-run \\
        --omega-deg-s -2 -3 --center-xyz 0.70,0.0,0.77 0.65,0.0,0.77 --arm d_only v2

Il timeout e' in tempo simulato (/clock): --sim-timeout viene passato a s1_run, che lo applica
(piu' una guardia sul /clock fermo). La campagna non ha timeout wall-clock.
Output in --outroot: results.csv (colonne fisse di s1_report), campaign_status.csv (tag, rc, status,
reason), <tag>/ per ogni run (s1_run.log, s1_run.err, report.*).
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s1_report  # noqa: E402
import s1_run  # noqa: E402  (import senza ROS: rclpy solo dentro make_monitor)

HERE = Path(__file__).resolve().parent
AXES = ["depth", "kt", "spawn_yaw", "follow_s", "hold_s", "squeeze", "stop_cube",
        "omega_deg_s", "center_xyz", "arm"]
DEFAULTS = {"depth": [0.0125], "kt": [2000.0], "spawn_yaw": [180.0],
            "follow_s": [3.0], "hold_s": [120.0], "squeeze": [None], "stop_cube": [False],
            "omega_deg_s": [s1_run.OMEGA_DEG_S_DEFAULT], "center_xyz": [s1_run.SAT_CENTER], "arm": [None]}


def _squeeze(v):
    if v is None or str(v).lower() in ("default", "none", "null"):
        return None
    return float(v)


def _bool(v):
    if isinstance(v, str):
        if v.lower() in ("1", "true", "yes", "on"):
            return True
        if v.lower() in ("0", "false", "no", "off"):
            return False
        raise SystemExit(f"stop_cube: valore booleano non valido: {v!r}")
    return bool(v)


def _spawn_yaw(v):
    return "auto" if str(v).strip().lower() == "auto" else float(v)


def _center(v):
    if isinstance(v, (list, tuple)):
        v = ",".join(str(x) for x in v)
    try:
        return s1_run.parse_center_xyz(v)
    except argparse.ArgumentTypeError as e:
        raise SystemExit(f"center_xyz: {e}")


def _arm(v):
    if v is None or str(v).lower() in ("none", "null", "default"):
        return None
    if v not in s1_run.ARMS:
        raise SystemExit(f"arm: valore non valido {v!r} (ammessi: {', '.join(s1_run.ARMS)})")
    return v


def build_grid(cli: dict, yaml_path=None) -> list[dict]:
    cfg = {}
    if yaml_path:
        import yaml
        cfg = yaml.safe_load(Path(yaml_path).read_text()) or {}
        unknown = set(cfg) - set(AXES)
        if unknown:
            raise SystemExit(f"chiavi sconosciute nel yaml: {sorted(unknown)}")
    axes = {}
    for ax in AXES:
        vals = cli.get(ax) or cfg.get(ax) or DEFAULTS[ax]
        vals = vals if isinstance(vals, list) else [vals]
        if ax == "center_xyz" and not (cli.get(ax) or cfg.get(ax)):
            vals = DEFAULTS[ax]
        elif ax == "center_xyz" and isinstance(vals, list) and vals and all(
                isinstance(x, (int, float)) for x in vals):
            vals = [vals]   # yaml: center_xyz: [0.7, 0.0, 0.77] = UN centro
        if ax == "spawn_yaw":
            if not (cli.get(ax) or cfg.get(ax)):
                # omega variato (non solo il default): lo yaw di spawn fisso non e' coerente => auto
                om = cli.get("omega_deg_s") or cfg.get("omega_deg_s") or DEFAULTS["omega_deg_s"]
                om = om if isinstance(om, list) else [om]
                if any(abs(float(o) - s1_run.OMEGA_DEG_S_DEFAULT) > 1e-12 for o in om):
                    vals = ["auto"]
            axes[ax] = [_spawn_yaw(v) for v in vals]
        elif ax == "center_xyz":
            axes[ax] = [_center(v) for v in vals]
        elif ax == "arm":
            axes[ax] = [_arm(v) for v in vals]
        elif ax == "squeeze":
            axes[ax] = [_squeeze(v) for v in vals]
        elif ax == "stop_cube":
            axes[ax] = [_bool(v) for v in vals]
        else:
            axes[ax] = [float(v) for v in vals]
    return [dict(zip(AXES, combo)) for combo in itertools.product(*(axes[a] for a in AXES))]


def _g(v):
    return "def" if v is None else f"{v:g}"


def make_tag(c: dict) -> str:
    yaw = c["spawn_yaw"] if c["spawn_yaw"] == "auto" else _g(c["spawn_yaw"])
    cx, cy, cz = c["center_xyz"]
    return (f"d{_g(c['depth'] * 1000)}_kt{_g(c['kt'])}_y{yaw}_"
            f"f{_g(c['follow_s'])}_h{_g(c['hold_s'])}_q{_g(c['squeeze'])}"
            + ("_stop" if c["stop_cube"] else "")
            + f"_w{_g(c['omega_deg_s'])}_c{_g(cx)}x{_g(cy)}x{_g(cz)}"
            + (f"_a{c['arm']}" if c["arm"] else "")).replace("-", "m")


def done_tags(csv_path: Path) -> set:
    if not csv_path.exists():
        return set()
    with open(csv_path, newline="") as f:
        return {r["tag"] for r in csv.DictReader(f)}


def append_status(path: Path, tag, rc, status, reason):
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["tag", "rc", "status", "reason"])
        w.writerow([tag, rc, status, reason])


def run_one(a, c, tag, outroot: Path):
    outdir = outroot / tag
    outdir.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(HERE / "s1_run.py"), "--weights-dir", a.weights_dir, "--tag", tag,
           "--outdir", str(outdir), "--kt", str(c["kt"]), "--depth", str(c["depth"]),
           "--follow-s", str(c["follow_s"]), "--hold-s", str(c["hold_s"]),
           "--spawn-yaw-deg", str(c["spawn_yaw"]),
           "--omega-deg-s", str(c["omega_deg_s"]),
           "--center-xyz", ",".join(repr(float(x)) for x in c["center_xyz"]), "--sim-timeout", str(a.sim_timeout),
           "--gui" if a.gui else "--no-gui", "--no-keep-alive"]
    if c["arm"]:
        cmd += ["--arm", c["arm"]]
    if c["stop_cube"]:
        cmd.append("--stop-cube")
    if c["squeeze"] is not None:
        cmd += ["--squeeze", str(c["squeeze"])]
    for p in a.extra_exec_param:
        cmd += ["--extra-exec-param", p]
    for p in a.planner_param:
        cmd += ["--planner-param", p]
    if a.axis_tilt_deg is not None:
        cmd += ["--axis-tilt-deg", str(a.axis_tilt_deg)]
    if a.axis_lean_deg is not None:
        cmd += ["--axis-lean-deg", str(a.axis_lean_deg)]
    if a.overwrite:
        cmd.append("--overwrite")
    with open(outdir / "s1_run.log", "w") as fo, open(outdir / "s1_run.err", "w") as fe:
        rc = subprocess.run(cmd, stdout=fo, stderr=fe).returncode
    reason = ""
    for line in reversed((outdir / "s1_run.err").read_text(errors="replace").splitlines()):
        if line.startswith("S1_RUN FAIL:"):
            reason = line[len("S1_RUN FAIL:"):].strip()
            break
    return rc, reason, outdir


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1], formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--weights-dir", required=True)
    ap.add_argument("--outroot", required=True)
    ap.add_argument("--grid", help="yaml con le liste dei valori")
    for ax in AXES:
        ap.add_argument(f"--{ax.replace('_', '-')}", nargs="+", dest=ax, help=f"valori di {ax}")
    ap.add_argument("--sim-timeout", type=float, default=500.0, help="tempo SIMULATO per run [s]")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--extra-exec-param", action="append", default=[])
    ap.add_argument("--planner-param", action="append", default=[], metavar="CHIAVE=VALORE",
                    help="parametro del planner inoltrato a s1_run a ogni run (ripetibile)")
    ap.add_argument("--axis-tilt-deg", type=float, default=None, metavar="ALPHA",
                    help="inclinazione dell'asse [deg], uguale per tutta la griglia (non passata se omessa)")
    ap.add_argument("--axis-lean-deg", type=float, default=None, metavar="PHI",
                    help="azimut dell'asse [deg], uguale per tutta la griglia (non passato se omesso)")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--skip-done", action="store_true", help="salta i tag gia' presenti in results.csv")
    ap.add_argument("--dry-run", action="store_true", help="stampa la griglia e basta")
    a = ap.parse_args(argv)

    cli = {ax: getattr(a, ax) for ax in AXES}
    grid = build_grid(cli, a.grid)
    outroot = Path(a.outroot).resolve()
    results, status = outroot / "results.csv", outroot / "campaign_status.csv"
    print(f"{len(grid)} run in griglia -> {outroot}", flush=True)
    if a.dry_run:
        for c in grid:
            yaw = (s1_run.auto_spawn_yaw_deg(c["omega_deg_s"]) if c["spawn_yaw"] == "auto" else c["spawn_yaw"])
            print(make_tag(c), json.dumps(c), f"yaw0={yaw:.2f}deg")
        return 0
    outroot.mkdir(parents=True, exist_ok=True)

    already = done_tags(results) if a.skip_done else set()
    n_fail = 0
    for i, c in enumerate(grid, 1):
        tag = make_tag(c)
        if tag in already:
            print(f"[{i}/{len(grid)}] {tag}: gia' fatto, salto", flush=True)
            continue
        print(f"[{i}/{len(grid)}] {tag}", flush=True)
        try:
            rc, reason, outdir = run_one(a, c, tag, outroot)
        except Exception as e:  # noqa: BLE001  (la campagna non si ferma per un run)
            rc, reason, outdir = -1, f"{type(e).__name__}: {e}", outroot / tag
        rep = outdir / "report.json"
        row_ok = False
        if rep.exists():
            try:
                s1_report.append_csv(results, json.loads(rep.read_text())["row"])
                row_ok = True
            except (OSError, KeyError, ValueError) as e:
                reason = reason or f"report.json illeggibile: {e}"
        if rc == 0 and row_ok:
            st = "ok"
        else:
            st = "fail"
            n_fail += 1
        append_status(status, tag, rc, st, reason)
        print(f"    rc={rc} {st}{': ' + reason if reason else ''}", flush=True)
    print(f"finito: {len(grid)} run, {n_fail} falliti", flush=True)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
