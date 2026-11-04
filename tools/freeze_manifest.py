#!/usr/bin/env python3
"""
tools/freeze_manifest.py - congela (sha256) tutto cio' che definisce la campagna e scrive un JSON e una tabella.

    python3 tools/freeze_manifest.py --manifest manifest.csv --dryrun dryrun.csv --demo-dirs runs/<fit_A> runs/<fit_B> \
        --s3-landscape-bin install/.../s3_landscape --planner-bin install/.../grasp_planner_node \
        [--controller-original <franka_gazebo_controllers.yaml>] [--demo-csv-dir _data/demos] --out freeze.json [--table freeze.txt]

Hasha: config/campaign_A.yaml e campaign_B.yaml; tools/s1_success.yaml; il file dei guadagni del controllore (copia nel repo
src/franka_gazebo_overrides/franka_gazebo_controllers.yaml e originale --controller-original; se non e' leggibile: NON LEGGIBILE, senza
fallire); src/haptic_dmp_learning/config/params.yaml; i binari s3_landscape e del nodo planner; per ogni --demo-dirs: weights.yaml e
weights_demo_params.yaml della cartella e, cercati in --demo-csv-dir come demo_raw_<stamp>_trim.csv e demo_raw_<stamp>_trim_joint_states.csv
(<stamp> = AAAAMMGGTHHMMSS preso dal nome della cartella, `fit_demo_<stamp>`), il CSV tagliato e quello dei giunti (NON TROVATO se mancano);
il manifesto e il CSV del dry-run; il profilo di corsa (stringa RUN_PROFILE di campaign_runner.py, incluso nell'hash complessivo).
Riporta `git rev-parse HEAD` e `git status --short` con un avviso se un file hashato ha modifiche non committate (o non e' tracciato).
Il campo `freeze_sha256` e' lo sha256 della lista ordinata "etichetta=sha256" + il profilo + HEAD.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import campaign_runner  # noqa: E402  (solo RUN_PROFILE_STR)

WS = HERE.parent
CONFIG = WS / "src/satellite_grasp_planner/config"
CONTROLLER_REPO = WS / "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml"
CONTROLLER_RUNBOOK = Path("/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/"
                          "franka_gazebo_controllers.yaml")   # tools/s1_run.py:44-47, docs/RUNBOOK_SIMULAZIONE.md (punto 6)
UNREADABLE, NOT_FOUND = "NON LEGGIBILE", "NON TROVATO"


def sha256(path) -> str:
    p = Path(path)
    try:
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return UNREADABLE if p.exists() else NOT_FOUND


def git(*args) -> str:
    r = subprocess.run(["git", "-C", str(WS), *args], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else f"git fallito: {r.stderr.strip()}"


def dirty_among(paths) -> list:
    """File hashati con modifiche non committate o non tracciati (percorsi dentro il repo)."""
    status = git("status", "--short", "--untracked-files=all").splitlines()
    changed = {ln[3:].strip() for ln in status if len(ln) > 3}
    out = []
    for p in paths:
        try:
            rel = str(Path(p).resolve().relative_to(WS.resolve()))
        except ValueError:
            continue   # fuori dal repo
        if rel in changed:
            out.append(rel)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--dryrun", required=True)
    ap.add_argument("--demo-dirs", nargs="*", default=[])
    ap.add_argument("--demo-csv-dir", default=str(WS / "_data/demos"))
    ap.add_argument("--s3-landscape-bin", required=True)
    ap.add_argument("--planner-bin", required=True)
    ap.add_argument("--controller-original", default=str(CONTROLLER_RUNBOOK))
    ap.add_argument("--out", required=True)
    ap.add_argument("--table", default=None)
    a = ap.parse_args(argv)

    items = [("campaign_A.yaml", CONFIG / "campaign_A.yaml"), ("campaign_B.yaml", CONFIG / "campaign_B.yaml"),
             ("s1_success.yaml", HERE / "s1_success.yaml"),
             ("controller_gains_repo", CONTROLLER_REPO), ("controller_gains_original", Path(a.controller_original)),
             ("executor_params.yaml", WS / "src/haptic_dmp_learning/config/params.yaml"),
             ("s3_landscape_bin", Path(a.s3_landscape_bin)), ("planner_bin", Path(a.planner_bin)),
             ("manifest", Path(a.manifest)), ("dryrun_csv", Path(a.dryrun))]
    for d in a.demo_dirs:
        dd = Path(d)
        did = dd.name
        items += [(f"demo[{did}]/weights.yaml", dd / "weights.yaml"), (f"demo[{did}]/weights_demo_params.yaml", dd / "weights_demo_params.yaml")]
        m = re.search(r"fit_demo_(\d{8}T\d{6})", did)
        for suffix, label in (("_trim.csv", "trim_csv"), ("_trim_joint_states.csv", "trim_joint_states_csv")):
            items.append((f"demo[{did}]/{label}", Path(a.demo_csv_dir) / f"demo_raw_{m.group(1)}{suffix}" if m else Path("NON_TROVATO_stamp_assente")))
    files = [{"label": label, "path": str(p), "sha256": sha256(p)} for label, p in items]
    head = git("rev-parse", "HEAD")
    status = git("status", "--short")
    dirty = dirty_among([p for _l, p in items])
    digest = hashlib.sha256(("\n".join(sorted(f"{f['label']}={f['sha256']}" for f in files)) +
                             f"\nprofile={campaign_runner.RUN_PROFILE_STR}\nHEAD={head}\n").encode()).hexdigest()
    doc = {"run_profile": campaign_runner.RUN_PROFILE_STR, "git_head": head, "git_status_short": status.splitlines(),
           "dirty_hashed_files": dirty, "files": files, "freeze_sha256": digest}
    Path(a.out).write_text(json.dumps(doc, indent=2))
    lines = [f"{f['sha256'][:16] if len(f['sha256']) == 64 else f['sha256']:<18} {f['label']:<46} {f['path']}" for f in files]
    lines += ["", f"run profile : {campaign_runner.RUN_PROFILE_STR}", f"git HEAD    : {head}", f"freeze      : {digest}"]
    if dirty:
        lines.append("AVVISO: modifiche non committate / file non tracciati tra i file hashati: " + ", ".join(dirty))
    text = "\n".join(lines)
    print(text)
    if a.table:
        Path(a.table).write_text(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
