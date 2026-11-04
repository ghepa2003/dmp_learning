#!/usr/bin/env python3
"""
tools/campaign_manifest.py - manifesto degli scenari della campagna di validazione (seed fisso, solo libreria standard).

    python3 tools/campaign_manifest.py --demos runs/<fit_A> runs/<fit_B> --n-per-demo 40 --seed 20261101 --out manifest.csv

Livelli (fissati): omega in {-1..-6} deg/s; alpha in {0, 10, 20} deg; lean phi in {0, 90, 180, 270} deg (phi non ha effetto
con alpha = 0: scritto 0); centro in {(0.70,0,0.77) nominale, (0.75,-0.05,0.82), (0.65,0.05,0.82), (0.70,-0.05,0.72), (0.75,0.05,0.72)}.

Per ciascuna demo:
  (i)  8 scenari d'angolo: omega in {-1,-6} x alpha in {0,20} x centro in {nominale, (0.75,0.05,0.72)}, phi = 0;
  (ii) n-per-demo - 8 scenari estratti con il generatore della demo, stratificati su omega: ogni livello ne riceve
       (n-per-demo - 8) // 6, e il resto (< 6) va a livelli di omega scelti dal generatore, uno ciascuno. Dentro un livello
       alpha, phi (solo se alpha > 0) e centro sono estratti uniformemente; uno scenario identico a uno gia' presente per la
       stessa demo (inclusi gli otto d'angolo) viene riestratto (nessun duplicato); non c'e' altra esclusione.
Il generatore di ciascuna demo e' random.Random(int derivato da sha256(f"{seed}|{id demo}")) e si usa solo random().
Colonne: scenario_id, demo, omega_deg_s, alpha_deg, phi_deg, cx, cy, cz, seed, kind (corner | random).
`seed` di riga = seed + 1000 * indice della demo + indice dello scenario nella demo. Stampa l'sha256 del file scritto.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import random
import sys
from pathlib import Path

OMEGAS = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0]
ALPHAS = [0.0, 10.0, 20.0]
PHIS = [0.0, 90.0, 180.0, 270.0]
NOMINAL = (0.70, 0.0, 0.77)
CENTRES = [NOMINAL, (0.75, -0.05, 0.82), (0.65, 0.05, 0.82), (0.70, -0.05, 0.72), (0.75, 0.05, 0.72)]
CORNER_OMEGAS = [-1.0, -6.0]
CORNER_ALPHAS = [0.0, 20.0]
CORNER_CENTRES = [NOMINAL, (0.75, 0.05, 0.72)]
N_CORNER = len(CORNER_OMEGAS) * len(CORNER_ALPHAS) * len(CORNER_CENTRES)
MAX_TRIES = 100000
COLUMNS = ["scenario_id", "demo", "omega_deg_s", "alpha_deg", "phi_deg", "cx", "cy", "cz", "seed", "kind"]


def demo_id(demo: str) -> str:
    return Path(demo.rstrip("/")).name


def demo_rng(seed: int, demo: str) -> random.Random:
    digest = hashlib.sha256(f"{seed}|{demo_id(demo)}".encode()).digest()
    return random.Random(int.from_bytes(digest[:8], "big"))


def pick(rng: random.Random, seq):
    """Estrazione uniforme con random() (stabile tra versioni di Python)."""
    return seq[min(int(rng.random() * len(seq)), len(seq) - 1)]


def corner_scenarios():
    """Gli 8 scenari d'angolo, (omega, alpha, phi, centro); phi = 0."""
    return [(w, a, 0.0, c) for w in CORNER_OMEGAS for a in CORNER_ALPHAS for c in CORNER_CENTRES]


def omega_quotas(rng: random.Random, n_random: int) -> dict:
    """Stesso numero per ogni livello di omega; il resto (< 6) a livelli scelti dal generatore, uno ciascuno."""
    base, extra = divmod(n_random, len(OMEGAS))
    quota = {w: base for w in OMEGAS}
    pool = list(OMEGAS)
    for _ in range(extra):
        w = pick(rng, pool)
        pool.remove(w)
        quota[w] += 1
    return quota


def random_scenarios(rng: random.Random, n_random: int, taken: set) -> list:
    out = []
    for w, q in omega_quotas(rng, n_random).items():
        got = 0
        for _ in range(MAX_TRIES):
            if got == q:
                break
            a = pick(rng, ALPHAS)
            ph = pick(rng, PHIS) if a > 0.0 else 0.0
            c = pick(rng, CENTRES)
            if (w, a, ph, c) in taken:
                continue
            taken.add((w, a, ph, c))
            out.append((w, a, ph, c))
            got += 1
        if got < q:
            raise SystemExit(f"omega {w:g}: {q} scenari richiesti ma i livelli ne offrono meno di {q} distinti")
    return out


def build_manifest(demos: list, n_per_demo: int, seed: int) -> list:
    if n_per_demo < N_CORNER:
        raise SystemExit(f"--n-per-demo {n_per_demo} < {N_CORNER}: gli scenari d'angolo da soli sono {N_CORNER}")
    rows = []
    for di, demo in enumerate(demos):
        rng = demo_rng(seed, demo)
        corners = corner_scenarios()
        taken = set(corners)
        scen = [(s, "corner") for s in corners]
        scen += [(s, "random") for s in random_scenarios(rng, n_per_demo - N_CORNER, taken)]
        for i, ((w, a, ph, c), kind) in enumerate(scen):
            rows.append({"scenario_id": f"d{di}_s{i:03d}", "demo": demo, "omega_deg_s": f"{w:g}", "alpha_deg": f"{a:g}",
                         "phi_deg": f"{ph:g}", "cx": f"{c[0]:.2f}", "cy": f"{c[1]:.2f}", "cz": f"{c[2]:.2f}",
                         "seed": seed + 1000 * di + i, "kind": kind})
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--demos", nargs="+", required=True, help="cartelle dei pesi ProDMP (weights.yaml + weights_demo_params.yaml)")
    ap.add_argument("--n-per-demo", type=int, required=True, help="scenari per demo (>= 8, gli otto d'angolo inclusi)")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if len(set(a.demos)) != len(a.demos):
        raise SystemExit("--demos: elementi ripetuti")
    if len({demo_id(d) for d in a.demos}) != len(a.demos):
        raise SystemExit("--demos: due cartelle con lo stesso nome (il generatore e' per nome)")
    rows = build_manifest(a.demos, a.n_per_demo, a.seed)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    sha = hashlib.sha256(Path(a.out).read_bytes()).hexdigest()
    print(f"{len(rows)} scenari ({len(a.demos)} demo) -> {a.out}")
    print(f"sha256  {sha}  {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
