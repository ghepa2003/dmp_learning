#!/usr/bin/env python3
"""Test sintetici degli avvisi di tools/s3_landscape_report.py (psi senza margini, metadati assenti).

Esecuzione: pytest -x tools/test_s3_landscape_report.py
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s3_landscape_report as rep  # noqa: E402

HEADER = ("k,theta_deg,psi_deg,accepted,reject_first,reject_mask,ik_ok_A,ik_ok_B,branch_selected,d_pos_mm,t_wait_s,"
          "J_current,w_hat,w_trans,w_rot,w_full,smin_trans,smin_rot,margin_rollout_rad,margin_follow_rad,follow_ik_ok,"
          "path_ok,w_full_other,margin_other,margin_ik_rad,dv_m_s,w_full_seedmin,w_full_seedmax,margin_seedmin,"
          "margin_seedmax,n_seed_conv,grid").split(",")
WARN = "ATTENZIONE: righe con psi != 0 senza margini"
NO_META = "metadati assenti: non verificabile lo scenario"


def _row(k, theta, psi, margins):
    nan = float("nan")
    m = 0.5 if margins else nan
    r = dict.fromkeys(HEADER, 0)
    r.update(k=k, theta_deg=theta, psi_deg=psi, accepted=1, reject_first="none", reject_mask=0, ik_ok_A=1, ik_ok_B=0,
             d_pos_mm=5.0 + k + abs(theta) / 100, t_wait_s=3.0 + k, J_current=5.0, w_hat=0.8, w_trans=0.5, w_rot=0.5,
             w_full=0.4 + 0.01 * psi, margin_rollout_rad=m, margin_follow_rad=m, follow_ik_ok=1 if margins else 0,
             path_ok=1 if margins else 0, margin_ik_rad=0.5, grid="coarse")
    return r


def _csv(tmp_path, margins, meta=None):
    p = tmp_path / "landscape.csv"
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=HEADER)
        w.writeheader()
        for k in range(2):
            for th in (-160.0, 0.0, 90.0):
                for psi in (-2.0, 0.0, 2.0):
                    # psi = 0 rows always carry margins (as with --rollout psi0)
                    w.writerow(_row(k, th, psi, margins or psi == 0.0))
    if meta:
        Path(str(p) + ".meta").write_text(meta)
    return p


def _run(tmp_path, csv_path, capsys):
    out = tmp_path / "out"
    assert rep.main(["--csv", str(csv_path), "--outdir", str(out), "--kappa-list", "1,10",
                     "--delta-m-list-deg", "0,6", "--map-kappa-w-list", "1", "--map-kappa-t-list", "0,1"]) == 0
    return (out / "report.txt").read_text()


def test_psi_without_margins_warns_and_marks_invalid(tmp_path, capsys):
    text = _run(tmp_path, _csv(tmp_path, margins=False), capsys)
    assert text.splitlines()[0].startswith(WARN)
    assert "psi* = 0 forzato" in text.splitlines()[0]
    assert "NON VALIDE" in text.splitlines()[0]
    sect = [l for l in text.splitlines() if l.startswith("k) psi*") or "winner vs psi_max" in l]
    assert len(sect) == 2 and all("[NON VALIDE]" in l for l in sect)


def test_psi_with_finite_margins_no_warning(tmp_path, capsys):
    text = _run(tmp_path, _csv(tmp_path, margins=True), capsys)
    assert WARN not in text and "NON VALIDE" not in text


def test_missing_meta_reported(tmp_path, capsys):
    text = _run(tmp_path, _csv(tmp_path, margins=True), capsys)
    assert NO_META in text


def test_meta_printed_when_present(tmp_path, capsys):
    meta = "params=/x/grasp_planner_example.yaml\ngrasp_radius_m=0.068\ntheta_snap_rad=-1.6727670737402633\nt_snap_s=18.318\n"
    text = _run(tmp_path, _csv(tmp_path, margins=True, meta=meta), capsys)
    assert NO_META not in text
    assert "grasp_radius_m=0.068" in text and "t_snap_s=18.318" in text
