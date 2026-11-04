#!/usr/bin/env python3
"""Stampa (senza lanciare nulla) il comando `ros2 run ... demo_replay_sync_orchestrator_node` per le demo
con cubo rotante, da incollare in T6.

  yaw0 = wrap(fase_al_contatto - omega * T_c)   [rad, in (-pi, pi]],   wz = omega [rad/s]

Con --alpha-deg > 0 roll/pitch/yaw vengono da s1_run.tilt_values (R_spawn = R_tilt.Rz(yaw0), angoli
estrinseci Rz(yaw)Ry(pitch)Rx(roll) di `ros_gz_sim create -R -P -Y`; wz resta nel frame BODY, quindi il cubo
ruota attorno a a = R_tilt.z). L'orchestratore inoltra i valori a free_target_object.launch.py con
std::to_string (6 decimali): roll/pitch/yaw stampati qui sono gia' arrotondati a 6 decimali.

Uso: demo_orchestrator_cmd.py --omega-deg-s -2 --tc-s 20 [--alpha-deg 20 --lean-deg 90]
     demo_orchestrator_cmd.py --selftest
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import s1_run  # noqa: E402  (solo definizioni e import all'import; lo importano gia' i test in tests/)

DEFAULT_PARAMS_FILE = "/root/thesis_ws/src/haptic_dmp_learning/config/params.yaml"
DEFAULT_CENTER = "0.70,0,0.77"
NODE_DECIMALS = 6      # std::to_string nell'orchestratore (core/target_spawn_args.hpp: numArg)


def wrap_rad_half_open(a: float) -> float:
    """Angolo [rad] in (-pi, pi]."""
    r = (a + math.pi) % (2.0 * math.pi) - math.pi      # [-pi, pi)
    return math.pi if r == -math.pi else r


def yaw0_rad(omega_deg_s: float, tc_s: float, contact_phase_deg: float) -> float:
    """yaw0 = wrap(fase al contatto - omega*T_c) [rad]."""
    return wrap_rad_half_open(math.radians(contact_phase_deg) - math.radians(omega_deg_s) * tc_s)


def compute(omega_deg_s, tc_s, alpha_deg, lean_deg, center, contact_phase_deg):
    """Valori numerici dei parametri dell'orchestratore (dict); roll/pitch/yaw arrotondati come li usera' il nodo."""
    if not (math.isfinite(omega_deg_s) and omega_deg_s < 0.0):
        raise ValueError("--omega-deg-s deve essere finito e negativo")
    if not (math.isfinite(tc_s) and tc_s > 0.0):
        raise ValueError("--tc-s deve essere finito e > 0")
    yaw0 = yaw0_rad(omega_deg_s, tc_s, contact_phase_deg)
    out = {"center": tuple(center), "wz": math.radians(omega_deg_s), "yaw0_rad": yaw0,
           "roll": 0.0, "pitch": 0.0, "yaw": yaw0, "axis": np.array([0.0, 0.0, 1.0])}
    if alpha_deg > 0.0:
        tv = s1_run.tilt_values(alpha_deg, lean_deg, math.degrees(yaw0))
        r, p, y = (float(s) for s in tv["spawn_rpy_str"])
        out.update(roll=r, pitch=p, yaw=y, axis=tv["axis"])
    for k in ("roll", "pitch", "yaw"):
        out[k] = round(out[k], NODE_DECIMALS)          # quello che il nodo passera' al launch
    # asse effettivo dai valori arrotondati: R_spawn.z (con wz nel frame BODY il corpo ruota attorno al proprio z)
    R = s1_run.rpy_extrinsic_to_matrix(out["roll"], out["pitch"], out["yaw"])
    eff = R @ np.array([0.0, 0.0, 1.0])
    out["axis_dev_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, float(eff @ out["axis"])))))
    return out


def format_command(v, alpha_deg, params_file) -> str:
    c = v["center"]
    # Sempre con il punto decimale: "-p target_y:=0" sarebbe un INTEGER per ROS e un parametro double lo rifiuta.
    lines = ["ros2 run haptic_dmp_learning demo_replay_sync_orchestrator_node --ros-args \\",
             f"  --params-file {params_file} \\",
             "  -p mode:=demo -p use_csv_playback:=false \\",
             f"  -p target_x:={c[0]:.6f} -p target_y:={c[1]:.6f} -p target_z:={c[2]:.6f} \\"]
    if alpha_deg > 0.0:
        lines.append(f"  -p target_roll_rad:={v['roll']:.{NODE_DECIMALS}f} "
                     f"-p target_pitch_rad:={v['pitch']:.{NODE_DECIMALS}f} \\")
    lines.append(f"  -p target_wz:={v['wz']:.9f} -p target_yaw:={v['yaw']:.{NODE_DECIMALS}f}")
    return "\n".join(lines)


def selftest() -> int:
    fails = []

    def check(name, ok, detail=""):
        print(f"[{'OK' if ok else 'FAIL'}] {name} {detail}")
        if not ok:
            fails.append(name)

    c = (0.70, 0.0, 0.77)
    v = compute(-2.0, 20.0, 0.0, 0.0, c, 180.0)
    check("omega -2, Tc 20: yaw -2.4435", abs(v["yaw"] - (-2.4435)) < 5e-5, f"(yaw={v['yaw']:.6f})")
    check("omega -2: wz -0.034906585", abs(v["wz"] - (-0.034906585)) < 5e-10, f"(wz={v['wz']:.9f})")
    v = compute(-4.0, 20.0, 0.0, 0.0, c, 180.0)
    check("omega -4, Tc 20: yaw -1.7453", abs(v["yaw"] - (-1.7453)) < 5e-5, f"(yaw={v['yaw']:.6f})")
    check("omega -4: wz -0.0698132", abs(v["wz"] - (-0.0698132)) < 5e-8, f"(wz={v['wz']:.9f})")
    check("alpha 0: roll = pitch = 0", v["roll"] == 0.0 and v["pitch"] == 0.0)
    check("wrap in (-pi, pi]", wrap_rad_half_open(-math.pi) == math.pi and wrap_rad_half_open(math.pi) == math.pi)
    check("centro con punto decimale",
          "target_y:=0.000000" in format_command(compute(-2.0, 20.0, 0.0, 0.0, c, 180.0), 0.0, "p.yaml"))

    for cell, lean in (("val_C5", 0.0), ("val_C6", 90.0)):
        meta_path = HERE.parent / "results" / cell / "run_meta.json"
        if not meta_path.exists():
            print(f"[NON TROVATO] {meta_path}")
            fails.append(cell)
            continue
        meta = json.loads(meta_path.read_text())
        yaw_deg = float(meta["spawn_yaw_deg"])
        # run_meta.json non contiene i roll/pitch/yaw di spawn (NON TROVATO): confronto con tilt_values alla stessa
        # (alpha, lean, yaw0) di run_meta e con l'asse registrato (R_spawn.z deve coincidere con 'axis').
        tv = s1_run.tilt_values(float(meta["axis_tilt_deg"]), float(meta["axis_lean_deg"]), yaw_deg)
        R = s1_run.rpy_extrinsic_to_matrix(*(float(s) for s in tv["spawn_rpy_str"]))
        ax = R @ np.array([0.0, 0.0, 1.0])
        dev = float(np.max(np.abs(ax - np.array(meta["axis"]))))
        check(f"{cell}: R_spawn.z == run_meta axis", dev < 1e-12, f"(max dev {dev:.2e})")
        # fase al contatto scelta in modo che yaw0 = spawn_yaw_deg del run
        omega, tc = -2.0, 20.0
        w = compute(omega, tc, 20.0, lean, c, yaw_deg + omega * tc)
        exp = tuple(round(float(s), NODE_DECIMALS) for s in tv["spawn_rpy_str"])
        got = (w["roll"], w["pitch"], w["yaw"])
        diff = max(abs(a - b) for a, b in zip(got, exp))
        check(f"{cell}: roll/pitch/yaw == tilt_values", diff < 2e-6, f"(got {got}, exp {exp})")
        check(f"{cell}: asse effettivo entro 1e-4 deg", w["axis_dev_deg"] < 1e-4, f"({w['axis_dev_deg']:.2e} deg)")
    print("SELFTEST", "FAILED: " + ", ".join(fails) if fails else "PASSED")
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--omega-deg-s", type=float, help="velocita' angolare [deg/s], negativa")
    ap.add_argument("--tc-s", type=float,
                    help="tempo dalla 1a alla 2a pressione [s] = sync_delay_sec + tempo di registrazione fino al trigger")
    ap.add_argument("--alpha-deg", type=float, default=0.0, help="inclinazione dell'asse [deg] (default 0)")
    ap.add_argument("--lean-deg", type=float, default=0.0, help="direzione phi dell'inclinazione [deg] (default 0)")
    ap.add_argument("--center-xyz", type=s1_run.parse_center_xyz, default=s1_run.parse_center_xyz(DEFAULT_CENTER),
                    help=f"centro X,Y,Z (default {DEFAULT_CENTER})")
    ap.add_argument("--contact-phase-deg", type=float, default=180.0, help="fase al contatto [deg] (default 180)")
    ap.add_argument("--params-file", default=DEFAULT_PARAMS_FILE, help="params.yaml nel container")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)
    if a.selftest:
        return selftest()
    if a.omega_deg_s is None or a.tc_s is None:
        ap.error("--omega-deg-s e --tc-s sono obbligatori (o --selftest)")
    try:
        if a.alpha_deg < 0.0:
            raise ValueError("--alpha-deg deve essere >= 0")
        s1_run.validate_tilt(a.alpha_deg, a.lean_deg)
        v = compute(a.omega_deg_s, a.tc_s, a.alpha_deg, a.lean_deg, a.center_xyz, a.contact_phase_deg)
    except (ValueError, s1_run.RunError) as e:
        print(f"errore: {e}", file=sys.stderr)
        return 2
    print(format_command(v, a.alpha_deg, a.params_file))
    print()
    print(f"# omega = {a.omega_deg_s:g} deg/s, T_c = {a.tc_s:g} s, fase al contatto = {a.contact_phase_deg:g} deg"
          f" -> yaw0 = {math.degrees(v['yaw0_rad']):.4f} deg ({v['yaw0_rad']:.6f} rad)")
    print(f"# asse di rotazione (mondo) = [{v['axis'][0]:.9f}, {v['axis'][1]:.9f}, {v['axis'][2]:.9f}]"
          f"  (alpha={a.alpha_deg:g}, lean={a.lean_deg:g})")
    if a.alpha_deg > 0.0:
        print(f"# asse effettivo dai valori arrotondati a {NODE_DECIMALS} decimali: scarto "
              f"{v['axis_dev_deg']:.2e} deg dal nominale")
        print("# ATTENZIONE: con alpha > 0 la 'fase al contatto' e' la rotazione attorno all'asse INCLINATO "
              "(Rz(yaw0) nel frame del corpo, R_spawn = R_tilt.Rz(yaw0)), NON l'angolo attorno a +z del mondo.")
    print(f"# nota: l'orchestratore passa wz con 6 decimali (std::to_string): wz = {v['wz']:.9f} diventa "
          f"{round(v['wz'], 6):.6f} rad/s ({math.degrees(round(v['wz'], 6)):.6f} deg/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
