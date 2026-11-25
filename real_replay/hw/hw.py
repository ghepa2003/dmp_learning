#!/usr/bin/env python3
"""hw.py - strumenti utente per il set-up a due Franka (stadio 1: robot 2 "satellite").

Sottocomandi:
  status [--sat] [--no-robot]   stato dei due robot (echo_robot_state) e, con --sat, pacchetti UDP del satellite_node
  ik [--psi0-deg X]             q del robot 2 da centro, inclinazione asse e psi0 (offline, senza robot)
  reset [--robot 1|2|both] [--yes]   porta i robot nella posa di partenza (joint_point_to_point_motion, uno alla volta)
  plan [--grasp-command F] [--weights F]   fattibilita' temporale/di finestra del GraspCommand, senza muovere nulla
  run [--dry-run] [--grasp-command F] [--weights F]   stadio 2: plan, posa robot, satellite_node, real_replay --mode task
  run --only-satellite [--dry-run] [--max-travel-deg D]   stadio 1: SOLO satellite_node, fase/omega live

Unico file di configurazione: hw/hw_config.yaml (--config per cambiarlo). Formati e protocollo: hw/README.md.
Dipendenze: python3 stdlib, PyYAML, numpy. Nessun ROS.
"""

import argparse
import hashlib
import json
import math
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "hw_config.yaml")
DEG = math.pi / 180.0

# ---- protocollo UDP (identico a include/real_replay/satellite_udp.hpp) --------------------------------------------
STATE_FMT = "<IHHIIQ5d3d3dII"
STATE_SIZE = struct.calcsize(STATE_FMT)  # 120
CMD_FMT = "<IHHIIQ"
CMD_SIZE = struct.calcsize(CMD_FMT)  # 24
STATE_MAGIC = 0x53544153  # 'SATS'
CMD_MAGIC = 0x43544153  # 'SATC'
PROTOCOL_VERSION = 2  # v2: campo flags (bit 0 omega_valid) al posto di reserved
FLAG_OMEGA_VALID = 1 << 0
CMD_HEARTBEAT, CMD_START, CMD_STOP = 1, 2, 3
STATE_NAMES = {0: "INIT", 1: "WAIT_START", 2: "RUNNING", 3: "STOPPING", 4: "HOLD", 5: "DONE"}
FAULT_NAMES = {0: "window", 1: "heartbeat", 2: "tracking", 3: "safety", 4: "phase_mismatch(warn)", 5: "stop_command",
               6: "signal", 7: "non_finite", 8: "start_timeout", 9: "log_full"}
assert STATE_SIZE == 120 and CMD_SIZE == 24

# exit code di satellite_node (stessa semantica di satellite_j7)
NODE_EXIT = {0: "completato (fine finestra) o STOP comandato, fine ordinata",
             1: "errore runtime / preflight fallito (nessun movimento o movimento non iniziato)",
             2: "argomenti o config rifiutati",
             3: "fermato (Ctrl-C / heartbeat / tracking / finestra / safety / START mancato) dopo rampa controllata",
             4: "rampa di stop NON completata (errore libfranka o stato non finito): il robot puo' essere in reflex"}

# ---- cinematica Panda (DH Craig/Franka, come satellite_phase.hpp) --------------------------------------------------
DH_A = [0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0]
DH_D = [0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107]
DH_ALPHA = [0.0, -math.pi / 2, math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, math.pi / 2, 0.0]
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])


class HwError(Exception):
    """Errore di configurazione / ingresso: exit 2."""


def dh_frames(q):
    """Lista delle 9 trasformate cumulative (base, giunti 1..7, flangia) nel frame base dello stesso robot."""
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(8):
        th = q[i] if i < 7 else 0.0
        ca, sa, ct, st = math.cos(DH_ALPHA[i]), math.sin(DH_ALPHA[i]), math.cos(th), math.sin(th)
        A = np.array([[ct, -st, 0.0, DH_A[i]],
                      [st * ca, ct * ca, -sa, -sa * DH_D[i]],
                      [st * sa, ct * sa, ca, ca * DH_D[i]],
                      [0.0, 0.0, 0.0, 1.0]])
        T = T @ A
        frames.append(T.copy())
    return frames


def fk_flange(q):
    return dh_frames(q)[-1]


def jacobian(q):
    """Jacobiano geometrico 6x7 della flangia (velocita' lineare dell'origine flangia, angolare), frame base."""
    fr = dh_frames(q)
    p = fr[-1][:3, 3]
    J = np.zeros((6, 7))
    for i in range(7):
        z = fr[i + 1][:3, 2]  # asse del giunto i+1 (modified DH: z del frame i+1)
        o = fr[i + 1][:3, 3]
        J[:3, i] = np.cross(z, p - o)
        J[3:, i] = z
    return J


def rot_log(R):
    """Vettore asse-angolo di R."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-12:
        return np.zeros(3)
    if math.pi - ang < 1e-6:
        w, V = np.linalg.eig(R)
        ax = np.real(V[:, np.argmin(np.abs(w - 1.0))])
        return ax / np.linalg.norm(ax) * ang
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * math.sin(ang)) * ang


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap_branch(x_deg, lo=-270.0):
    """Angolo in [lo, lo+360)."""
    y = x_deg - 360.0 * math.floor((x_deg - lo) / 360.0)
    if y >= lo + 360.0:
        y -= 360.0
    return y


def wrap_pi_deg(x_deg):
    """(-180, 180]."""
    y = wrap_branch(x_deg, -180.0)
    return 180.0 if y == -180.0 else y


# ---- config --------------------------------------------------------------------------------------------------------
def load_config(path):
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f)
    except OSError as e:
        raise HwError("config '%s' non leggibile: %s" % (path, e))
    if not isinstance(cfg, dict):
        raise HwError("config '%s' non e' una mappa YAML" % path)
    cfg["_dir"] = os.path.dirname(os.path.abspath(path))
    cfg["_path"] = os.path.abspath(path)
    for sec in ("scene", "mockup", "robot1", "robot2", "udp", "timing", "paths", "planner", "plan"):
        if sec not in cfg:
            raise HwError("config: sezione '%s' mancante" % sec)
    return cfg


def cfg_path(cfg, p):
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(cfg["_dir"], p))


def cli_or_cfg_path(cfg, cli_value, cfg_value):
    """Percorso da riga di comando (relativo alla cartella corrente) oppure dalla config (relativo alla config)."""
    return os.path.abspath(cli_value) if cli_value else cfg_path(cfg, cfg_value)


def axis_from_tilt(tilt_deg):
    return rot_y(tilt_deg[1] * DEG) @ rot_x(tilt_deg[0] * DEG) @ np.array([0.0, 0.0, 1.0])


def base2(cfg):
    b = cfg["scene"]["base2_in_base1"]
    return rot_z(float(b["yaw_deg"]) * DEG), np.array(b["t_m"], dtype=float)


def sat_geom(cfg, q2):
    """Geometria del satellite dalla q del robot 2: flangia, centro, asse (frame 1), theta da flangia e da q7 [deg]."""
    sc = cfg["scene"]
    Rb, tb = base2(cfg)
    F = fk_flange(q2)
    R1 = Rb @ F[:3, :3]
    p1 = Rb @ F[:3, 3] + tb
    z = R1[:, 2]
    x_angle = math.degrees(math.atan2(R1[1, 0], R1[0, 0]))
    return {
        "flange1": p1, "flange2": F[:3, 3], "R1": R1,
        "center": p1 + float(sc["flange_to_center_m"]) * z,
        "axis": -z,
        "theta_flange": wrap_branch(x_angle + float(sc["collar_phase_offset_deg"])),
        "theta_q7": wrap_branch(float(sc["q7_phase"]["offset_deg"]) + float(sc["q7_phase"]["sign"]) * math.degrees(q2[6])),
    }


# ---- IK ------------------------------------------------------------------------------------------------------------
def ik_target(cfg, psi0_deg):
    """Posa obiettivo della flangia del robot 2 (frame 2) da centro, tilt e psi0."""
    sc = cfg["scene"]
    axis = axis_from_tilt(sc["axis_tilt_deg"])
    z = -axis
    c = np.array(sc["center_m"], dtype=float)
    p1 = c - float(sc["flange_to_center_m"]) * z
    a = (psi0_deg - float(sc["collar_phase_offset_deg"])) * DEG  # x_angle desiderato
    xh = np.array([math.cos(a), math.sin(a), 0.0])
    x = xh - xh.dot(z) * z
    if np.linalg.norm(x) < 1e-6:
        raise HwError("ik: asse quasi orizzontale, x_angle indefinito")
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R1 = np.column_stack([x, y, z])
    Rb, tb = base2(cfg)
    return Rb.T @ R1, Rb.T @ (p1 - tb), axis


NULL_WEIGHTS = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0])


def solve_ik(R_des, p_des, q_seed, iters=2000, k_null=0.5, w_null=NULL_WEIGHTS):
    """Minimi quadrati smorzati + termine nello spazio nullo verso il seme, q dentro i limiti.

    Il termine di spazio nullo e' pesato con w_null: default J1-J6 verso il seme, J7 libero (J7 e' la fase del
    satellite: cosi' una variazione di psi0 finisce su q7 e non viene spalmata sulla ridondanza J1/J3/J5)."""
    q = np.array(q_seed, dtype=float)
    seed = q.copy()
    for _ in range(iters):
        F = fk_flange(q)
        e = np.concatenate([p_des - F[:3, 3], rot_log(R_des @ F[:3, :3].T)])
        en = np.linalg.norm(e)
        if en < 1e-12 and np.linalg.norm(w_null * (seed - q)) < 1e-9:
            break
        J = jacobian(q)
        lam = 1e-4 + min(0.05, en)
        JJt = J @ J.T + lam ** 2 * np.eye(6)
        Jp = J.T @ np.linalg.inv(JJt)
        dq = Jp @ e + (np.eye(7) - Jp @ J) @ (k_null * w_null * (seed - q))
        n = np.linalg.norm(dq)
        if n > 0.2:
            dq *= 0.2 / n
        q = np.clip(q + dq, Q_MIN, Q_MAX)
    F = fk_flange(q)
    pos_err = float(np.linalg.norm(p_des - F[:3, 3]))
    rot_err = float(np.linalg.norm(rot_log(R_des @ F[:3, :3].T)))
    return q, pos_err, rot_err


def compute_ik(cfg, psi0_deg=None):
    r2 = cfg["robot2"]
    psi0 = float(r2["start"]["psi0_deg"]) if psi0_deg is None else float(psi0_deg)
    R_des, p_des, axis = ik_target(cfg, psi0)
    seed = np.array(r2["ik_seed_deg"], dtype=float) * DEG
    q, pe, re = solve_ik(R_des, p_des, seed)
    margins = np.minimum(q - Q_MIN, Q_MAX - q) / DEG
    lim = float(r2["ik_limit_margin_deg"])
    ok = bool(margins.min() >= lim and pe <= 1e-4 and re <= 0.01 * DEG)
    return {"psi0": psi0, "q": q, "pos_err": pe, "rot_err": re, "margins_deg": margins, "seed": seed, "ok": ok,
            "axis": axis, "p_des2": p_des, "limit_margin_deg": lim}


def robot2_start_q(cfg):
    """(q [rad], descrizione). mode q: q_deg; mode center_tilt: IK (HwError se rifiutata)."""
    st = cfg["robot2"]["start"]
    if st["mode"] == "q":
        return np.array(st["q_deg"], dtype=float) * DEG, "robot2.start.q_deg"
    if st["mode"] == "center_tilt":
        r = compute_ik(cfg)
        if not r["ok"]:
            raise HwError("IK della partenza del robot 2 rifiutata (esegui 'hw.py ik' per i dettagli)")
        return r["q"], "IK (center_tilt, psi0 %.2f deg)" % r["psi0"]
    raise HwError("robot2.start.mode deve essere 'q' o 'center_tilt'")


def fmt_q(q):
    return "[" + " ".join("%8.3f" % math.degrees(x) for x in q) + "] deg"


def cmd_ik(args, cfg):
    r = compute_ik(cfg, args.psi0_deg)
    r2 = cfg["robot2"]
    print("IK robot 2 (offline): centro %s m, tilt %s deg -> asse [%.5f %.5f %.5f], psi0 %.3f deg"
          % (cfg["scene"]["center_m"], cfg["scene"]["axis_tilt_deg"], r["axis"][0], r["axis"][1], r["axis"][2], r["psi0"]))
    print("  flangia obiettivo (frame 2): [%.5f %.5f %.5f] m" % tuple(r["p_des2"]))
    print("  q   = %s" % fmt_q(r["q"]))
    print("  margini dai limiti [deg]: " + " ".join("J%d %.1f" % (i + 1, m) for i, m in enumerate(r["margins_deg"]))
          + "   (min %.1f, richiesto >= %.1f)" % (r["margins_deg"].min(), r["limit_margin_deg"]))
    print("  errore di posa: %.4f mm, %.5f deg" % (r["pos_err"] * 1e3, r["rot_err"] / DEG))
    d = (r["q"] - r["seed"]) / DEG
    print("  distanza dal seme: max |dq| %.3f deg (J%d), norma %.3f deg" % (np.abs(d).max(), int(np.argmax(np.abs(d))) + 1, np.linalg.norm(d)))
    g = sat_geom(cfg, r["q"])
    print("  verifica FK: centro [%.5f %.5f %.5f] m (frame 1), theta da flangia %.3f deg, da q7 %.3f deg"
          % (g["center"][0], g["center"][1], g["center"][2], g["theta_flange"], g["theta_q7"]))
    if "validated_q_deg" in r2:
        qv = np.array(r2["validated_q_deg"], dtype=float) * DEG
        dv = (r["q"] - qv) / DEG
        pv = fk_flange(qv)[:3, 3]
        print("  vs posa validata: max |dq| J1-J6 %.4f deg, dq7 %.3f deg (= psi validata - psi0), |flangia - flangia validata| %.4f mm"
              % (np.abs(dv[:6]).max(), dv[6], np.linalg.norm(fk_flange(r["q"])[:3, 3] - pv) * 1e3))
    if r["ok"]:
        print("ESITO: OK")
        return 0
    why = []
    if r["margins_deg"].min() < r["limit_margin_deg"]:
        why.append("margine dal limite < %.1f deg" % r["limit_margin_deg"])
    if r["pos_err"] > 1e-4:
        why.append("errore di posizione > 0.1 mm")
    if r["rot_err"] > 0.01 * DEG:
        why.append("errore di orientamento > 0.01 deg")
    print("ESITO: RIFIUTATA (%s)" % "; ".join(why))
    return 1


# ---- echo_robot_state -----------------------------------------------------------------------------------------------
def parse_robot_state(text):
    """Primo oggetto JSON (una riga che inizia con '{') dell'output di echo_robot_state."""
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except ValueError:
                # nan/inf non sono JSON: sostituisci e riprova
                fixed = line.replace("-nan", "NaN").replace("nan", "NaN").replace("-inf", "-Infinity").replace("inf", "Infinity")
                return json.loads(fixed)
    raise HwError("nessuno stato JSON nell'output di echo_robot_state")


def col_major(a):
    return np.array(a, dtype=float).reshape(4, 4).T


def read_robot_state(cfg, ip, timeout=8.0):
    exe = os.path.join(cfg_path(cfg, cfg["paths"]["libfranka_examples_dir"]), "echo_robot_state")
    if not os.access(exe, os.X_OK):
        raise HwError("echo_robot_state non trovato: %s" % exe)
    try:
        r = subprocess.run([exe, ip], stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise HwError("echo_robot_state %s: timeout dopo %.0f s (robot spento / rete?)" % (ip, timeout))
    if r.returncode != 0:
        raise HwError("echo_robot_state %s: exit %d: %s" % (ip, r.returncode, (r.stdout + r.stderr).strip()[-300:]))
    return parse_robot_state(r.stdout)


def flange_from_state(s):
    """O_T_F = O_T_EE * F_T_EE^-1 (frame base del robot)."""
    return col_major(s["O_T_EE"]) @ np.linalg.inv(col_major(s["F_T_EE"]))


# ---- UDP -----------------------------------------------------------------------------------------------------------
def decode_state(buf):
    if len(buf) != STATE_SIZE:
        return None
    v = struct.unpack(STATE_FMT, buf)
    if v[0] != STATE_MAGIC or v[1] != PROTOCOL_VERSION or v[2] != STATE_SIZE:
        return None
    return {"seq": v[3], "state": v[4], "t_mono_ns": v[5], "theta": v[6], "omega_meas": v[7], "theta_q7": v[8], "q7": v[9],
            "omega_cmd": v[10], "center": v[11:14], "axis": v[14:17], "faults": v[17], "flags": v[18],
            "omega_valid": bool(v[18] & FLAG_OMEGA_VALID)}


def encode_command(ctype, seq):
    return struct.pack(CMD_FMT, CMD_MAGIC, PROTOCOL_VERSION, ctype, seq, 0, time.monotonic_ns())


def faults_str(f):
    names = [FAULT_NAMES[b] for b in FAULT_NAMES if f & (1 << b)]
    return ",".join(names) if names else "-"


def state_line(p, t_age_s=None):
    age = (time.monotonic_ns() - p["t_mono_ns"]) * 1e-9
    om = p["omega_meas"]
    # omega_meas mostrata solo se omega_valid (il flag del nodo), altrimenti "n.v."
    om_s = "%8.4f" % math.degrees(om) if p["omega_valid"] else "  n.v.  "
    return ("%-10s theta %8.3f deg  (q7: %8.3f)  omega %s deg/s (cmd %7.3f)  q7 %8.3f deg  age %5.0f ms%s  faults %s"
            % (STATE_NAMES.get(p["state"], str(p["state"])), math.degrees(p["theta"]), math.degrees(p["theta_q7"]),
               om_s, math.degrees(p["omega_cmd"]), math.degrees(p["q7"]),
               age * 1e3, " STALE" if (t_age_s is not None and age > t_age_s) else "", faults_str(p["faults"])))


def listen(host, port, duration_s):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.settimeout(0.1)
    out, bad = [], 0
    t_end = time.monotonic() + duration_s
    try:
        while time.monotonic() < t_end:
            try:
                buf = s.recv(512)
            except socket.timeout:
                continue
            p = decode_state(buf)
            if p is None:
                bad += 1
            else:
                p["t_rx"] = time.monotonic()
                out.append(p)
    finally:
        s.close()
    return out, bad


# ---- status ----------------------------------------------------------------------------------------------------------
def cmd_status(args, cfg):
    rc = 0
    if not args.no_robot:
        targets = [(1, cfg["robot1"]["ip"], np.array(cfg["robot1"]["start_q_rad"], dtype=float), float(cfg["robot1"]["tol_deg"]), "robot1.start_q_rad")]
        try:
            q2, src = robot2_start_q(cfg)
            targets.append((2, cfg["robot2"]["ip"], q2, float(cfg["robot2"]["tol"]["q_deg"]), src))
        except HwError as e:
            print("robot 2: posa di partenza non calcolabile: %s" % e)
            targets.append((2, cfg["robot2"]["ip"], None, 0.0, ""))
            rc = 1
        for n, ip, q_start, tol, src in targets:
            print("== robot %d (%s) ==" % (n, ip))
            try:
                s = read_robot_state(cfg, ip)
            except HwError as e:
                print("  NON LEGGIBILE: %s" % e)
                rc = 1
                continue
            q = np.array(s["q"], dtype=float)
            OF = flange_from_state(s)
            print("  robot_mode: %s   errori correnti: %s" % (s.get("robot_mode"), s.get("current_errors")))
            print("  q = %s" % fmt_q(q))
            print("  flangia (O_T_EE*F_T_EE^-1, frame base %d): [%.4f %.4f %.4f] m   FK DH: [%.4f %.4f %.4f] m"
                  % ((n,) + tuple(OF[:3, 3]) + tuple(fk_flange(q)[:3, 3])))
            print("  m_ee %.3f kg, m_load %.3f kg" % (s.get("m_ee", float("nan")), s.get("m_load", float("nan"))))
            if n == 2:
                g = sat_geom(cfg, q)
                print("  satellite (frame 1): centro [%.4f %.4f %.4f] m, theta flangia %.2f deg, theta q7 %.2f deg"
                      % (g["center"][0], g["center"][1], g["center"][2], g["theta_flange"], g["theta_q7"]))
            if q_start is not None:
                d = np.abs(q - q_start) / DEG
                ok = d.max() <= tol
                print("  distanza dalla partenza (%s): max |dq| %.3f deg (J%d) -> %s (tol %.2f deg)"
                      % (src, d.max(), int(np.argmax(d)) + 1, "IN POSA" if ok else "FUORI POSA", tol))
    if args.sat:
        port = int(cfg["udp"]["monitor_port"]) or int(cfg["udp"]["publish_port"])
        print("== satellite_node: ascolto %s:%d per 2 s ==" % (cfg["udp"]["host"], port))
        try:
            pk, bad = listen(cfg["udp"]["host"], port, 2.0)
        except OSError as e:
            print("  porta non disponibile: %s" % e)
            return 1
        if not pk:
            print("  nessun pacchetto (satellite_node non attivo?)%s" % ("; %d datagrammi non validi" % bad if bad else ""))
        else:
            span = pk[-1]["t_rx"] - pk[0]["t_rx"]
            rate = (len(pk) - 1) / span if span > 0 else float("nan")
            lost = pk[-1]["seq"] - pk[0]["seq"] + 1 - len(pk)
            print("  %d pacchetti, %.1f Hz, persi %d, non validi %d" % (len(pk), rate, lost, bad))
            print("  ultimo: " + state_line(pk[-1], float(cfg["udp"]["T_age_s"])))
    return rc


# ---- reset -----------------------------------------------------------------------------------------------------------
BAD_MODES = {"Reflex": "il robot e' in Reflex: esegui l'error recovery (Desk) prima del reset",
             "User stopped": "user stop premuto: rilascialo prima del reset",
             "Other": "modo 'Other' (FCI non attivo / freni?): controlla Desk"}


def confirm(msg):
    if not sys.stdin.isatty():
        return False
    try:
        return input(msg + " [s/N] ").strip().lower() in ("s", "si", "y", "yes")
    except EOFError:
        return False


def run_child_forwarding_sigint(argv, stdin_text=None):
    """Lancia un figlio in una sessione propria; Ctrl-C = un solo SIGINT inoltrato, poi attesa (SIGKILL solo dopo 15 s)."""
    p = subprocess.Popen(argv, stdin=subprocess.PIPE if stdin_text is not None else subprocess.DEVNULL, start_new_session=True)
    if stdin_text is not None:
        try:
            p.stdin.write(stdin_text.encode())
            p.stdin.close()
        except BrokenPipeError:
            pass
    return wait_child(p)


def wait_child(p, on_tick=None):
    """Attende il figlio. Ctrl-C (anche durante on_tick): UN SIGINT al figlio, poi attesa; SIGKILL solo dopo 15 s."""
    interrupted_at = None
    while True:
        try:
            try:
                return p.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                if on_tick is not None:
                    on_tick()
        except KeyboardInterrupt:
            if interrupted_at is None:
                interrupted_at = time.monotonic()
                print("\nCtrl-C: SIGINT al processo figlio, attendo la sua rampa di stop (max 15 s)...")
                try:
                    p.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
            else:
                print("\n(attendo ancora la rampa di stop del figlio)")
        if interrupted_at is not None and time.monotonic() - interrupted_at > 15.0 and p.poll() is None:
            print("il figlio non e' terminato in 15 s dopo SIGINT: SIGKILL")
            p.kill()


def cmd_reset(args, cfg):
    robots = {"1": [1], "2": [2], "both": [1, 2]}[args.robot]
    exe = os.path.join(cfg_path(cfg, cfg["paths"]["libfranka_examples_dir"]), "joint_point_to_point_motion")
    if not os.access(exe, os.X_OK):
        raise HwError("joint_point_to_point_motion non trovato: %s" % exe)
    if not args.yes and not sys.stdin.isatty():
        print("reset: stdin non e' un terminale e --yes non dato: rifiuto di muovere.")
        return 1
    for n in robots:
        sec = cfg["robot%d" % n]
        ip = sec["ip"]
        if n == 1:
            q_goal, src = np.array(sec["start_q_rad"], dtype=float), "robot1.start_q_rad"
        else:
            q_goal, src = robot2_start_q(cfg)
        speed = float(sec["reset_speed"])
        if not (0.0 < speed <= 1.0):
            raise HwError("robot%d.reset_speed deve essere in (0, 1]" % n)
        print("== reset robot %d (%s) ==" % (n, ip))
        s = read_robot_state(cfg, ip)
        mode = s.get("robot_mode")
        if mode != "Idle":
            print("  RIFIUTATO: robot_mode = %s. %s" % (mode, BAD_MODES.get(mode, "serve 'Idle' per muovere")))
            return 1
        q = np.array(s["q"], dtype=float)
        if np.any(q_goal < Q_MIN) or np.any(q_goal > Q_MAX):
            print("  RIFIUTATO: q obiettivo fuori dai limiti di giunto")
            return 1
        d = (q_goal - q) / DEG
        print("  q attuale  = %s" % fmt_q(q))
        print("  q obiettivo= %s  (%s)" % (fmt_q(q_goal), src))
        print("  max |dq| %.2f deg (J%d), speed-factor %.3f" % (np.abs(d).max(), int(np.argmax(np.abs(d))) + 1, speed))
        if n == 2:
            print("  NOTA: joint_point_to_point_motion non imposta il carico: il mockup (%.2f kg) non e' noto al robot durante il reset"
                  % float(cfg["mockup"]["mass_kg"]))
        if not args.yes and not confirm("  Muovere il robot %d? Tieni a portata di mano lo user stop." % n):
            print("  annullato.")
            return 1
        argv = [exe, ip] + ["%.6f" % x for x in q_goal] + ["%.4f" % speed]
        # l'esempio aspetta Invio dopo il proprio avviso: lo diamo noi, dopo la nostra conferma
        rc = run_child_forwarding_sigint(argv, stdin_text="\n")
        if rc != 0:
            print("  joint_point_to_point_motion: exit %d -> interrompo" % rc)
            return 1
        s = read_robot_state(cfg, ip)
        dd = np.abs(np.array(s["q"], dtype=float) - q_goal) / DEG
        print("  dopo il reset: max |dq| %.3f deg, robot_mode %s" % (dd.max(), s.get("robot_mode")))
    return 0


# ---- plan ------------------------------------------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def load_yaml(path, what):
    try:
        with open(path) as f:
            d = yaml.safe_load(f)
    except OSError as e:
        raise HwError("%s '%s' non leggibile: %s" % (what, path, e))
    if not isinstance(d, dict):
        raise HwError("%s '%s' non e' una mappa YAML" % (what, path))
    return d


def check_poses(cmd):
    """start_pose / goal_pose: {position_m: [3], quaternion_wxyz: [4]} finiti, |q| = 1 +- 1e-3."""
    det = []
    for key in ("start_pose", "goal_pose"):
        p = cmd.get(key)
        if not isinstance(p, dict):
            return False, "%s mancante o null" % key
        try:
            pos = [float(x) for x in p["position_m"]]
            q = [float(x) for x in p["quaternion_wxyz"]]
        except (KeyError, TypeError, ValueError):
            return False, "%s: position_m [3] e quaternion_wxyz [4] richiesti" % key
        if len(pos) != 3 or len(q) != 4 or not all(math.isfinite(x) for x in pos + q):
            return False, "%s: valori non finiti o dimensioni errate" % key
        n = math.sqrt(sum(x * x for x in q))
        if abs(n - 1.0) > 1e-3:
            return False, "%s: |q| = %.6f" % (key, n)
        det.append("%s [%.4f %.4f %.4f]" % (key, pos[0], pos[1], pos[2]))
    return True, ", ".join(det) + ", k %s, wrist_flip %s" % (cmd.get("k"), cmd.get("wrist_flip"))


def check_robot1_reach(cfg, cmd, plan):
    """PUNTO DI ESTENSIONE (stadio 2): portata del robot 1 lungo contatto + follow. None = non verificata."""
    return None


def plan_feasibility(cfg, cmd, weights_path, theta0_deg=None):
    """Controlli di fattibilita'. Ritorna (ok, checks, info). checks: lista di (nome, esito, dettaglio, bloccante)."""
    checks = []
    info = {}

    def add(name, ok, detail, blocking=True):
        checks.append((name, bool(ok), detail, blocking))

    sc, tm, pl = cfg["scene"], cfg["timing"], cfg["plan"]
    status = str(cmd.get("status", "")).upper()
    add("status", status == "SELECTED", "status = %s" % status)
    add("format grasp_command_v2", cmd.get("format") == "grasp_command_v2", "format = %s" % cmd.get("format"))
    add("frame_id world", cmd.get("frame_id") == "world", "frame_id = %s" % cmd.get("frame_id"))
    poses_ok, pd = check_poses(cmd)
    add("start_pose e goal_pose presenti, finiti, quaternioni unitari", poses_ok, pd)
    for k in ("theta_star_deg", "contact_time_s", "omega_deg_s", "weights_sha256"):
        if k not in cmd:
            raise HwError("grasp command: chiave '%s' mancante" % k)
    w_cmd = float(cmd["omega_deg_s"])
    w_cfg = float(sc["omega_deg_s"])
    sha = sha256_file(weights_path)
    add("sha256(weights) == weights_sha256", sha == str(cmd["weights_sha256"]).lower(), "file %s, comando %s" % (sha[:16] + "...", str(cmd["weights_sha256"])[:16] + "..."))
    weights = load_yaml(weights_path, "weights")
    if "tau" not in weights:
        raise HwError("weights: chiave 'tau' mancante (stessa chiave di prodmp_io / ProDmpSource)")
    tau = float(weights["tau"])
    info["tau"] = tau
    add("contact_time_s == tau dei pesi", abs(float(cmd["contact_time_s"]) - tau) <= float(pl["tau_tol_s"]),
        "contact_time_s %.6f, tau %.6f (tol %.1e)" % (float(cmd["contact_time_s"]), tau, float(pl["tau_tol_s"])))
    add("omega comando coerente con scene.omega_deg_s", abs(w_cmd - w_cfg) <= float(pl["omega_tol_deg_s"]),
        "comando %.3f, config %.3f deg/s (tol %.2f)" % (w_cmd, w_cfg, float(pl["omega_tol_deg_s"])))
    add("omega != 0", w_cmd != 0.0, "omega %.3f deg/s" % w_cmd)
    dom = [float(x) for x in pl["omega_domain_deg_s"]]
    add("E: |omega| nel dominio %s deg/s" % dom, any(abs(abs(w_cmd) - d) <= float(pl["omega_tol_deg_s"]) for d in dom),
        "|omega| %.3f deg/s" % abs(w_cmd), blocking=False)

    lo, hi = [float(x) for x in sc["psi_window_deg"]]
    m = float(cfg["robot2"]["window_margin_deg"])
    if theta0_deg is None:
        q2, src = robot2_start_q(cfg)
        theta0 = sat_geom(cfg, q2)["theta_flange"]
    else:
        theta0, src = float(theta0_deg), "argomento"
    info.update(theta0=theta0, theta0_src=src, window=(lo, hi), margin=m)
    add("C: theta0 dentro la finestra con margine", lo + m <= theta0 <= hi - m, "theta0 %.3f deg (%s), ammesso [%.1f, %.1f]" % (theta0, src, lo + m, hi - m))
    if w_cmd == 0.0:
        return all(c[1] for c in checks if c[3]), checks, info

    s = 1.0 if w_cmd > 0 else -1.0
    T_c = tau + float(tm["contact_delay_s"])
    theta_star = wrap_branch(float(cmd["theta_star_deg"]))
    theta_trig_wrap = wrap_pi_deg(theta_star - w_cmd * T_c)
    theta_trig = wrap_branch(theta_trig_wrap)  # confronti sul ramo [-270, 90)
    info.update(s=s, T_c=T_c, theta_star=theta_star, theta_trig=theta_trig, theta_trig_wrapPi=theta_trig_wrap)
    arm = abs(w_cmd) * float(tm["T_arm_min_s"])
    add("B: s*(theta_trig - theta0) >= |omega|*T_arm_min", s * (theta_trig - theta0) >= arm - 1e-9,
        "s*(theta_trig - theta0) = %.3f deg, richiesto >= %.3f deg" % (s * (theta_trig - theta0), arm))
    # Condizione A: prima che omega_valid diventi vero il satellite percorre |omega|*(ramp_s/2 + omega_window_s)
    # (rampa smoothstep: meta' angolo a pari tempo; poi una finestra intera di regressione a velocita' costante);
    # dopo serve ancora |omega|*margin_arm_s prima del trigger.
    ctl = cfg["robot2"]["control"]
    ang_valid = abs(w_cmd) * (float(ctl["ramp_s"]) / 2.0 + float(ctl["omega_window_s"]))
    need_a = ang_valid + abs(w_cmd) * float(tm["margin_arm_s"])
    info.update(angle_before_omega_valid=ang_valid, condition_a=need_a)
    add("A: s*(theta_trig - theta0) >= |omega|*(ramp_s/2 + omega_window_s + margin_arm_s)", s * (theta_trig - theta0) >= need_a - 1e-9,
        "s*(theta_trig - theta0) = %.3f deg, richiesto >= %.3f deg (%.3f prima di omega_valid + %.3f di margine)"
        % (s * (theta_trig - theta0), need_a, ang_valid, need_a - ang_valid))
    need = max(arm, need_a)
    # intervallo ammesso di theta0: finestra con margine AND armamento AND condizione A
    if s < 0:
        a_lo, a_hi = max(lo + m, theta_trig + need), hi - m
    else:
        a_lo, a_hi = lo + m, min(hi - m, theta_trig - need)
    info["theta0_interval"] = (a_lo, a_hi) if a_lo <= a_hi else None
    T_after = float(tm["T_close_max_s"]) + float(tm["T_follow_s"]) + float(tm["T_ramp_s"])
    theta_end = theta_star + w_cmd * T_after
    info["theta_end"] = theta_end
    add("D: theta_end = theta* + omega*(T_close_max+T_follow+T_ramp) dentro la finestra con margine", lo + m <= theta_end <= hi - m,
        "theta_end %.3f deg, ammesso [%.1f, %.1f]" % (theta_end, lo + m, hi - m))
    t_trig = (theta_trig - theta0) / w_cmd
    info["t_trigger_s"] = t_trig
    # timeout come real_replay --mode task: tempo dall'invio di START (rampa smoothstep: + ramp_s/2) + margine
    info["trigger_timeout_s"] = t_trig + float(ctl["ramp_s"]) / 2.0 + float(tm["trigger_timeout_margin_s"])
    info["robot1_reach"] = check_robot1_reach(cfg, cmd, info)
    return all(c[1] for c in checks if c[3]), checks, info


def cmd_plan(args, cfg):
    gc = cli_or_cfg_path(cfg, args.grasp_command, cfg["planner"]["grasp_command_yaml"])
    wp = cli_or_cfg_path(cfg, args.weights, cfg["planner"]["weights_yaml"])
    cmd = load_yaml(gc, "grasp command")
    weights = load_yaml(wp, "weights")
    print("plan: grasp command %s\n      weights       %s%s" % (gc, wp, "   [SINTETICO, solo test]" if weights.get("synthetic_test_only") else ""))
    ok, checks, info = plan_feasibility(cfg, cmd, wp, args.theta0_deg)
    if "theta_trig" in info:
        print("  s = %+d, tau %.3f s, contact_delay %.3f s -> T_c %.3f s" % (info["s"], info["tau"], info["T_c"] - info["tau"], info["T_c"]))
        print("  theta* %.3f deg -> theta_trig = wrapPi(theta* - omega*T_c) = %.3f deg (ramo [-270,90): %.3f deg)"
              % (info["theta_star"], info["theta_trig_wrapPi"], info["theta_trig"]))
        iv = info["theta0_interval"]
        print("  angolo percorso prima di omega_valid: %.3f deg; condizione A richiede s*(theta_trig - theta0) >= %.3f deg"
              % (info["angle_before_omega_valid"], info["condition_a"]))
        print("  intervallo ammesso di theta0: %s" % ("[%.3f, %.3f] deg" % iv if iv else "VUOTO"))
        inside = iv is not None and iv[0] <= info["theta0"] <= iv[1]
        print("  partenza configurata: theta0 %.3f deg (%s) -> %s" % (info["theta0"], info["theta0_src"], "OK" if inside else "NON OK"))
        print("  theta_end %.3f deg; trigger atteso dopo %.2f s, timeout trigger %.2f s" % (info["theta_end"], info["t_trigger_s"], info["trigger_timeout_s"]))
    for name, good, detail, blocking in checks:
        print("  [%s] %s: %s" % ("PASS" if good else ("FAIL" if blocking else "WARN"), name, detail))
    print("  portata robot 1: NON verificata" if info.get("robot1_reach") is None else "  portata robot 1: %s" % info["robot1_reach"])
    print("ESITO: %s" % ("FATTIBILE" if ok else "NON FATTIBILE"))
    return 0 if ok else 1


# ---- run -------------------------------------------------------------------------------------------------------------
class Heartbeat(threading.Thread):
    def __init__(self, host, port, hz):
        super().__init__(daemon=True)
        self.addr = (host, port)
        self.period = 1.0 / hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.lock = threading.Lock()
        self.active = threading.Event()
        self.quit = threading.Event()

    def send(self, ctype):
        with self.lock:
            self.seq += 1
            try:
                self.sock.sendto(encode_command(ctype, self.seq), self.addr)
            except OSError:
                pass

    def run(self):
        while not self.quit.wait(self.period):
            if self.active.is_set():
                self.send(CMD_HEARTBEAT)


def confirm_load(cfg):
    """Conferma digitata del carico del robot 2 (satellite_node con --yes salta la propria): stessa regola, 1 g."""
    mk = cfg["mockup"]
    mass = float(mk["mass_kg"])
    print("=" * 80)
    print("  CARICO DICHIARATO: %.3f kg, com %s m (frame flangia) -- il mockup e' montato?" % (mass, mk["com_m"]))
    print("=" * 80)
    try:
        typed = input("Digita la massa del mockup montato in kg (deve coincidere con %.3f kg entro 1 g): " % mass)
    except EOFError:
        typed = ""
    if not mass_confirmation_ok(typed, mass):
        print("carico NON confermato ('%s')." % typed.strip())
        return False
    return True


TASK_EXIT = {0: "HOLD_SAFE completato (stadio 2), STOP inviato al satellite",
             1: "INIT fallito (config, comando, preflight, socket): nessun comando al satellite",
             2: "annullato dall'operatore prima di prendere il controllo",
             3: "eccezione libfranka / controllo",
             5: "FAULT_NOMOTION: guasto prima dell'avvicinamento (STOP inviato, nessun movimento)",
             6: "FAULT_MOTION: guasto durante/dopo l'avvicinamento (target congelato, STOP inviato)"}


def robot_ready(cfg, n, q_goal, tol_deg, max_dq=0.01):
    """robot_mode Idle, in posa (max |dq| <= tol) e fermo (max |dq| <= max_dq rad/s). (ok, messaggio)."""
    s = read_robot_state(cfg, cfg["robot%d" % n]["ip"])
    mode = s.get("robot_mode")
    q = np.array(s["q"], dtype=float)
    dq = np.abs(np.array(s["dq"], dtype=float)).max() if "dq" in s else float("nan")
    d = np.abs(q - q_goal).max() / DEG
    ok = mode == "Idle" and d <= tol_deg and dq <= max_dq
    return ok, "robot %d: mode %s, max |q - partenza| %.3f deg (tol %.2f), max |dq| %.4f rad/s%s" % (
        n, mode, d, tol_deg, dq, "" if mode == "Idle" else " (" + BAD_MODES.get(mode, "serve Idle") + ")")


def wait_state(sock, state, child, timeout_s):
    """Ultimo pacchetto ricevuto; None se il figlio esce o scade il timeout prima di vedere `state`."""
    t_end = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < t_end:
        if child.poll() is not None:
            return None
        r, _, _ = select.select([sock], [], [], 0.1)
        if r:
            p = decode_state(sock.recv(512))
            if p is not None:
                last = p
                if p["state"] == state:
                    return p
    return None


def cmd_run_full(args, cfg):
    """Sequenza completa dello stadio 2: plan, posa robot, satellite_node, real_replay --mode task."""
    build = cfg_path(cfg, cfg["paths"]["build_dir"])
    sat_exe = os.path.join(build, "satellite_node")
    rr_exe = os.path.join(build, "real_replay")
    for exe in (sat_exe, rr_exe):
        if not os.access(exe, os.X_OK):
            raise HwError("eseguibile non trovato: %s" % exe)
    u = cfg["udp"]
    host, mon_port = u["host"], int(u["monitor_port"])
    if mon_port <= 0:
        raise HwError("udp.monitor_port = 0: hw.py run ha bisogno della porta monitor")
    gc = cli_or_cfg_path(cfg, args.grasp_command, cfg["planner"]["grasp_command_yaml"])
    wp = cli_or_cfg_path(cfg, args.weights, cfg["planner"]["weights_yaml"])
    rr_cfg = cfg_path(cfg, cfg["robot1"]["real_replay_config_dry" if args.dry_run else "real_replay_config"])

    # 1) plan
    cmd = load_yaml(gc, "grasp command")
    ok, checks, info = plan_feasibility(cfg, cmd, wp)
    for name, good, detail, blocking in checks:
        print("  plan [%s] %s: %s" % ("PASS" if good else ("FAIL" if blocking else "WARN"), name, detail))
    print("  portata robot 1: NON verificata")
    if not ok:
        if not args.no_plan_gate:
            print("plan NON fattibile: non avvio nulla.")
            return 1
        print("plan NON fattibile ma --no-plan-gate: avvio comunque (VALIDATE di real_replay deve rifiutare prima di START)")

    # 2) robot in posa e fermi (come reset)
    tty = sys.stdin.isatty()
    if not args.dry_run:
        q1 = np.array(cfg["robot1"]["start_q_rad"], dtype=float)
        q2, _src = robot2_start_q(cfg)
        for n, qg, tol in ((1, q1, float(cfg["robot1"]["tol_deg"])), (2, q2, float(cfg["robot2"]["tol"]["q_deg"]))):
            good, msg = robot_ready(cfg, n, qg, tol)
            print("  " + msg)
            if not good:
                print("robot %d non pronto: usa 'hw.py reset --robot %d'. Non avvio nulla." % (n, n))
                return 1
        if not tty:
            print("stdin non e' un terminale: in robot mode serve la conferma dell'operatore. Non avvio nulla.")
            return 1
    # 3) conferme
    if tty:
        if not confirm("Avviare satellite_node e real_replay --mode task%s?" % (" (DRY-RUN)" if args.dry_run else " (ENTRAMBI i robot in controllo di coppia)")):
            print("annullato.")
            return 1
        if not args.dry_run and not confirm_load(cfg):
            return 2

    # 4) satellite_node + handshake WAIT_START
    sat_argv = [sat_exe, "--config", cfg["_path"]]
    if args.dry_run:
        q2, _src = robot2_start_q(cfg)
        sat_argv += ["--dry-run", "--dry-q-deg", ",".join("%.6f" % math.degrees(x) for x in q2)]
    else:
        sat_argv.append("--yes")  # solo dopo la NOSTRA conferma (e quella del carico)
    rr_argv = [rr_exe, "--config", rr_cfg, "--mode", "task", "--grasp-command", gc, "--hw-config", cfg["_path"], "--weights", wp]
    if args.dry_run:
        rr_argv.append("--dry-run")
    if tty or args.dry_run:
        rr_argv.append("--yes")  # VALIDATE: la conferma e' gia' stata data qui (o dry-run senza movimento)
    if args.fault_linger_s is not None:
        rr_argv += ["--fault-linger-s", "%.3f" % args.fault_linger_s]

    mon = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mon.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mon.bind((host, mon_port))
    print("+ " + " ".join(sat_argv))
    sat = subprocess.Popen(sat_argv, stdin=subprocess.DEVNULL, start_new_session=True)
    rr = None
    children = [sat]
    interrupted = [None]

    def on_sigint_all():
        if interrupted[0] is None:
            interrupted[0] = time.monotonic()
            print("\nCtrl-C: SIGINT a entrambi i processi, attendo le rampe di arresto (max 15 s)...")
            for c in children:
                if c.poll() is None:
                    c.send_signal(signal.SIGINT)

    try:
        p = wait_state(mon, 1, sat, 30.0 + float(cfg["robot2"]["control"]["countdown_s"]))
        if p is None:
            print("satellite_node non e' arrivato in WAIT_START: interrompo.")
            if sat.poll() is None:
                sat.send_signal(signal.SIGINT)
            rc_sat = wait_child(sat)
            print("satellite_node: exit %d = %s" % (rc_sat, NODE_EXIT.get(rc_sat, "?")))
            return 1
        print("handshake: satellite_node in WAIT_START, theta %.3f deg" % math.degrees(p["theta"]))
        # 5) real_replay --mode task (invia START, heartbeat, STOP)
        print("+ " + " ".join(rr_argv))
        rr = subprocess.Popen(rr_argv, stdin=subprocess.DEVNULL, start_new_session=True)
        children.append(rr)
        mon.setblocking(False)
        t_print = 0.0
        last = p
        t_rr_end = None
        while True:
            try:
                if rr.poll() is not None and t_rr_end is None:
                    t_rr_end = time.monotonic()
                    print("real_replay terminato (exit %d): attendo la fine del satellite (STOP gia' inviato)..." % rr.returncode)
                if t_rr_end is not None and sat.poll() is not None:
                    break
                while True:
                    r, _, _ = select.select([mon], [], [], 0.05)
                    if not r:
                        break
                    q = decode_state(mon.recv(512))
                    if q is not None:
                        last = q
                if time.monotonic() - t_print >= 1.0 and sat.poll() is None:
                    t_print = time.monotonic()
                    print("  satellite: " + state_line(last, float(u["T_age_s"])))
                if interrupted[0] is not None and time.monotonic() - interrupted[0] > 15.0:
                    for c in children:
                        if c.poll() is None:
                            print("processo %d non terminato 15 s dopo SIGINT: SIGKILL" % c.pid)
                            c.kill()
                if t_rr_end is not None and sat.poll() is None and time.monotonic() - t_rr_end > 20.0 and interrupted[0] is None:
                    print("satellite_node ancora attivo 20 s dopo real_replay: SIGINT")
                    on_sigint_all()
            except KeyboardInterrupt:
                on_sigint_all()
    finally:
        mon.close()
    rc_rr = rr.returncode if rr is not None else None
    rc_sat = sat.wait()
    print("real_replay --mode task: exit %s = %s" % (rc_rr, TASK_EXIT.get(rc_rr, "?")))
    print("satellite_node: exit %d = %s" % (rc_sat, NODE_EXIT.get(rc_sat, "?")))
    if rc_rr:
        return rc_rr
    return 0 if rc_sat == 0 else 7


def cmd_run(args, cfg):
    if not args.only_satellite:
        return cmd_run_full(args, cfg)
    exe = os.path.join(cfg_path(cfg, cfg["paths"]["build_dir"]), "satellite_node")
    if not os.access(exe, os.X_OK):
        raise HwError("satellite_node non trovato: %s (cmake --build build --target satellite_node)" % exe)
    u = cfg["udp"]
    host, cmd_port = u["host"], int(u["command_port"])
    mon_port = int(u["monitor_port"])
    if mon_port <= 0:
        raise HwError("udp.monitor_port = 0: hw.py run ha bisogno della porta monitor (la porta di pubblicazione resta al task layer)")
    argv = [exe, "--config", cfg["_path"]]
    if args.dry_run:
        q2, src = robot2_start_q(cfg)
        argv += ["--dry-run", "--dry-q-deg", ",".join("%.6f" % math.degrees(x) for x in q2)]
    if args.max_travel_deg:
        argv += ["--max-travel-deg", "%.3f" % args.max_travel_deg]
    tty = sys.stdin.isatty()
    print("satellite_node %s: robot 2 %s, omega %.2f deg/s" % ("DRY-RUN" if args.dry_run else "ROBOT", cfg["robot2"]["ip"], float(cfg["scene"]["omega_deg_s"])))
    if tty:
        if not confirm("Avviare satellite_node%s?" % ("" if args.dry_run else " (il robot 2 verra' messo in controllo di coppia)")):
            print("annullato.")
            return 1
        if not args.dry_run and not confirm_load(cfg):
            print("non avvio satellite_node.")
            return 2
        argv.append("--yes")  # solo dopo la NOSTRA conferma
        for k in range(3, 0, -1):
            print("avvio tra %d..." % k)
            time.sleep(1.0)
    elif not args.dry_run:
        print("stdin non e' un terminale: satellite_node non ricevera' --yes e rifiutera' di prendere il controllo.")

    mon = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mon.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mon.bind((host, mon_port))
    mon.setblocking(False)
    hb = Heartbeat(host, cmd_port, float(u.get("hb_send_hz", 10.0)))
    hb.start()
    print("+ " + " ".join(argv))
    child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, start_new_session=True)
    st = {"last": None, "started": False, "t_print": 0.0, "asked": False}

    def tick():
        while True:
            r, _, _ = select.select([mon], [], [], 0)
            if not r:
                break
            p = decode_state(mon.recv(512))
            if p is not None:
                st["last"] = p
        p = st["last"]
        if p is None:
            return
        if p["state"] == 1 and not st["started"] and not st["asked"]:
            st["asked"] = True
            if tty:
                print("\nsatellite_node in WAIT_START (tiene q0). Premi Invio per START (Ctrl-C per annullare)...")
                threading.Thread(target=_wait_enter_then_start, args=(hb, st), daemon=True).start()
            elif args.dry_run:
                hb.send(CMD_START)
                hb.active.set()
                st["started"] = True
        now = time.monotonic()
        if now - st["t_print"] >= 0.5:
            st["t_print"] = now
            print("  " + state_line(p, float(u["T_age_s"])))

    try:
        rc = wait_child(child, on_tick=tick)
    finally:
        hb.quit.set()
        mon.close()
    print("satellite_node: exit %d = %s" % (rc, NODE_EXIT.get(rc, "sconosciuto")))
    return rc


def mass_confirmation_ok(typed, expected_kg, tol_kg=0.001):
    """Stessa regola di real_replay::massConfirmationOk: solo un numero (kg), entro 1 g."""
    try:
        v = float(typed.strip())
    except ValueError:
        return False
    return math.isfinite(v) and abs(v - expected_kg) <= tol_kg + 1e-12


def _wait_enter_then_start(hb, st):
    try:
        sys.stdin.readline()
    except Exception:
        return
    hb.send(CMD_START)
    hb.active.set()
    st["started"] = True
    print("START inviato; heartbeat attivo.")


# ---- main ------------------------------------------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(
        prog="hw.py", formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Strumenti per i due Franka (robot 1 = task, robot 2 = satellite). Config unica: hw/hw_config.yaml.",
        epilog="Exit code: 0 ok; 1 controllo fallito o operazione rifiutata/annullata; 2 errore di config/ingresso;\n"
               "run: exit code di satellite_node (0 completato/STOP, 3 fermato con rampa, 4 rampa non completata).")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="file di configurazione (default: %(default)s)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("status", help="stato dei robot (echo_robot_state) e del satellite_node")
    p.add_argument("--sat", action="store_true", help="ascolta 2 s i pacchetti UDP del satellite_node (porta monitor)")
    p.add_argument("--no-robot", action="store_true", help="non interrogare i robot (solo --sat)")
    p = sub.add_parser("ik", help="IK offline del robot 2 da centro, inclinazione asse e psi0")
    p.add_argument("--psi0-deg", type=float, default=None, help="fase di partenza (default: robot2.start.psi0_deg)")
    p = sub.add_parser("reset", help="porta i robot nella posa di partenza (uno alla volta, con conferma)")
    p.add_argument("--robot", choices=["1", "2", "both"], default="both")
    p.add_argument("--yes", action="store_true", help="non chiedere conferma")
    p = sub.add_parser("plan", help="fattibilita' del GraspCommand (nessun movimento)")
    p.add_argument("--grasp-command", help="YAML del comando (default: planner.grasp_command_yaml)")
    p.add_argument("--weights", help="YAML dei pesi (default: planner.weights_yaml)")
    p.add_argument("--theta0-deg", type=float, default=None, help="fase di partenza da usare invece di quella da config/IK")
    p = sub.add_parser("run", help="sequenza completa (plan, posa, satellite_node, real_replay --mode task) o --only-satellite")
    p.add_argument("--only-satellite", action="store_true", help="stadio 1: avvia solo satellite_node (START/heartbeat da hw.py)")
    p.add_argument("--dry-run", action="store_true", help="entrambi i processi in dry-run (nessun robot)")
    p.add_argument("--grasp-command", help="YAML del comando (default: planner.grasp_command_yaml)")
    p.add_argument("--weights", help="YAML dei pesi ProDMP veri (default: planner.weights_yaml)")
    p.add_argument("--fault-linger-s", type=float, default=None,
                   help="real_replay: secondi nello stato di guasto prima di uscire (default config: 0 = fino a Ctrl-C)")
    p.add_argument("--no-plan-gate", action="store_true",
                   help="SOLO TEST: avvia anche se plan fallisce (VALIDATE di real_replay deve rifiutare prima di START)")
    p.add_argument("--max-travel-deg", type=float, default=0.0, help="limita la corsa di J7 (oltre alla finestra)")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        cfg = load_config(args.config)
        return {"status": cmd_status, "ik": cmd_ik, "reset": cmd_reset, "plan": cmd_plan, "run": cmd_run}[args.cmd](args, cfg)
    except HwError as e:
        print("errore: %s" % e, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
