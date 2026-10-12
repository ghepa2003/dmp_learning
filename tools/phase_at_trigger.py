#!/usr/bin/env python3
"""
phase_at_trigger.py - fase assoluta del satellite al trigger di grasp, da un bag rosbag2 (sqlite3), SENZA ROS.

Input:
  --bag <cartella>      bag della demo con /touch0/buttons_synced e /free_target_object/odometry
  --demo-csv <file>     demo_raw.csv (t,x,y,z,qw,qx,qy,qz,gripper_trigger), per il controllo di tau
  --q-ref x y z w       quaternione di riferimento della fase (default 0 0 0 1)
  --axis x y z          asse di rotazione (default 0 0 1)

Procedura:
  1. /touch0/buttons_synced (sensor_msgs/Joy): istante sim (header.stamp) del primo fronte su buttons[0]
     (0 -> != 0, il primo messaggio fa solo da seme, come nel recorder) = t_start, e del primo messaggio
     con buttons[2] != 0 = t_trigger. Se header.stamp e' zero: errore (nessun tempo sim disponibile).
  2. /free_target_object/odometry (nav_msgs/Odometry): orientamento interpolato (slerp) a t_trigger.
  3. theta come PhaseTracker (satellite_intercept.hpp): R_rel = R(q) * R(q_ref)^T, vettore di rotazione
     (|v| in [0, pi]), proiezione sull'asse, riportata in (-pi, pi]. R(q)R(q_ref)^T = R(q * conj(q_ref)).
  4. Controllo: (t_trigger - t_start) contro tau del CSV (t della riga col primo gripper_trigger != 0
     meno t della prima riga); scarto >= 20 ms => AVVISO.

Stampa anche la riga da incollare "--phase-at-contact-deg <v>" e lo yaw assoluto non avvolto
(unwrap continuo dello yaw sui campioni di odometria dal primo fino a t_trigger).
"""
from __future__ import annotations

import argparse
import csv
import math
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bag_time_probe import db_files  # noqa: E402  (stessa lettura del bag di bag_time_probe)

BUTTONS_TOPIC = "/touch0/buttons_synced"
ODOM_TOPIC = "/free_target_object/odometry"
TAU_TOL_S = 0.020


class PhaseError(Exception):
    pass


# --------------------------------------------------------------------------------------------
# CDR (little/big endian). Allineamento relativo all'inizio del payload (dopo i 4 byte di intestazione).
# --------------------------------------------------------------------------------------------
class _Cdr:
    def __init__(self, blob: bytes):
        if len(blob) < 4 or blob[0] != 0 or blob[1] not in (0, 1):
            raise PhaseError(f"intestazione CDR anomala: {bytes(blob[:4]).hex()}")
        self.b = bytes(blob)
        self.e = "<" if blob[1] == 1 else ">"
        self.o = 4

    def _align(self, n):
        self.o = 4 + ((self.o - 4 + n - 1) // n) * n

    def take(self, fmt, n_align, count=1):
        self._align(n_align)
        size = struct.calcsize(self.e + fmt) * count
        if self.o + size > len(self.b):
            raise PhaseError("messaggio CDR troncato")
        vals = struct.unpack_from(self.e + fmt * count, self.b, self.o)
        self.o += size
        return vals

    def header(self):
        sec, nsec = self.take("iI", 4)
        self.string()
        return sec, nsec

    def string(self):
        (n,) = self.take("I", 4)
        if self.o + n > len(self.b):
            raise PhaseError("stringa CDR troncata")
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b"\0").decode(errors="replace")


def decode_joy(blob):
    """(stamp_ns, buttons[list[int]]) da sensor_msgs/Joy."""
    c = _Cdr(blob)
    sec, nsec = c.header()
    (na,) = c.take("I", 4)
    c.take("f", 4, na)
    (nb,) = c.take("I", 4)
    buttons = list(c.take("i", 4, nb)) if nb else []
    return sec * 10**9 + nsec, buttons


def decode_odom(blob):
    """(stamp_ns, quaternion (x, y, z, w)) da nav_msgs/Odometry (pose.pose.orientation)."""
    c = _Cdr(blob)
    sec, nsec = c.header()
    c.string()  # child_frame_id
    c.take("ddd", 8)  # position
    qx, qy, qz, qw = c.take("dddd", 8)
    return sec * 10**9 + nsec, (qx, qy, qz, qw)


def read_messages(bag_dir, topic, decoder):
    """Messaggi decodificati del topic, ordinati per (timestamp di ricezione, id)."""
    rows = []
    for db in db_files(bag_dir):
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
            if tid is None:
                continue
            rows += con.execute("SELECT timestamp, id, data FROM messages WHERE topic_id=?", (tid[0],)).fetchall()
        finally:
            con.close()
    if not rows:
        raise PhaseError(f"topic {topic} assente o vuoto nel bag {bag_dir}")
    rows.sort(key=lambda r: (r[0], r[1]))
    return [decoder(bytes(r[2])) for r in rows]


# --------------------------------------------------------------------------------------------
# Matematica
# --------------------------------------------------------------------------------------------
def wrap_pi(a):
    """Riporta a in (-pi, pi]."""
    r = math.fmod(a + math.pi, 2.0 * math.pi)
    if r <= 0.0:
        r += 2.0 * math.pi
    return r - math.pi


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qnorm(q):
    n = math.sqrt(sum(v * v for v in q))
    if not n > 0:
        raise PhaseError("quaternione di norma nulla")
    return tuple(v / n for v in q)


def slerp(q0, q1, u):
    q0, q1 = _qnorm(q0), _qnorm(q1)
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0:
        q1, d = tuple(-v for v in q1), -d
    if d > 0.9995:
        return _qnorm(tuple(a + u * (b - a) for a, b in zip(q0, q1)))
    th = math.acos(d)
    s = math.sin(th)
    w0, w1 = math.sin((1 - u) * th) / s, math.sin(u * th) / s
    return _qnorm(tuple(w0 * a + w1 * b for a, b in zip(q0, q1)))


def theta_from_quat(q, q_ref=(0.0, 0.0, 0.0, 1.0), axis=(0.0, 0.0, 1.0)):
    """(theta [rad] in (-pi, pi], componente fuori asse [deg]) come PhaseTracker::update (primo campione)."""
    a = np.asarray(axis, float)
    if not np.linalg.norm(a) > 1e-9:
        raise PhaseError("asse di norma <= 1e-9")
    a = a / np.linalg.norm(a)
    qr = _qnorm(q_ref)
    q_rel = _qmul(_qnorm(q), (-qr[0], -qr[1], -qr[2], qr[3]))  # R(q) R(q_ref)^T
    if q_rel[3] < 0:
        q_rel = tuple(-v for v in q_rel)  # angolo in [0, pi]
    s = math.sqrt(q_rel[0] ** 2 + q_rel[1] ** 2 + q_rel[2] ** 2)
    angle = 2.0 * math.atan2(s, q_rel[3])
    v = np.zeros(3) if s < 1e-15 else angle * np.array(q_rel[:3]) / s
    raw = float(v.dot(a))
    off_deg = math.degrees(float(np.linalg.norm(v - raw * a)))
    return wrap_pi(raw), off_deg


def yaw_from_quat(q):
    x, y, z, w = _qnorm(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# --------------------------------------------------------------------------------------------
# Eventi, tau, fase
# --------------------------------------------------------------------------------------------
def button_events(joy_msgs):
    """(t_start_ns, t_trigger_ns): primo fronte su buttons[0]; primo buttons[2] != 0. Errore se stamp zero."""
    if any(t == 0 for t, _ in joy_msgs):
        raise PhaseError(f"{BUTTONS_TOPIC}: header.stamp nullo (zero) in almeno un messaggio: tempo sim non disponibile")
    t_start = t_trigger = None
    prev0 = None
    for t, b in joy_msgs:
        b0 = b[0] if len(b) >= 1 else 0
        if t_start is None and prev0 is not None and b0 != 0 and prev0 == 0:
            t_start = t
        prev0 = b0
        if t_trigger is None and len(b) >= 3 and b[2] != 0:
            t_trigger = t
    if t_start is None:
        raise PhaseError(f"{BUTTONS_TOPIC}: nessun fronte su buttons[0]")
    if t_trigger is None:
        raise PhaseError(f"{BUTTONS_TOPIC}: nessun messaggio con buttons[2] != 0")
    return t_start, t_trigger


def csv_tau(demo_csv):
    """tau [s] = t della riga col primo gripper_trigger != 0 meno t della prima riga."""
    with open(demo_csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or "t" not in rows[0] or "gripper_trigger" not in rows[0]:
        raise PhaseError(f"{demo_csv}: header deve contenere 't' e 'gripper_trigger'")
    t0 = float(rows[0]["t"])
    for r in rows:
        if r["gripper_trigger"].strip() != "" and float(r["gripper_trigger"]) != 0.0:
            return float(r["t"]) - t0
    raise PhaseError(f"{demo_csv}: nessuna riga con gripper_trigger != 0")


def orientation_at(odom, t_ns):
    """(quaternione slerp a t_ns, yaw assoluto non avvolto [rad] a t_ns). odom = [(stamp_ns, q)] ordinato."""
    if any(t == 0 for t, _ in odom):
        raise PhaseError(f"{ODOM_TOPIC}: header.stamp nullo in almeno un messaggio")
    ts = np.array([t for t, _ in odom], dtype=np.int64)
    if np.any(np.diff(ts) < 0):
        raise PhaseError(f"{ODOM_TOPIC}: header.stamp non monotono")
    if not (ts[0] <= t_ns <= ts[-1]):
        raise PhaseError(f"t_trigger {t_ns * 1e-9:.6f} s fuori dall'odometria [{ts[0] * 1e-9:.6f}, {ts[-1] * 1e-9:.6f}] s")
    i = int(np.searchsorted(ts, t_ns, side="right")) - 1
    i = min(i, len(ts) - 2) if len(ts) > 1 else 0
    if len(ts) == 1 or ts[i + 1] == ts[i]:
        u = 0.0
    else:
        u = (t_ns - ts[i]) / (ts[i + 1] - ts[i])
    q = slerp(odom[i][1], odom[min(i + 1, len(ts) - 1)][1], u)
    yaws = np.unwrap([yaw_from_quat(qq) for _, qq in odom[:min(i + 2, len(ts))]])
    if len(yaws) == 1:
        yaw = float(yaws[0])
    else:
        yaw = float(yaws[i] + u * (yaws[i + 1] - yaws[i]))
    return q, yaw


def phase_at_trigger(joy_msgs, odom, q_ref=(0.0, 0.0, 0.0, 1.0), axis=(0.0, 0.0, 1.0)):
    t_start, t_trigger = button_events(joy_msgs)
    q, yaw_unwrapped = orientation_at(odom, t_trigger)
    theta, off_deg = theta_from_quat(q, q_ref, axis)
    return {"t_start_s": t_start * 1e-9, "t_trigger_s": t_trigger * 1e-9,
            "dt_s": (t_trigger - t_start) * 1e-9, "q": q,
            "yaw_unwrapped_deg": math.degrees(yaw_unwrapped),
            "theta_deg": math.degrees(theta), "off_axis_deg": off_deg}


def tau_check(dt_s, tau_s):
    """(scarto [s], avviso o None)."""
    diff = dt_s - tau_s
    if abs(diff) >= TAU_TOL_S:
        return diff, (f"AVVISO: (t_trigger - t_start) = {dt_s:.4f} s differisce dal tau del CSV = {tau_s:.4f} s "
                      f"di {diff * 1e3:+.1f} ms (soglia {TAU_TOL_S * 1e3:.0f} ms): bag e CSV potrebbero non corrispondere")
    return diff, None


def main(argv=None):
    ap = argparse.ArgumentParser(description="Fase assoluta del satellite al trigger (da bag rosbag2 sqlite3, senza ROS)")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--demo-csv", required=True)
    ap.add_argument("--q-ref", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0], metavar=("X", "Y", "Z", "W"))
    ap.add_argument("--axis", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("X", "Y", "Z"))
    args = ap.parse_args(argv)
    try:
        joy = read_messages(args.bag, BUTTONS_TOPIC, decode_joy)
        odom = read_messages(args.bag, ODOM_TOPIC, decode_odom)
        r = phase_at_trigger(joy, odom, tuple(args.q_ref), tuple(args.axis))
        tau = csv_tau(args.demo_csv)
    except (PhaseError, OSError, ValueError) as e:
        print(f"ERRORE: {e}", file=sys.stderr)
        return 1
    diff, warn = tau_check(r["dt_s"], tau)
    print(f"t_start    (sim) = {r['t_start_s']:.6f} s")
    print(f"t_trigger  (sim) = {r['t_trigger_s']:.6f} s")
    print(f"t_trigger - t_start = {r['dt_s']:.6f} s   tau CSV = {tau:.6f} s   scarto = {diff * 1e3:+.2f} ms")
    if warn:
        print(warn)
    print(f"quaternione a t_trigger (x y z w) = {' '.join(f'{v:.9f}' for v in r['q'])}")
    print(f"yaw assoluto non avvolto = {r['yaw_unwrapped_deg']:.4f} deg")
    print(f"theta (q_ref={list(args.q_ref)}, asse={list(args.axis)}) = {r['theta_deg']:.4f} deg   "
          f"[componente fuori asse {r['off_axis_deg']:.4f} deg]")
    print(f"--phase-at-contact-deg {r['theta_deg']:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
