#!/usr/bin/env python3
"""Valuta il filtro a MEDIA MOBILE A DUE STADI per la stima di velocita'/
accelerazione (posizione) ed eta/eta_dot (orientamento), confrontandolo con
lo stimatore grezzo attuale (differenze centrali dirette sui dati) contro
una verita' nota in forma chiusa.

Perche' "a due stadi": la pipeline attuale deriva due volte in cascata
(posizione -> velocita' -> accelerazione; log-map -> eta -> eta_dot). Uno
smoothing applicato una sola volta sul segnale grezzo migliora bene il primo
stadio ma lascia rumore residuo (introdotto dalla prima derivazione stessa)
nel secondo. Questo script applica la stessa media mobile PRIMA di ciascuno
dei due stadi, senza toccare la formula di differenza centrale gia' presente
nel core -- solo un pre-filtraggio in due punti.

Nessuna dipendenza da scipy (solo numpy), coerente con l'ambiente del
progetto.

Uso:
    python3 evaluate_moving_average_filter.py \
        --demo demo_noisy.csv --truth demo_truth.csv \
        --window-sec-1 0.1 0.2 0.5 --window-sec-2 0.1 0.2 0.5 \
        --output-csv ma_eval_results.csv
"""
import argparse
import csv
import math

import numpy as np


# --------------------------------------------------------------------------
# Caricamento CSV
# --------------------------------------------------------------------------

def load_demo(path):
    t, pos, quat = [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            pos.append([float(row["x"]), float(row["y"]), float(row["z"])])
            quat.append([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])
    return np.array(t), np.array(pos), np.array(quat)


def load_truth(path):
    t, vel, acc, eta, eta_dot = [], [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            vel.append([float(row["vx"]), float(row["vy"]), float(row["vz"])])
            acc.append([float(row["ax"]), float(row["ay"]), float(row["az"])])
            eta.append([float(row["etax"]), float(row["etay"]), float(row["etaz"])])
            eta_dot.append([float(row["eta_dot_x"]), float(row["eta_dot_y"]), float(row["eta_dot_z"])])
    return np.array(t), np.array(vel), np.array(acc), np.array(eta), np.array(eta_dot)


# --------------------------------------------------------------------------
# Quaternioni (w,x,y,z), stessa convenzione di Eigen::Quaterniond
# --------------------------------------------------------------------------

def quat_normalize(q):
    return q / np.linalg.norm(q)


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def log_map(q):
    """Replica esatta di QuaternionDMP::logMap."""
    v = q[1:4]
    vnorm = np.linalg.norm(v)
    if vnorm < 1e-8:
        return np.zeros(3)
    w = max(-1.0, min(1.0, q[0]))
    angle = math.acos(w)
    return angle * v / vnorm


def central_diff(t, signal):
    """Replica ESATTA delle differenze centrali gia' presenti in
    dmp.cpp/quaternion_dmp.cpp, incluso il clamping degli indici ai bordi."""
    N = len(t)
    out = np.zeros_like(signal)
    for k in range(N):
        km1 = max(0, k - 1)
        kp1 = min(N - 1, k + 1)
        dt = t[kp1] - t[km1]
        if dt <= 0.0:
            dt = 1e-6
        out[k] = (signal[kp1] - signal[km1]) / dt
    return out


# --------------------------------------------------------------------------
# Stimatore GREZZO (baseline attuale, nessun filtro)
# --------------------------------------------------------------------------

def raw_position_derivatives(t, pos):
    vel = central_diff(t, pos)
    acc = central_diff(t, vel)
    return vel, acc


def raw_orientation_derivatives(t, quat, tau):
    """Replica ESATTA di QuaternionDMP::learnFromDemonstration: omega stimata
    da rotazione INCREMENTALE tra campioni k-1/k+1 (non rispetto a un
    riferimento fisso) -- valida per qualunque percorso, non solo rotazioni
    ad asse fisso."""
    N = len(t)
    eta = np.zeros((N, 3))
    for k in range(N):
        km1 = max(0, k - 1)
        kp1 = min(N - 1, k + 1)
        dt = t[kp1] - t[km1]
        if dt <= 0.0:
            dt = 1e-6
        qk1 = quat_normalize(quat[kp1])
        qk0 = quat_normalize(quat[km1])
        dq = quat_multiply(qk1, quat_conjugate(qk0))
        omega = 2.0 * log_map(dq) / dt
        eta[k] = tau * omega
    eta_dot = central_diff(t, eta)
    return eta, eta_dot


def unwrap_rotation_vector(t, quat):
    """Costruisce una traiettoria CONTINUA in R^3 (coordinate tangenti)
    integrando cumulativamente gli incrementi di rotazione LOCALI tra
    campioni consecutivi -- a differenza di logMap(q(t)*q0^-1), questo resta
    valido anche se l'asse di rotazione cambia nel tempo (percorso non
    vincolato a un'unica geodetica), perche' ogni incremento e' piccolo per
    costruzione (campioni adiacenti nel tempo)."""
    N = len(t)
    r = np.zeros((N, 3))
    for k in range(1, N):
        dt = t[k] - t[k - 1]
        if dt <= 0.0:
            dt = 1e-6
        qk = quat_normalize(quat[k])
        qkm1 = quat_normalize(quat[k - 1])
        dq = quat_multiply(qk, quat_conjugate(qkm1))
        incr = 2.0 * log_map(dq)  # incremento di rotazione (rad) tra k-1 e k
        r[k] = r[k - 1] + incr
    return r


# --------------------------------------------------------------------------
# Filtro: media mobile centrata, finestra ridotta simmetricamente vicino ai
# bordi veri (nessun padding, nessun dato inventato oltre gli estremi).
# --------------------------------------------------------------------------

def moving_average_smooth(signal, window_samples):
    N = len(signal)
    half = window_samples // 2
    out = np.zeros_like(signal)
    for k in range(N):
        lo = max(0, k - half)
        hi = min(N, k + half + 1)
        out[k] = signal[lo:hi].mean(axis=0)
    return out


def filtered_position_derivatives(t, pos, dt, window_sec_1, window_sec_2):
    """Media mobile PRIMA di ciascuno dei due stadi di derivazione, non solo
    sul segnale grezzo in ingresso."""
    w1 = max(1, int(round(window_sec_1 / dt)))
    w2 = max(1, int(round(window_sec_2 / dt)))
    pos_s = moving_average_smooth(pos, w1)
    vel = central_diff(t, pos_s)
    vel_s = moving_average_smooth(vel, w2)
    acc = central_diff(t, vel_s)
    return vel, acc


def filtered_orientation_derivatives(t, quat, tau, dt, window_sec_1, window_sec_2):
    """Come filtered_position_derivatives, ma sulla traiettoria unwrapped
    (coordinate tangenti costruite per accumulo di incrementi locali, valida
    per percorsi di rotazione generali)."""
    w1 = max(1, int(round(window_sec_1 / dt)))
    w2 = max(1, int(round(window_sec_2 / dt)))
    r = unwrap_rotation_vector(t, quat)
    r_s = moving_average_smooth(r, w1)
    eta = tau * central_diff(t, r_s)
    eta_s = moving_average_smooth(eta, w2)
    eta_dot = central_diff(t, eta_s)
    return eta, eta_dot


# --------------------------------------------------------------------------
# Metriche
# --------------------------------------------------------------------------

def rmse(est, truth):
    return math.sqrt(np.mean(np.sum((est - truth) ** 2, axis=1)))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--demo", required=True, help="CSV rumoroso (t,x,y,z,qw,qx,qy,qz)")
    p.add_argument("--truth", required=True, help="CSV verita' nota (da generate_truth)")
    p.add_argument("--window-sec-1", type=float, nargs="+", default=[0.1, 0.2, 0.5],
                    help="finestre (s) per il PRIMO stadio (posizione->velocita', logmap->eta)")
    p.add_argument("--window-sec-2", type=float, nargs="+", default=[0.1, 0.2, 0.5],
                    help="finestre (s) per il SECONDO stadio (velocita'->accelerazione, eta->eta_dot)")
    p.add_argument("--output-csv", default="ma_eval_results.csv")
    args = p.parse_args()

    t, pos, quat = load_demo(args.demo)
    t_truth, vel_true, acc_true, eta_true, eta_dot_true = load_truth(args.truth)
    assert np.allclose(t, t_truth), "griglia temporale demo/truth non allineata"
    tau = t[-1] - t[0]
    dt = t[1] - t[0]

    print(f"Caricati {len(t)} campioni, tau={tau:.3f}s, dt={dt:.4f}s\n")

    rows = []

    vel_raw, acc_raw = raw_position_derivatives(t, pos)
    eta_raw, eta_dot_raw = raw_orientation_derivatives(t, quat, tau)
    rmse_vel_raw, rmse_acc_raw = rmse(vel_raw, vel_true), rmse(acc_raw, acc_true)
    rmse_eta_raw, rmse_eta_dot_raw = rmse(eta_raw, eta_true), rmse(eta_dot_raw, eta_dot_true)
    rows.append(["raw", "-", "-", rmse_vel_raw, rmse_acc_raw, rmse_eta_raw, rmse_eta_dot_raw])
    print(f"[grezzo]                     vel={rmse_vel_raw:.6f}  acc={rmse_acc_raw:.6f}  "
          f"eta={rmse_eta_raw:.6f}  eta_dot={rmse_eta_dot_raw:.6f}")

    for w1 in args.window_sec_1:
        for w2 in args.window_sec_2:
            vel_f, acc_f = filtered_position_derivatives(t, pos, dt, w1, w2)
            eta_f, eta_dot_f = filtered_orientation_derivatives(t, quat, tau, dt, w1, w2)
            rv, ra = rmse(vel_f, vel_true), rmse(acc_f, acc_true)
            re, red = rmse(eta_f, eta_true), rmse(eta_dot_f, eta_dot_true)
            rows.append(["filtered", w1, w2, rv, ra, re, red])
            print(f"[media w1={w1:>4}s w2={w2:>4}s]     vel={rv:.6f}  acc={ra:.6f}  "
                  f"eta={re:.6f}  eta_dot={red:.6f}")

    with open(args.output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["method", "window_sec_1", "window_sec_2",
                          "rmse_vel", "rmse_acc", "rmse_eta", "rmse_eta_dot"])
        writer.writerows(rows)
    print(f"\nRisultati scritti in {args.output_csv}")


if __name__ == "__main__":
    main()
