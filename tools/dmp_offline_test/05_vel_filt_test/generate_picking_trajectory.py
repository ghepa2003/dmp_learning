#!/usr/bin/env python3
"""Genera una traiettoria sintetica di tipo "picking" (traslazione
orizzontale seguita da traslazione verticale, con transizione smussata)
nello stesso formato CSV usato dalle demo reali (t,x,y,z,qw,qx,qy,qz),
cosi' da poter essere usata direttamente da learnFromDemonstration /
replay_saved_dmp senza modifiche alla pipeline esistente.

Metodologia di smussatura: profilo minimum-jerk (stessa famiglia
matematica di min_jerk_step.m, gia' usata come riferimento nel resto del
progetto) applicato indipendentemente a ciascun asse su una finestra
temporale propria. Le finestre orizzontale e verticale si sovrappongono
parzialmente (di default) per ottenere una transizione dolce invece di
uno spigolo netto tra le due fasi del movimento.

Uso:
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.15 --dy 0.05 --dz -0.10 --output demo_synth_45s.csv

    # per goal diversi (stesso spostamento totale, direzioni diverse):
    python3 generate_picking_trajectory.py --duration 45 \
        --dx 0.20 --dy -0.05 --dz -0.12 --output demo_synth_45s_goalB.csv
"""
import argparse
import csv
import math
import random


def min_jerk_profile(t, t0, t1, start, end):
    smoothed = min_jerk_blend(t, t0, t1)
    return start + (end - start) * smoothed


def min_jerk_velocity(t, t0, t1, start, end):
    """Derivata prima analitica ESATTA di min_jerk_profile rispetto al tempo."""
    if t <= t0 or t >= t1:
        return 0.0
    s = (t - t0) / (t1 - t0)
    ds_dt = 1.0 / (t1 - t0)
    dsmoothed_ds = 30 * s**2 - 60 * s**3 + 30 * s**4
    return (end - start) * dsmoothed_ds * ds_dt


def min_jerk_acceleration(t, t0, t1, start, end):
    """Derivata seconda analitica ESATTA di min_jerk_profile rispetto al tempo."""
    if t <= t0 or t >= t1:
        return 0.0
    s = (t - t0) / (t1 - t0)
    ds_dt = 1.0 / (t1 - t0)
    d2smoothed_ds2 = 60 * s - 180 * s**2 + 120 * s**3
    return (end - start) * d2smoothed_ds2 * ds_dt**2


def min_jerk_blend(t, t0, t1):
    """Restituisce solo il fattore di blend smussato (0 prima di t0, 1 dopo
    t1), da usare per interpolazioni diverse dalla somma pesata lineare
    (es. slerp tra quaternioni)."""
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    s = (t - t0) / (t1 - t0)
    return 10 * s**3 - 15 * s**4 + 6 * s**5

def min_jerk_blend_derivative(t, t0, t1, order=1):
    """Derivata (1a o 2a) del solo fattore di blend (0->1)."""
    if t <= t0 or t >= t1:
        return 0.0
    s = (t - t0) / (t1 - t0)
    ds_dt = 1.0 / (t1 - t0)
    if order == 1:
        dsmoothed_ds = 30 * s**2 - 60 * s**3 + 30 * s**4
        return dsmoothed_ds * ds_dt
    elif order == 2:
        d2smoothed_ds2 = 60 * s - 180 * s**2 + 120 * s**3
        return d2smoothed_ds2 * ds_dt**2
    raise ValueError("order deve essere 1 o 2")

def quat_from_axis_angle(axis, angle_deg):
    """Costruisce un quaternione (w,x,y,z) da asse (non necessariamente
    normalizzato) e angolo in gradi."""
    angle_rad = math.radians(angle_deg)
    n = math.sqrt(sum(c * c for c in axis))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    ax = tuple(c / n for c in axis)
    s = math.sin(angle_rad / 2.0)
    return (math.cos(angle_rad / 2.0), ax[0] * s, ax[1] * s, ax[2] * s)


def slerp(q0, q1, s):
    """Interpolazione sferica tra due quaternioni unitari (w,x,y,z), con
    correzione di continuita' di emisfero e fallback lineare per angoli
    quasi nulli (evita divisioni per sin(theta)~0)."""
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-c for c in q1)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))

    if dot > 0.9995:
        result = tuple(a + s * (b - a) for a, b in zip(q0, q1))
    else:
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        theta = theta0 * s
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
        s1 = math.sin(theta) / sin_theta0
        result = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))

    norm = math.sqrt(sum(c * c for c in result))
    return tuple(c / norm for c in result)


def quat_multiply(q1, q2):
    """Prodotto tra due quaternioni (w,x,y,z): q1 * q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def generate(duration, dx, dy, dz, dt=0.001, transition_duration=10.0,
             horizontal_window=(0.0, 0.6), vertical_window=(0.4, 1.0),
             y0=(0.0, 0.0, 0.0), q0=(1.0, 0.0, 0.0, 0.0),
             rot_axis=(0.0, 0.0, 1.0), rot_angle_deg=0.0,
             pos_noise_std=0.0, orient_noise_std_deg=0.0, noise_seed=None):
    """Genera la traiettoria completa.

    transition_duration: durata ASSOLUTA in secondi (non frazione di
    `duration`) del solo movimento (orizzontale+verticale). Il resto della
    durata totale richiesta viene riempito automaticamente da una tenuta
    ferma alla posizione di goal (min_jerk_profile restituisce gia' `end`
    costante per t oltre la finestra, quindi non serve altro codice).

    rot_axis, rot_angle_deg: asse (in questo frame) e ampiezza della
    rotazione del pennino durante il movimento (es. rotazione del polso
    durante il pick). L'orientamento passa da q0 al quaternione ruotato
    tramite slerp, usando lo stesso profilo minimum-jerk (in blend, non in
    posizione) sull'intera finestra di transizione [0, transition_duration].

    pos_noise_std: deviazione standard (in metri) di un rumore gaussiano
    indipendente aggiunto a ciascuna componente x,y,z di ciascun campione --
    pensato per imitare il rumore di quantizzazione/jitter di un
    dispositivo aptico reale (0.0 = nessun rumore, traiettoria pulita).
    orient_noise_std_deg: deviazione standard (in gradi) di una piccola
    rotazione casuale composta con l'orientamento pulito ad ogni campione
    (stesso spirito, applicato all'orientamento invece che alla posizione).
    noise_seed: seed per riproducibilita' (None = non deterministico).

    horizontal_window / vertical_window: frazioni di `transition_duration`
    (non piu' di `duration`) che definiscono inizio/fine di ciascuna fase.
    """
    if transition_duration > duration:
        raise ValueError(
            f"transition_duration ({transition_duration}s) non puo' superare "
            f"duration ({duration}s): non ci sarebbe tempo per completare il moto."
        )

    hx0, hx1 = horizontal_window[0] * transition_duration, horizontal_window[1] * transition_duration
    vz0, vz1 = vertical_window[0] * transition_duration, vertical_window[1] * transition_duration

    x0, y0_, z0 = y0
    q_target = quat_from_axis_angle(rot_axis, rot_angle_deg)

    rng = random.Random(noise_seed)

    n_samples = int(round(duration / dt)) + 1
    rows = []
    for k in range(n_samples):
        t = k * dt
        x = min_jerk_profile(t, hx0, hx1, x0, x0 + dx)
        y = min_jerk_profile(t, hx0, hx1, y0_, y0_ + dy)
        z = min_jerk_profile(t, vz0, vz1, z0, z0 + dz)

        if pos_noise_std > 0.0:
            x += rng.gauss(0.0, pos_noise_std)
            y += rng.gauss(0.0, pos_noise_std)
            z += rng.gauss(0.0, pos_noise_std)

        if rot_angle_deg != 0.0:
            blend = min_jerk_blend(t, 0.0, transition_duration)
            q_clean = slerp(q0, q_target, blend)
        else:
            q_clean = q0

        if orient_noise_std_deg > 0.0:
            # piccola rotazione casuale (asse e angolo indipendenti),
            # composta con l'orientamento pulito
            noise_axis = (rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0))
            noise_angle = rng.gauss(0.0, orient_noise_std_deg)
            dq = quat_from_axis_angle(noise_axis, noise_angle)
            qw, qx, qy, qz = quat_multiply(dq, q_clean)
        else:
            qw, qx, qy, qz = q_clean

        rows.append((t, x, y, z, qw, qx, qy, qz))
    return rows

def generate_truth(duration, dx, dy, dz, dt=0.001, transition_duration=10.0,
                    horizontal_window=(0.0, 0.6), vertical_window=(0.4, 1.0),
                    rot_axis=(0.0, 0.0, 1.0), rot_angle_deg=0.0):
    """Velocita'/accelerazione di posizione ed eta/eta_dot di orientamento
    VERE (forma chiusa, senza rumore, senza differenze finite), sulla stessa
    griglia temporale di generate(). eta/eta_dot scalati per tau=duration,
    stessa convenzione di QuaternionDMP (eta = tau*omega)."""
    hx0, hx1 = horizontal_window[0] * transition_duration, horizontal_window[1] * transition_duration
    vz0, vz1 = vertical_window[0] * transition_duration, vertical_window[1] * transition_duration

    tau = duration
    angle_rad = math.radians(rot_angle_deg)
    n = math.sqrt(sum(c * c for c in rot_axis))
    axis_unit = tuple(c / n for c in rot_axis) if n > 1e-12 else (0.0, 0.0, 0.0)

    n_samples = int(round(duration / dt)) + 1
    rows = []
    for k in range(n_samples):
        t = k * dt
        vx = min_jerk_velocity(t, hx0, hx1, 0.0, dx)
        vy = min_jerk_velocity(t, hx0, hx1, 0.0, dy)
        vz = min_jerk_velocity(t, vz0, vz1, 0.0, dz)
        ax = min_jerk_acceleration(t, hx0, hx1, 0.0, dx)
        ay = min_jerk_acceleration(t, hx0, hx1, 0.0, dy)
        az = min_jerk_acceleration(t, vz0, vz1, 0.0, dz)

        if rot_angle_deg != 0.0:
            blend_d1 = min_jerk_blend_derivative(t, 0.0, transition_duration, order=1)
            blend_d2 = min_jerk_blend_derivative(t, 0.0, transition_duration, order=2)
        else:
            blend_d1 = blend_d2 = 0.0

        etax = tau * axis_unit[0] * angle_rad * blend_d1
        etay = tau * axis_unit[1] * angle_rad * blend_d1
        etaz = tau * axis_unit[2] * angle_rad * blend_d1
        edx = tau * axis_unit[0] * angle_rad * blend_d2
        edy = tau * axis_unit[1] * angle_rad * blend_d2
        edz = tau * axis_unit[2] * angle_rad * blend_d2

        rows.append((t, vx, vy, vz, ax, ay, az, etax, etay, etaz, edx, edy, edz))
    return rows


def write_truth_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "vx", "vy", "vz", "ax", "ay", "az",
                          "etax", "etay", "etaz", "eta_dot_x", "eta_dot_y", "eta_dot_z"])
        for row in rows:
            writer.writerow([f"{v:.10f}" for v in row])

def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for row in rows:
            writer.writerow([f"{v:.6f}" for v in row])


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, required=True,
                    help="durata totale della traiettoria in secondi (movimento + tenuta)")
    p.add_argument("--transition-duration", type=float, default=10.0,
                    help="durata ASSOLUTA in secondi del solo movimento (default 10s); "
                         "il resto di --duration e' tenuta ferma al goal")
    p.add_argument("--dx", type=float, default=0.15, help="spostamento orizzontale asse x [m]")
    p.add_argument("--dy", type=float, default=0.0, help="spostamento orizzontale asse y [m]")
    p.add_argument("--dz", type=float, default=-0.10, help="spostamento verticale asse z [m]")
    p.add_argument("--dt", type=float, default=0.001, help="passo di campionamento [s] (default 1kHz, come il Geomagic Touch)")
    p.add_argument("--h-start-frac", type=float, default=0.0, help="inizio finestra orizzontale, frazione di duration")
    p.add_argument("--h-end-frac", type=float, default=0.6, help="fine finestra orizzontale, frazione di duration")
    p.add_argument("--v-start-frac", type=float, default=0.4, help="inizio finestra verticale, frazione di transition_duration")
    p.add_argument("--v-end-frac", type=float, default=1.0, help="fine finestra verticale, frazione di transition_duration")
    p.add_argument("--rot-axis", type=str, default="0,0,1",
                    help="asse di rotazione del pennino, 'x,y,z' (default 0,0,1 = imbardata/yaw)")
    p.add_argument("--rot-angle-deg", type=float, default=0.0,
                    help="ampiezza della rotazione durante il movimento, in gradi (default 0 = nessuna rotazione)")
    p.add_argument("--pos-noise-std", type=float, default=0.0,
                    help="deviazione standard del rumore gaussiano di posizione, in metri (default 0 = pulito)")
    p.add_argument("--orient-noise-std-deg", type=float, default=0.0,
                    help="deviazione standard del rumore rotazionale, in gradi (default 0 = pulito)")
    p.add_argument("--noise-seed", type=int, default=None,
                    help="seed per riproducibilita' del rumore (default: non deterministico)")
    p.add_argument("--output", required=True, help="percorso del CSV di output")
    p.add_argument("--output-truth", default=None,
                    help="se fornito, scrive anche la verita' nota (vel/acc/eta/eta_dot) per validare gli stimatori")
    args = p.parse_args()

    rot_axis = tuple(float(v) for v in args.rot_axis.split(","))
    if len(rot_axis) != 3:
        raise ValueError("--rot-axis deve avere 3 componenti, es. '0,0,1'")

    rows = generate(
        duration=args.duration, dx=args.dx, dy=args.dy, dz=args.dz, dt=args.dt,
        transition_duration=args.transition_duration,
        horizontal_window=(args.h_start_frac, args.h_end_frac),
        vertical_window=(args.v_start_frac, args.v_end_frac),
        rot_axis=rot_axis, rot_angle_deg=args.rot_angle_deg,
        pos_noise_std=args.pos_noise_std, orient_noise_std_deg=args.orient_noise_std_deg,
        noise_seed=args.noise_seed,
    )
    write_csv(rows, args.output)
    print(f"Scritto {args.output}: {len(rows)} campioni, ...")

    if args.output_truth:
        truth_rows = generate_truth(
            duration=args.duration, dx=args.dx, dy=args.dy, dz=args.dz, dt=args.dt,
            transition_duration=args.transition_duration,
            horizontal_window=(args.h_start_frac, args.h_end_frac),
            vertical_window=(args.v_start_frac, args.v_end_frac),
            rot_axis=rot_axis, rot_angle_deg=args.rot_angle_deg,
        )
        write_truth_csv(truth_rows, args.output_truth)
        print(f"Scritto {args.output_truth}: {len(truth_rows)} campioni (verita' nota)")


if __name__ == "__main__":
    main()
