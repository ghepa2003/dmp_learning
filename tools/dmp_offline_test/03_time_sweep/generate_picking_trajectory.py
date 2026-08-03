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


def min_jerk_profile(t, t0, t1, start, end):
    """Profilo posizione 1D minimum-jerk tra start ed end sulla finestra
    [t0, t1]. Costante = start prima di t0, costante = end dopo t1.
    Velocita' e accelerazione nulle a entrambi gli estremi della finestra
    (continuita' C2), stessa famiglia di min_jerk_step.m."""
    if t <= t0:
        return start
    if t >= t1:
        return end
    s = (t - t0) / (t1 - t0)
    smoothed = 10 * s**3 - 15 * s**4 + 6 * s**5
    return start + (end - start) * smoothed


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


def generate(duration, dx, dy, dz, dt=0.001, transition_duration=10.0,
             horizontal_window=(0.0, 0.6), vertical_window=(0.4, 1.0),
             y0=(0.0, 0.0, 0.0), q0=(1.0, 0.0, 0.0, 0.0),
             rot_axis=(0.0, 0.0, 1.0), rot_angle_deg=0.0):
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
    posizione) sull'intera finestra di transizione [0, transition_duration]
    -- quindi la rotazione e' sincronizzata con l'inizio/fine del movimento
    di traslazione, non con una finestra indipendente.

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

    n_samples = int(round(duration / dt)) + 1
    rows = []
    for k in range(n_samples):
        t = k * dt
        x = min_jerk_profile(t, hx0, hx1, x0, x0 + dx)
        y = min_jerk_profile(t, hx0, hx1, y0_, y0_ + dy)
        z = min_jerk_profile(t, vz0, vz1, z0, z0 + dz)

        if rot_angle_deg != 0.0:
            blend = min_jerk_blend(t, 0.0, transition_duration)
            qw, qx, qy, qz = slerp(q0, q_target, blend)
        else:
            qw, qx, qy, qz = q0

        rows.append((t, x, y, z, qw, qx, qy, qz))
    return rows


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
    p.add_argument("--output", required=True, help="percorso del CSV di output")
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
    )
    write_csv(rows, args.output)
    print(f"Scritto {args.output}: {len(rows)} campioni, durata totale {args.duration}s "
          f"(transizione {args.transition_duration}s + tenuta {args.duration - args.transition_duration:.1f}s), "
          f"goal finale = ({rows[-1][1]:.4f}, {rows[-1][2]:.4f}, {rows[-1][3]:.4f})")


if __name__ == "__main__":
    main()
