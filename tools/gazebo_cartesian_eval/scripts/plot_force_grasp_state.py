#!/usr/bin/env python3
"""Plot headless di forza stimata / stato del grasp / comando gripper.

Complementare a evaluate_cartesian_tracking_headless.py (stesso stile: backend
Agg, nessuna finestra interattiva, salva un PNG e stampa un riepilogo testuale).
Legge i CSV prodotti da extract_force_grasp_to_csv.py in ../data/ e produce
un'unica figura con tre subplot allineati sullo stesso asse temporale:

    1. Forza: norma euclidea di (fx, fy, fz) nel tempo, con linea orizzontale
       opzionale per hard_force_limit_n.
    2. Stato del grasp nel tempo (step categoriale):
       free_space=0, contact_pending=1, contact_confirmed=2, limit_action=3.
    3. Comando gripper (posizione) nel tempo.

Un CSV vuoto (solo intestazione, topic non pubblicato durante la
registrazione) non e' un errore: il subplot relativo viene lasciato vuoto con
una nota e viene stampato un warning.

Uso:
    python3 plot_force_grasp_state.py <nome_run> [hard_force_limit_n]
"""
import sys
import os
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Mapping stato testuale -> valore numerico per lo step plot categoriale.
# Valori (stringhe) verificati in core/grasp_state_machine.cpp::stateToString().
STATE_TO_LEVEL = {
    "free_space": 0,
    "contact_pending": 1,
    "contact_confirmed": 2,
    "limit_action": 3,
}
LEVEL_LABELS = ["free_space", "contact_pending", "contact_confirmed", "limit_action"]


def load_rows(path):
    """Ritorna (header, list_of_dict_rows). Se il file non esiste -> (None, [])."""
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            return reader.fieldnames, rows
    except FileNotFoundError:
        return None, []


def col(rows, name):
    return [float(r[name]) for r in rows]


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 plot_force_grasp_state.py <nome_run> [hard_force_limit_n]")
        sys.exit(1)

    run_name = sys.argv[1]
    hard_force_limit_n = None
    if len(sys.argv) >= 3:
        try:
            hard_force_limit_n = float(sys.argv[2])
        except ValueError:
            print(f"[WARNING] hard_force_limit_n='{sys.argv[2]}' non e' un numero, ignorato.")

    data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    force_path = os.path.join(data_dir, f"force_{run_name}.csv")
    state_path = os.path.join(data_dir, f"grasp_state_{run_name}.csv")
    gripper_path = os.path.join(data_dir, f"gripper_cmd_{run_name}.csv")

    _, force_rows = load_rows(force_path)
    _, state_rows = load_rows(state_path)
    _, gripper_rows = load_rows(gripper_path)

    if not force_rows:
        print(f"[WARNING] {force_path} vuoto o assente: nessun dato di forza.")
    if not state_rows:
        print(f"[WARNING] {state_path} vuoto o assente: nessun dato di grasp-state.")
    if not gripper_rows:
        print(f"[WARNING] {gripper_path} vuoto o assente: nessun dato di comando gripper.")

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Forza / grasp-state / gripper: {run_name}")
    ax_f, ax_s, ax_g = axes

    # --- 1. Forza (norma euclidea) ---
    max_force = float("nan")
    if force_rows:
        tf = col(force_rows, "t")
        fx = col(force_rows, "fx")
        fy = col(force_rows, "fy")
        fz = col(force_rows, "fz")
        fnorm = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(fx, fy, fz)]
        max_force = max(fnorm)
        ax_f.plot(tf, fnorm, color="tab:red", label="|F| stimata")
        if hard_force_limit_n is not None:
            ax_f.axhline(hard_force_limit_n, color="k", linestyle="--",
                         label=f"hard_force_limit_n = {hard_force_limit_n:.2f} N")
        ax_f.legend(loc="upper right")
    else:
        ax_f.text(0.5, 0.5, "nessun dato di forza", ha="center", va="center",
                  transform=ax_f.transAxes)
    ax_f.set_ylabel("forza [N]")
    ax_f.set_title("Norma della forza di contatto stimata")

    # --- 2. Stato del grasp (step categoriale) ---
    confirmed_window_sec = 0.0
    limit_action_reached = False
    if state_rows:
        ts = col(state_rows, "t")
        raw_states = [r["state"] for r in state_rows]
        levels = []
        for s in raw_states:
            if s not in STATE_TO_LEVEL:
                print(f"[WARNING] stato sconosciuto '{s}' nel CSV, mappato a -1.")
            levels.append(STATE_TO_LEVEL.get(s, -1))
        ax_s.step(ts, levels, where="post", color="tab:blue")
        ax_s.set_yticks(range(len(LEVEL_LABELS)))
        ax_s.set_yticklabels(LEVEL_LABELS)
        ax_s.set_ylim(-0.5, len(LEVEL_LABELS) - 0.5)

        # Somma degli intervalli in cui lo stato resta 'contact_confirmed'.
        # grasp_state_machine pubblica ad ogni callback di ingresso (non solo
        # alle transizioni): ogni run di campioni consecutivi 'contact_confirmed'
        # si estende fino al timestamp del primo campione successivo con stato
        # diverso (o all'ultimo campione disponibile).
        n = len(ts)
        i = 0
        while i < n:
            if raw_states[i] == "contact_confirmed":
                j = i
                while j + 1 < n and raw_states[j + 1] == "contact_confirmed":
                    j += 1
                end_t = ts[j + 1] if j + 1 < n else ts[j]
                confirmed_window_sec += end_t - ts[i]
                i = j + 1
            else:
                i += 1
        limit_action_reached = any(s == "limit_action" for s in raw_states)
    else:
        ax_s.text(0.5, 0.5, "nessun dato di grasp-state", ha="center", va="center",
                  transform=ax_s.transAxes)
    ax_s.set_ylabel("stato")
    ax_s.set_title("Stato della macchina a stati del grasp")

    # --- 3. Comando gripper ---
    if gripper_rows:
        tg = col(gripper_rows, "t")
        pos = col(gripper_rows, "position")
        ax_g.plot(tg, pos, color="tab:green", label="gripper_position_cmd")
        ax_g.legend(loc="upper right")
    else:
        ax_g.text(0.5, 0.5, "nessun dato di comando gripper", ha="center", va="center",
                  transform=ax_g.transAxes)
    ax_g.set_ylabel("posizione [m]")
    ax_g.set_xlabel("t [s]")
    ax_g.set_title("Comando di posizione del gripper")

    plt.tight_layout()
    out_dir = os.path.join(os.path.dirname(__file__), "..", "plots", "04_grasp_test")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"force_grasp_state_{run_name}.png")
    plt.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")

    # --- Riepilogo testuale ---
    print(f"\n=== {run_name} (forza / grasp-state) ===")
    if force_rows:
        print(f"forza massima osservata: {max_force:.3f} N")
    else:
        print("forza massima osservata: NA (nessun dato)")
    if state_rows:
        print(f"finestra contact_confirmed: {confirmed_window_sec:.3f} s")
        print(f"limit_action raggiunto: {'SI' if limit_action_reached else 'no'}")
    else:
        print("finestra contact_confirmed: NA (nessun dato)")
        print("limit_action raggiunto: NA (nessun dato)")


if __name__ == "__main__":
    main()
