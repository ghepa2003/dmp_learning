#!/usr/bin/env python3
"""
tools/eval_controllers.py — Esecutore e valutatore canonico dei controllori Franka in Gazebo Sim.
Supporta ProDMP, DMP e Velocity Cartesian Controller con validazione stringente su sim_time e gap.
"""

import os
import sys
import time
import argparse
import subprocess
import signal
import json
import csv
import glob
import sqlite3
import numpy as np

import eval_common as ec

WS_ROOT = "/root/thesis_ws"
FRANKA_WS = "/root/ros_workspaces/ros2/franka_ws"

# Riferimenti storici (dal diario del 07/10)
HISTORICAL_REFERENCES = {
    "prodmp_kt2000": {
        "pos_media_mm": 0.52,
        "pos_max_mm": 5.92,
        "pos_max_1s_mm": 1.37,
        "pos_finale_mm": 0.37,
        "rtf": 0.60
    },
    "prodmp_kt200": {
        "pos_media_mm": 5.24,
        "pos_max_mm": 12.80,
        "pos_max_1s_mm": 8.15,
        "pos_finale_mm": 4.12,
        "rtf": 0.60
    }
}


def _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers, reason):
    """Marca la run INVALIDA: scrive metrics.json e stampa il riepilogo."""
    summary = dict(run_info)
    summary.update({
        "run_name": run_name, "controller_type": controller_type,
        "kt": kt, "dt": dt, "start_pose": start_pose, "fingers": fingers,
        "is_valid": False, "validation_reason": reason,
    })
    json_path = os.path.join(output_dir, f"{run_name}_metrics.json")
    with open(json_path, "w") as f_json:
        json.dump(summary, f_json, indent=2)
    print("\n=================================================================")
    print(f"RISULTATI RUN: {run_name}")
    print("=================================================================")
    print(f"Stato Validità:          INVALIDA ({reason})")
    print(f"Metriche salvate in:     {json_path}")
    print("=================================================================\n")
    return summary


def run_canonical_evaluation(
    output_dir,
    controller_type="prodmp",
    kt=2000.0,
    dt=10.0,
    kr=150.0,
    dr=1.0,
    start_pose="ready",
    fingers="fixed",
    suffix="_canon1008",
    force=False,
    spawn_check_only=False
):
    # 1. Controllo directory di output e run esistente
    os.makedirs(output_dir, exist_ok=True)
    bags_dir = os.path.join(output_dir, "bags")
    os.makedirs(bags_dir, exist_ok=True)

    run_name = f"eval_{controller_type}_kt{int(kt)}_{start_pose}_{fingers}{suffix}"
    bag_path = os.path.join(bags_dir, run_name)
    if os.path.exists(bag_path):
        if not force:
            print(f"[ERRORE FATALE] La run '{run_name}' esiste già in '{bag_path}'. Rifiuto di sovrascrivere (usa --force).")
            sys.exit(1)
        else:
            import shutil
            shutil.rmtree(bag_path)

    launch_log = os.path.join(output_dir, f"launch_{run_name}.log")
    exec_log = os.path.join(output_dir, f"exec_{run_name}.log")

    # Durata nominale tau e margine esplicito di sim_time
    if controller_type == "prodmp":
        tau_nom_expected = 73.8
        sim_margin = 10.0
    elif controller_type == "dmp":
        tau_nom_expected = 60.97
        sim_margin = 10.0
    else:
        tau_nom_expected = 40.0
        sim_margin = 10.0
    target_sim_time = tau_nom_expected + sim_margin

    print("=================================================================")
    print(f"AVVIO VALUTAZIONE CANONICA: {run_name}")
    print(f"  Controllore:   {controller_type} (Kt={kt:.1f}, Dt={dt:.1f})")
    print(f"  Posa & Dita:   start_pose={start_pose}, fingers={fingers}")
    print(f"  Target Sim:    tau={tau_nom_expected:.2f}s + margine={sim_margin:.2f}s -> {target_sim_time:.2f}s (NO wall-clock)")
    print(f"  Output Dir:    {output_dir}")
    print("=================================================================")

    # Metadati di tracciabilita' della run (git, tipo di build) e ambiente richiesto al launch
    meta = ec.run_metadata()
    run_info = {
        "run_metadata": meta,
        "FRANKA_START_POSE": start_pose,
        "FRANKA_FINGERS": fingers,
        "spawn_check_only": bool(spawn_check_only),
    }
    print(f"  Metadati:      git={meta['git_head']} dirty={meta['git_dirty']} build_type={meta['cmake_build_type']}")

    # Override xacro: l'installato deve coincidere col repo, prima di avviare qualsiasi cosa
    ov_ok, ov_details = ec.check_overrides_in_sync()
    meta["xacro_overrides"] = ov_details
    print(f"  Override xacro: {ov_details['status']}")
    for m in ov_details["messages"]:
        print(f"  {m}")
    if ov_ok is False:
        reason = "; ".join(ov_details["messages"])
        summary = dict(run_info)
        summary.update({
            "run_name": run_name, "controller_type": controller_type,
            "kt": kt, "dt": dt, "start_pose": start_pose, "fingers": fingers,
            "is_valid": False, "validation_reason": reason,
        })
        json_path = os.path.join(output_dir, f"{run_name}_metrics.json")
        with open(json_path, "w") as f_json:
            json.dump(summary, f_json, indent=2)
        print("\n=================================================================")
        print(f"RISULTATI RUN: {run_name}")
        print("=================================================================")
        print(f"Stato Validità:          INVALIDA ({reason})")
        print(f"Metriche salvate in:     {json_path}")
        print("=================================================================\n")
        return summary

    # Ambiente ROS nella shell che lancia lo script: prima di toccare processi o avviare Gazebo
    ros_err = ec.check_ros_python_env()
    if ros_err:
        reason = (f"AMBIENTE ROS NON CARICATO NELLA SHELL CHE LANCIA LO SCRIPT ({ros_err}). "
                  f"Esegui: {ec.ROS_ENV_HINT}")
        print(f"[ERRORE FATALE] {reason}")
        return _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers, reason)

    # Stato pulito: yaml dei guadagni tracciato = HEAD, nessun'altra run in corso (prima di toccare processi)
    others = ec.find_other_eval_runs()
    if others:
        return _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers,
                              "ALTRA RUN IN ESECUZIONE: PID " + ", ".join(str(p) for p in others))
    yaml_ok, yaml_msg = ec.check_gains_yaml_clean()
    if not yaml_ok:
        return _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers, yaml_msg)

    # 2. Setup e avvio ambiente. SIGTERM -> eccezione; ProcessGuard chiude i processi e YamlGainManager
    # ripristina la copia nel bringup (il file in src non viene mai scritto), anche su KeyboardInterrupt/SIGTERM.
    ec.clean_processes()
    ec.setup_bringup_share()

    with ec.SigtermAsException(), ec.YamlGainManager(kt=kt, dt=dt, kr=kr, dr=dr), ec.ProcessGuard():
        # Selezione launch file
        if controller_type == "velocity":
            launch_file = "gazebo_velocity_cartesian_control.launch.py"
            controller_name = "velocity_cartesian_controller"
        else:
            launch_file = "gazebo_cartesian_impedance_control.launch.py"
            controller_name = "cartesian_impedance_controller"

        launch_cmd = (
            f"export PYTHONUNBUFFERED=1 && "
            f"export FRANKA_START_POSE={start_pose} && "
            f"export FRANKA_FINGERS={fingers} && "
            f"source /opt/ros/humble/setup.bash && "
            f"source {FRANKA_WS}/install/setup.bash && "
            f"source {WS_ROOT}/install/setup.bash && "
            f"export QT_QPA_PLATFORM=offscreen && "
            f"stdbuf -oL -eL ros2 launch franka_gazebo_bringup {launch_file} "
            f"headless:=true load_gripper:=true"
        )

        with open(launch_log, "w") as f_log:
            launch_proc = subprocess.Popen(
                launch_cmd, shell=True, executable="/bin/bash",
                stdout=f_log, stderr=subprocess.STDOUT
            )

        print("Attesa attivazione controller...")
        try:
            activated = ec.wait_for_controller(launch_log, controller_name=controller_name, timeout_sec=60,
                                               launch_proc=launch_proc)
            fail_reason = f"controller {controller_name} non attivato entro il timeout"
        except ec.RunInvalid as e:
            activated, fail_reason = False, str(e)
        if not activated:
            print(f"[ERRORE FATALE] {fail_reason}")
            ec.clean_processes()  # ProcessGuard lo rifa' comunque all'uscita
            return _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers,
                                  fail_reason)

        # Guadagni effettivamente letti dal controller_manager (copia del bringup)
        gains = ec.read_gains_yaml()
        run_info["controller_gains_yaml"] = gains
        expected = {"Kt": kt, "Kr": kr, "Dt": dt, "Dr": dr}
        bad_gains = [k for k, v in expected.items() if gains.get(k) != round(v, 1)]
        print(f"  YAML guadagni: sha256={gains.get('sha256')} Kt={gains.get('Kt')} Kr={gains.get('Kr')} "
              f"Dt={gains.get('Dt')} Dr={gains.get('Dr')}")
        if bad_gains:
            reason = f"GUADAGNI NEL YAML DEL CONTROLLER DIVERSI DAI RICHIESTI: {bad_gains} ({gains})"
            print(f"[ERRORE FATALE] {reason}")
            ec.clean_processes()
            return _write_invalid(output_dir, run_name, run_info, controller_type, kt, dt, start_pose, fingers,
                                  reason)

        # Guardia sulla posa di partenza: prima di bag, executor e qualsiasi target
        print("Controller ATTIVO! Verifica posa di partenza (q6 da /joint_states)...", flush=True)
        q6_read, q6_reason = ec.read_joint6_from_joint_states()
        j6_lines, j6_init = ec.extract_joint6_initial_value_lines(launch_log)
        guard_ok, guard_msg = ec.check_start_pose(q6_read, start_pose, unread_reason=q6_reason)
        run_info.update({
            "q6_rad": q6_read,
            "q6_unread_reason": q6_reason,
            "found_initial_value_j6_lines": j6_lines,
            "start_pose_guard": {"ok": guard_ok, "message": guard_msg},
        })
        print(f"  FRANKA_START_POSE={start_pose} FRANKA_FINGERS={fingers}")
        print(f"  q6 letto = {q6_read}")
        for l in j6_lines:
            print(f"  [launch log, giunto 6] {l}")
        print(f"  Guardia: {guard_msg}", flush=True)

        if (not guard_ok) or spawn_check_only:
            ec.clean_processes()
            if guard_ok:
                reason = "controllo di sola posa di spawn (--spawn-check-only): nessun executor, nessuna traiettoria"
            else:
                reason = guard_msg
            summary = dict(run_info)
            summary.update({
                "run_name": run_name, "controller_type": controller_type,
                "kt": kt, "dt": dt, "start_pose": start_pose, "fingers": fingers,
                "is_valid": bool(guard_ok), "validation_reason": reason,
            })
            json_path = os.path.join(output_dir, f"{run_name}_metrics.json")
            with open(json_path, "w") as f_json:
                json.dump(summary, f_json, indent=2)
            print("\n=================================================================")
            print(f"RISULTATI RUN: {run_name}")
            print("=================================================================")
            print(f"Stato Validità:          {'VALIDA' if guard_ok else 'INVALIDA'} ({reason})")
            print(f"Guardia posa di partenza: {guard_msg}")
            print(f"Metadati:                {meta}")
            print(f"Metriche salvate in:     {json_path}")
            print("=================================================================\n")
            return summary

        print("Avvio registrazione bag...")
        bag_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 bag record -o {bag_path} "
            f"/{controller_name}/target_pose_aligned "
            f"/{controller_name}/actual_pose "
            f"/joint_states "
            f"/target_pose "
            f"/clock"
        )
        bag_proc = subprocess.Popen(
            bag_cmd, shell=True, executable="/bin/bash",
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(2.0)

        # Avvio executor ProDMP / DMP
        print(f"Avvio executor {controller_type}...", flush=True)
        if controller_type == "prodmp":
            exec_cmd = (
                f"source /opt/ros/humble/setup.bash && "
                f"source {FRANKA_WS}/install/setup.bash && "
                f"source {WS_ROOT}/install/setup.bash && "
                f"ros2 run haptic_dmp_learning prodmp_gazebo_executor_node "
                f"--ros-args "
                f"-p target_pose_topic:=/target_pose "
                f"-p target_twist_topic:=/target_twist "
                f"-p weights_yaml_path:={ec.WEIGHTS_PRODMP} "
                f"-p orientation_weights_yaml_path:={ec.ORIENTATION_PATH} "
                f"-p target_odom_required:=false"
            )
        elif controller_type == "dmp":
            exec_cmd = (
                f"source /opt/ros/humble/setup.bash && "
                f"source {FRANKA_WS}/install/setup.bash && "
                f"source {WS_ROOT}/install/setup.bash && "
                f"ros2 run haptic_dmp_learning dmp_gazebo_executor_node "
                f"--ros-args "
                f"-p target_pose_topic:=/target_pose"
            )
        else:
            exec_cmd = None

        if exec_cmd:
            with open(exec_log, "w") as f_exec:
                exec_proc = subprocess.Popen(
                    exec_cmd, shell=True, executable="/bin/bash",
                    stdout=f_exec, stderr=subprocess.STDOUT
                )
        else:
            exec_proc = None

        # 3. Monitoraggio tempo simulato (da /clock via ROS 2)
        print("Monitoraggio tempo simulato (/clock)...")
        clock_monitor = ec.SimTimeClockMonitor()
        sim_start_time = None
        wall_start = time.time()
        while sim_start_time is None:
            sim_start_time = clock_monitor.get_time()
            time.sleep(0.2)
            if (time.time() - wall_start) > 30.0:
                print("\n[ERRORE] Nessun messaggio /clock ricevuto entro 30s.")
                break

        if sim_start_time is not None:
            print(f"Misura avviata a sim_time = {sim_start_time:.3f} s. Target = {target_sim_time:.1f} s sim.")
            while True:
                now_sim = clock_monitor.get_time()
                if now_sim is not None:
                    current_sim_time = now_sim - sim_start_time
                    wall_elapsed = time.time() - wall_start
                    rtf_curr = current_sim_time / wall_elapsed if wall_elapsed > 0 else 0.0
                    print(f"\r  Sim time: {current_sim_time:5.2f} / {target_sim_time:.1f} s | Wall: {wall_elapsed:5.1f} s | RTF: {rtf_curr:.2f}", end="", flush=True)

                    if current_sim_time >= target_sim_time:
                        print(f"\n[OK] Raggiunto target sim_time ({current_sim_time:.2f} s >= {target_sim_time:.2f} s)!")
                        break

                # Watchdog di sicurezza wall-clock a 3x target per evitare blocco infinito
                if (time.time() - wall_start) > 3.0 * target_sim_time:
                    print(f"\n[ATTENZIONE] Watchdog wall-clock superato. Termine run...")
                    break
                time.sleep(0.5)

        clock_monitor.stop()

        # 4. Arresto pulito
        if exec_proc and exec_proc.poll() is None:
            exec_proc.send_signal(signal.SIGINT)
            try:
                exec_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                exec_proc.kill()

        if bag_proc.poll() is None:
            bag_proc.send_signal(signal.SIGINT)
            try:
                bag_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                bag_proc.kill()

        ec.clean_processes()

    # 5. Estrazione e calcolo metriche
    print("\nEstrazione dati da rosbag...")
    bag_data = ec.extract_bag_data(bag_path)
    metrics = ec.compute_metrics(
        bag_data["target_records"],
        bag_data["actual_records"],
        bag_data["clock_records"]
    )

    if not metrics:
        print("[ERRORE FATALE] Impossibile calcolare le metriche (dati mancanti nel bag).")
        return None

    # 6. Validazione canonica
    is_valid, validation_reason = ec.validate_run(
        bag_data["target_records"],
        bag_data["clock_records"],
        tau_nom=metrics["tau_nom"],
        exclude_window_sec=5.0
    )
    metrics["is_valid"] = is_valid
    metrics["validation_reason"] = validation_reason
    metrics["run_name"] = run_name
    metrics["controller_type"] = controller_type
    metrics["kt"] = kt
    metrics["dt"] = dt
    metrics["start_pose"] = start_pose
    metrics["fingers"] = fingers
    metrics.update(run_info)

    # Salva CSV dei record estratti
    csv_out_path = os.path.join(output_dir, f"{run_name}_records.csv")
    with open(csv_out_path, "w", newline="") as f_csv:
        w = csv.writer(f_csv)
        w.writerow(["t_sim", "t_wall", "target_x", "target_y", "target_z", "actual_x", "actual_y", "actual_z"])
        n_pts = min(len(bag_data["target_records"]), len(bag_data["actual_records"]))
        for i in range(n_pts):
            tr = bag_data["target_records"][i]
            ar = bag_data["actual_records"][i]
            w.writerow([
                f"{tr['t_sim']:.6f}", f"{tr['t_wall']:.6f}",
                f"{tr['pos'][0]:.6f}", f"{tr['pos'][1]:.6f}", f"{tr['pos'][2]:.6f}",
                f"{ar['pos'][0]:.6f}", f"{ar['pos'][1]:.6f}", f"{ar['pos'][2]:.6f}"
            ])

    # Salva JSON metriche
    json_path = os.path.join(output_dir, f"{run_name}_metrics.json")
    with open(json_path, "w") as f_json:
        json.dump(metrics, f_json, indent=2)

    # 7. Stampa report e confronto storico
    print("\n=================================================================")
    print(f"RISULTATI RUN: {run_name}")
    print("=================================================================")
    print(f"Stato Validità:          {'VALIDA' if is_valid else 'INVALIDA'} ({validation_reason})")
    print(f"Guardia posa di partenza: {run_info['start_pose_guard']['message']}")
    print(f"Posa/dita/q6:            FRANKA_START_POSE={start_pose} FRANKA_FINGERS={fingers} q6={run_info['q6_rad']}")
    print(f"Giunto 6 (launch log):   {run_info['found_initial_value_j6_lines']}")
    print(f"Metadati:                {meta}")
    print(f"Tempo sim effettivo:     {metrics['t_sim_ultimo']:.3f} s (tau: {metrics['tau_nom']:.3f} s, frazione: {metrics['completed_frac']*100:.1f}%)")
    print(f"RTF medio:               {metrics['rtf']:.3f}")
    print(f"Max gap target_pose:     {metrics['max_gap_tgt_ms']:.2f} ms")
    print(f"Errore pos medio:        {metrics['pos_media_mm']:.3f} mm")
    print(f"Errore pos p95:          {metrics['pos_p95_mm']:.3f} mm")
    print(f"Errore pos max:          {metrics['pos_max_mm']:.3f} mm (a t=0s)")
    print(f"Errore pos max (>1s):    {metrics['pos_max_1s_mm']:.3f} mm")
    print(f"Errore pos finale (tau): {metrics['pos_finale_tau_mm']:.3f} mm")
    print(f"Errore pos finale (end): {metrics['pos_finale_ultimo_mm']:.3f} mm")
    print(f"Errore rot medio:        {metrics['ori_media_deg']:.3f} deg (max: {metrics['ori_max_deg']:.3f} deg, end: {metrics['ori_finale_deg']:.3f} deg)")

    ref_key = f"{controller_type}_kt{int(kt)}"
    if ref_key in HISTORICAL_REFERENCES:
        ref = HISTORICAL_REFERENCES[ref_key]
        print("\n-----------------------------------------------------------------")
        print(f"CONFRONTO CON TABELLA STORICA DIARIO ({ref_key}):")
        print("-----------------------------------------------------------------")
        print(f"  Metrica               | Atteso (07/10) | Misurato Oggi | Differenza")
        print(f"  Posizione media [mm]  | {ref['pos_media_mm']:14.2f} | {metrics['pos_media_mm']:13.3f} | {metrics['pos_media_mm'] - ref['pos_media_mm']:+10.3f}")
        print(f"  Posizione max [mm]    | {ref['pos_max_mm']:14.2f} | {metrics['pos_max_mm']:13.3f} | {metrics['pos_max_mm'] - ref['pos_max_mm']:+10.3f}")
        print(f"  Pos max dopo 1s [mm]  | {ref['pos_max_1s_mm']:14.2f} | {metrics['pos_max_1s_mm']:13.3f} | {metrics['pos_max_1s_mm'] - ref['pos_max_1s_mm']:+10.3f}")
        print(f"  Pos finale [mm]       | {ref['pos_finale_mm']:14.2f} | {metrics['pos_finale_tau_mm']:13.3f} | {metrics['pos_finale_tau_mm'] - ref['pos_finale_mm']:+10.3f}")
        print(f"  RTF medio             | {ref['rtf']:14.2f} | {metrics['rtf']:13.3f} | {metrics['rtf'] - ref['rtf']:+10.3f}")

    print("=================================================================\n")
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Valutatore canonico controllori Franka Gazebo")
    parser.add_argument("output_dir", help="Cartella unica di output (non deve esistere)")
    parser.add_argument("--controller", default="prodmp", choices=["prodmp", "dmp", "velocity"], help="Tipo di controllore")
    parser.add_argument("--kt", type=float, default=2000.0, help="Rigidezza traslazionale Kt [N/m]")
    parser.add_argument("--dt", type=float, default=10.0, help="Smorzamento traslazionale Dt [Ns/m]")
    parser.add_argument("--kr", type=float, default=150.0, help="Rigidezza rotazionale Kr [Nm/rad]")
    parser.add_argument("--dr", type=float, default=1.0, help="Smorzamento rotazionale Dr [Nms/rad]")
    parser.add_argument("--start-pose", default="ready", help="Posa di spawn (ready, ready_q6pi, capture)")
    parser.add_argument("--fingers", default="fixed", choices=["fixed", "fixed_open", "prismatic"], help="Tipo giunto dita")
    parser.add_argument("--suffix", default="_canon1008", help="Suffisso per il nome della run")
    parser.add_argument("--force", action="store_true", help="Forza sovrascrittura della run se già presente")
    parser.add_argument("--spawn-check-only", action="store_true",
                        help="Dopo la guardia sulla posa di partenza scrive riepilogo e metrics.json, chiude Gazebo e termina "
                             "senza avviare executor né traiettorie")

    args = parser.parse_args()
    run_canonical_evaluation(
        output_dir=args.output_dir,
        controller_type=args.controller,
        kt=args.kt,
        dt=args.dt,
        kr=args.kr,
        dr=args.dr,
        start_pose=args.start_pose,
        fingers=args.fingers,
        suffix=args.suffix,
        force=args.force,
        spawn_check_only=args.spawn_check_only
    )


if __name__ == "__main__":
    main()
