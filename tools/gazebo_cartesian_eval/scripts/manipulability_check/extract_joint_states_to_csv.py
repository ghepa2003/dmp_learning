#!/usr/bin/env python3
"""
Estrae il topic /joint_states dai rosbag in formato CSV.
Salva in tools/gazebo_cartesian_eval/data/joint_states_<run_name>.csv

Colonne:
t, fer_joint1, fer_joint2, fer_joint3, fer_joint4, fer_joint5, fer_joint6, fer_joint7
"""

import sys
import os
import csv
import rclpy.serialization
from sensor_msgs.msg import JointState
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

JOINT_NAMES = [f"fer_joint{i}" for i in range(1, 8)]


def extract_joint_states(bag_path, run_name, out_dir):
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    out_file = os.path.join(out_dir, f"joint_states_{run_name}.csv")
    f = open(out_file, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["t"] + JOINT_NAMES)

    t0 = None
    count = 0

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != "/joint_states":
            continue

        msg = rclpy.serialization.deserialize_message(data, JointState)
        t = timestamp_ns * 1e-9
        if t0 is None:
            t0 = t

        # Mappa dinamica nome_giunto -> valore di posizione
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            row_q = [name_to_pos[jn] for jn in JOINT_NAMES]
        except KeyError as e:
            print(f"[ERROR] Giunto {e} non trovato nel messaggio /joint_states!")
            continue

        writer.writerow([t - t0] + row_q)
        count += 1

    f.close()
    print(f"[{run_name}] /joint_states: {count} messaggi scritti in {out_file}")


def main():
    if len(sys.argv) < 3:
        # Se non passati argomenti, estrai i 4 bag di default
        script_dir = os.path.dirname(os.path.abspath(__file__))
        eval_root = os.path.abspath(os.path.join(script_dir, "../.."))
        bags_dir = os.path.join(eval_root, "bags")
        out_dir = os.path.join(eval_root, "data")
        os.makedirs(out_dir, exist_ok=True)

        runs = [
            "reach_task_baseline_velocity_delay1",
            "reach_task_baseline_impedance_kt200_delay1",
            "reach_task_baseline_impedance_kt2000_delay1",
            "reach_task_baseline_replay_prodmp_kt200_delay1",
        ]

        for run in runs:
            bag_p = os.path.join(bags_dir, run)
            if os.path.exists(bag_p):
                print(f"\n--- Estrazione /joint_states per {run} ---")
                extract_joint_states(bag_p, run, out_dir)
            else:
                print(f"[WARN] Bag path {bag_p} non trovato.")
    else:
        bag_path = sys.argv[1]
        run_name = sys.argv[2]
        out_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.join(os.path.dirname(__file__), "../../data")
        os.makedirs(out_dir, exist_ok=True)
        extract_joint_states(bag_path, run_name, out_dir)


if __name__ == "__main__":
    main()
