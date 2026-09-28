#!/usr/bin/env python3
"""Estrae da un bag i topic di forza / grasp-state / gripper / target in CSV.

Complementare a extract_bag_to_csv.py (che estrae le pose per la valutazione
del tracking): stesso modo di aprire il bag via rosbag2_py, stessa convenzione
di naming degli output in ../data/.

Output (in tools/gazebo_cartesian_eval/data/):
    force_<run>.csv        : t,fx,fy,fz,tx,ty,tz   (da geometry_msgs/WrenchStamped)
    grasp_state_<run>.csv  : t,state               (da std_msgs/String)
    gripper_cmd_<run>.csv  : t,position            (da std_msgs/Float64)
    target_odom_<run>.csv  : t,x,y,z               (da nav_msgs/Odometry, sola posizione)

Un topic assente/non pubblicato durante la registrazione non e' un errore: il
CSV corrispondente viene comunque creato con la sola riga di intestazione e
viene stampato un warning.

Uso:
    python3 extract_force_grasp_to_csv.py <path_al_bag> <nome_run> [nome_controller]
"""
import sys
import os
import csv

import rclpy.serialization
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def extract(bag_path, run_name, controller_name, out_dir):
    # topic -> (short_name, ros_msg_type, header_row, row_builder(msg) -> list)
    force_topic = f"/{controller_name}/contact_wrench_estimate"
    topic_spec = {
        force_topic: (
            "force", WrenchStamped, ["t", "fx", "fy", "fz", "tx", "ty", "tz"],
            lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                       m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z],
        ),
        "/grasp_state_machine/grasp_state": (
            "grasp_state", String, ["t", "state"],
            lambda m: [m.data],
        ),
        "/gripper_position_cmd": (
            "gripper_cmd", Float64, ["t", "position"],
            lambda m: [m.data],
        ),
        "/free_target_object/odometry": (
            "target_odom", Odometry, ["t", "x", "y", "z"],
            lambda m: [m.pose.pose.position.x, m.pose.pose.position.y,
                       m.pose.pose.position.z],
        ),
    }

    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    writers = {}
    for topic, (short_name, _msg_type, header_row, _builder) in topic_spec.items():
        path = os.path.join(out_dir, f"{short_name}_{run_name}.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(header_row)
        writers[topic] = (f, w)

    t0 = None
    counts = {topic: 0 for topic in topic_spec}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in writers:
            continue
        _short_name, msg_type, _header_row, builder = topic_spec[topic]
        msg = rclpy.serialization.deserialize_message(data, msg_type)
        t = timestamp_ns * 1e-9
        if t0 is None:
            t0 = t
        _, w = writers[topic]
        w.writerow([t - t0] + builder(msg))
        counts[topic] += 1

    for f, _ in writers.values():
        f.close()

    for topic, (short_name, _msg_type, _header_row, _builder) in topic_spec.items():
        count = counts[topic]
        out_name = f"{short_name}_{run_name}.csv"
        if count == 0:
            print(f"[WARNING] {topic}: 0 messaggi nel bag -> {out_name} vuoto "
                  f"(solo intestazione). Il topic non era pubblicato durante la "
                  f"registrazione?")
        else:
            print(f"{topic}: {count} messaggi -> {out_name}")


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4):
        print("Uso: python3 extract_force_grasp_to_csv.py <path_al_bag> <nome_run> "
              "[nome_controller]")
        sys.exit(1)

    bag_path, run_name = sys.argv[1], sys.argv[2]
    controller_name = sys.argv[3] if len(sys.argv) == 4 else "cartesian_impedance_controller"
    out_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    os.makedirs(out_dir, exist_ok=True)
    extract(bag_path, run_name, controller_name, out_dir)
