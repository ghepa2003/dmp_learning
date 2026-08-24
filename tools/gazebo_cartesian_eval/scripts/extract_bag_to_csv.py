#!/usr/bin/env python3
"""... (docstring invariata, solo aggiornare firma uso) ...
Uso:
    python3 extract_bag_to_csv.py <path_al_bag> <nome_run> [nome_controller]
"""
import sys
import os
import csv

import rclpy.serialization
from geometry_msgs.msg import PoseStamped
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def extract(bag_path, run_name, controller_name, out_dir):
    topic_to_filename = {
        f"/{controller_name}/target_pose_aligned": "target_aligned",
        f"/{controller_name}/actual_pose": "actual_pose",
    }

    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    writers = {}
    for topic, short_name in topic_to_filename.items():
        path = os.path.join(out_dir, f"{short_name}_{run_name}.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        writers[topic] = (f, w)

    t0 = None
    counts = {topic: 0 for topic in topic_to_filename}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in writers:
            continue
        msg = rclpy.serialization.deserialize_message(data, PoseStamped)
        t = timestamp_ns * 1e-9
        if t0 is None:
            t0 = t
        _, w = writers[topic]
        p, q = msg.pose.position, msg.pose.orientation
        w.writerow([t - t0, p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        counts[topic] += 1

    for f, _ in writers.values():
        f.close()

    for topic, count in counts.items():
        print(f"{topic}: {count} messaggi -> {topic_to_filename[topic]}_{run_name}.csv")


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4):
        print("Uso: python3 extract_bag_to_csv.py <path_al_bag> <nome_run> [nome_controller]")
        sys.exit(1)

    bag_path, run_name = sys.argv[1], sys.argv[2]
    controller_name = sys.argv[3] if len(sys.argv) == 4 else "cartesian_impedance_controller"
    out_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    os.makedirs(out_dir, exist_ok=True)
    extract(bag_path, run_name, controller_name, out_dir)