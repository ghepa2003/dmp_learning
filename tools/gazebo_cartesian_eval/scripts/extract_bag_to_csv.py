#!/usr/bin/env python3
"""Estrae i topic PoseStamped da un bag rosbag2 in CSV con header,
nello stesso formato (t,x,y,z,qw,qx,qy,qz) usato dagli altri strumenti
diagnostici del progetto (vedi dmp_offline_test/plot_dmp_test.py).

Uso:
    python3 extract_bag_to_csv.py <path_al_bag> <nome_run>

Produce, dentro ../data/:
    target_aligned_<nome_run>.csv
    actual_pose_<nome_run>.csv
"""
import sys
import os
import csv

import rclpy.serialization
from geometry_msgs.msg import PoseStamped
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


TOPIC_TO_FILENAME = {
    "/velocity_cartesian_controller/target_pose_aligned": "target_aligned",
    "/velocity_cartesian_controller/actual_pose": "actual_pose",
}


def extract(bag_path, run_name, out_dir):
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr",
                                          output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    writers = {}
    for topic, short_name in TOPIC_TO_FILENAME.items():
        path = os.path.join(out_dir, f"{short_name}_{run_name}.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qw", "qx", "qy", "qz"])
        writers[topic] = (f, w)

    t0 = None
    counts = {topic: 0 for topic in TOPIC_TO_FILENAME}

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
        print(f"{topic}: {count} messaggi -> {TOPIC_TO_FILENAME[topic]}_{run_name}.csv")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: python3 extract_bag_to_csv.py <path_al_bag> <nome_run>")
        sys.exit(1)

    bag_path, run_name = sys.argv[1], sys.argv[2]
    out_dir = os.path.join(os.path.dirname(__file__), "..", "data")
    os.makedirs(out_dir, exist_ok=True)
    extract(bag_path, run_name, out_dir)