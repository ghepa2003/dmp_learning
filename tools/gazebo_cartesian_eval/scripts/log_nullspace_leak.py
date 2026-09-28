#!/usr/bin/env python3
"""
Diagnostic Logger for Nullspace Acceleration Leak.

Quantifies how much the (kinematically-projected) nullspace torque leaks into task-space
Cartesian acceleration via the anisotropic joint-space mass matrix M(q):
    leak = J(q) * M(q)^-1 * tau_nullspace  [3 linear (m/s^2) + 3 angular (rad/s^2)]

Subscribes to ~/nullspace_leak (Float64MultiArray, 6 elements) for a specified duration,
computes L2 norm statistics for linear and angular leak components, and outputs a formatted RESULT_LINE.

Usage:
    python3 log_nullspace_leak.py [topic] [duration_sec]

Example:
    python3 log_nullspace_leak.py /cartesian_impedance_controller/nullspace_leak 30.0
"""

import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped


class NullspaceLeakLogger(Node):
    def __init__(self, leak_topic: str, target_pose_topic: str,
                 inactivity_timeout_s: float, safety_timeout_s: float):
        super().__init__('nullspace_leak_logger')
        self.samples = []
        self.trajectory_started = False
        self.last_target_pose_time = None
        self.inactivity_timeout_s = inactivity_timeout_s

        self.leak_sub = self.create_subscription(
            Float64MultiArray, leak_topic, self._leak_cb, 50)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, target_pose_topic, self._target_pose_cb, 50)

        # controllo periodico di inattività (non basato sui messaggi stessi,
        # altrimenti un'assenza di messaggi non triggererebbe mai nulla)
        self._watchdog_timer = self.create_timer(0.1, self._check_inactivity)
        self._safety_timer = self.create_timer(safety_timeout_s, self._safety_timeout)

    def _target_pose_cb(self, msg: PoseStamped):
        now = time.monotonic()
        if not self.trajectory_started:
            self.trajectory_started = True
            self.get_logger().info("Primo /target_pose ricevuto: inizio accumulo.")
        self.last_target_pose_time = now

    def _leak_cb(self, msg: Float64MultiArray):
        if self.trajectory_started:
            self.samples.append(np.array(msg.data))

    def _check_inactivity(self):
        if not self.trajectory_started or self.last_target_pose_time is None:
            return
        elapsed = time.monotonic() - self.last_target_pose_time
        if elapsed > self.inactivity_timeout_s:
            self.get_logger().info(
                f"Nessun /target_pose da {elapsed:.2f}s: rollout terminato.")
            self._finish()

    def _safety_timeout(self):
        if not self.trajectory_started:
            self.get_logger().warn(
                "Safety timeout: nessun /target_pose ricevuto. "
                "Verifica che dmp_gazebo_executor_node sia in esecuzione e "
                "che il nome del topic sia corretto.")
        self._finish()

    def _finish(self):
        if not self.samples:
            print("RESULT_LINE no_samples=1", flush=True)
            os._exit(0)

        data = np.stack(self.samples)
        lin_norm = np.linalg.norm(data[:, :3], axis=1)
        ang_norm = np.linalg.norm(data[:, 3:], axis=1)

        print(
            f"RESULT_LINE n_samples={len(data)} "
            f"leak_lin_mean_m_s2={lin_norm.mean():.6f} leak_lin_max_m_s2={lin_norm.max():.6f} "
            f"leak_ang_mean_rad_s2={ang_norm.mean():.6f} leak_ang_max_rad_s2={ang_norm.max():.6f}",
            flush=True
        )
        os._exit(0)


def main():
    leak_topic = sys.argv[1] if len(sys.argv) > 1 else '/cartesian_impedance_controller/nullspace_leak'
    target_pose_topic = sys.argv[2] if len(sys.argv) > 2 else '/target_pose'
    inactivity_timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    safety_timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 180.0
    rclpy.init()
    node = NullspaceLeakLogger(leak_topic, target_pose_topic, inactivity_timeout, safety_timeout)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()