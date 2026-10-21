#!/usr/bin/env python3
import sys
import time
import math
import argparse
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import ListControllers
import tf2_ros

def quat_slerp(q0, q1, s):
    """
    Spherical linear interpolation between two quaternions q0 and q1 with factor s in [0, 1].
    Quaternions in format [x, y, z, w]. Shortest path.
    """
    q0 = np.array(q0, dtype=np.float64)
    q1 = np.array(q1, dtype=np.float64)
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.99995:
        res = q0 + s * (q1 - q0)
        return res / np.linalg.norm(res)

    dot = np.clip(dot, -1.0, 1.0)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)

    theta = theta_0 * s
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    res = (s0 * q0) + (s1 * q1)
    return res / np.linalg.norm(res)

def quat_angular_diff_deg(q1, q2):
    """Angular difference in degrees between two quaternions [x, y, z, w]."""
    q1 = np.array(q1, dtype=np.float64) / np.linalg.norm(q1)
    q2 = np.array(q2, dtype=np.float64) / np.linalg.norm(q2)
    dot = np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0)
    return float(2.0 * math.acos(dot) * 180.0 / math.pi)

def min_jerk(tau):
    """Minimum jerk polynomial s(tau) for tau in [0, 1]"""
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)

class PreposCaptureNode(Node):
    def __init__(self, args):
        super().__init__('prepos_capture_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.args = args
        self.world_frame = args.world_frame
        self.ee_frame = args.ee_frame
        self.target_pos = np.array(args.anchor_pos, dtype=np.float64)
        self.target_quat = np.array(args.anchor_quat, dtype=np.float64)
        self.target_quat = self.target_quat / np.linalg.norm(self.target_quat)
        self.ramp_duration = float(args.duration)
        self.max_error_m = float(args.max_error)
        self.max_error_mm = self.max_error_m * 1000.0
        self.rate_hz = float(args.rate)
        self.dt = 1.0 / self.rate_hz

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(PoseStamped, args.target_topic, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joy_sub = self.create_subscription(Joy, args.joy_topic, self.joy_callback, 10)
        self.prev_button0 = 0
        self.stop_requested = False
        self.start_edge_during_ramp = False
        self.hold_banner_printed = False
        self.joy_trigger_time = None

        self.p0 = None
        self.q0 = None
        self.start_sim_time = None
        self.start_wall_time = time.time()
        self.timer = None
        self.active_phase = 'INIT'

        # Tracking metrics
        self.max_tracking_error_mm = 0.0
        self.latest_error_mm = 0.0
        self.latest_error_deg = 0.0
        self.final_p_tcp = None
        self.final_q_tcp = None

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 0:
            b0 = msg.buttons[0]
            if self.prev_button0 == 0 and b0 == 1 and self.active_phase == 'RAMP':
                self.get_logger().error("start edge during ramp: arm not at anchor, discard session")
                self.start_edge_during_ramp = True
                self.stop_requested = True
            elif self.prev_button0 == 0 and b0 == 1:
                self.get_logger().info(f"Joy button 0 rising edge detected at sim_time={self.get_sim_time():.3f}s! Stopping publisher...")
                self.stop_requested = True
                self.joy_trigger_time = self.get_sim_time()
            self.prev_button0 = b0

    def get_sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def get_tcp_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.ee_frame, rclpy.time.Time())
            pos = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z], dtype=np.float64)
            quat = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w], dtype=np.float64)
            return pos, quat / np.linalg.norm(quat)
        except Exception:
            return None, None

    def wait_for_controller(self):
        self.get_logger().info(f"Checking if controller '{self.args.controller_name}' is active...")
        cli = self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /controller_manager/list_controllers service...")
            if not rclpy.ok():
                return False

        for _ in range(60):
            req = ListControllers.Request()
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done():
                res = future.result()
                if res is not None:
                    for c in res.controller:
                        if c.name == self.args.controller_name and c.state == 'active':
                            self.get_logger().info(f"Controller '{c.name}' is ACTIVE!")
                            return True
            time.sleep(0.5)

        self.get_logger().error(f"Controller '{self.args.controller_name}' did not become active within timeout!")
        return False

    def set_skip_initial_alignment(self):
        self.get_logger().info(f"Setting skip_initial_alignment=true on /{self.args.controller_name}...")
        cmd = ["ros2", "param", "set", f"/{self.args.controller_name}", "skip_initial_alignment", "true"]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode == 0 and "successful" in res.stdout.lower():
            self.get_logger().info(f"Parameter set successfully: {res.stdout.strip()}")
            return True
        else:
            self.get_logger().error(f"Param set FAILED (returncode={res.returncode}): {res.stdout.strip()} | stderr: {res.stderr.strip()}")
            return False

    def run_prepos(self):
        # 1. Wait for controller
        if not self.wait_for_controller():
            return 1

        # 2. Set skip_initial_alignment
        if not self.set_skip_initial_alignment():
            return 1

        # 3. Read initial TCP pose
        self.get_logger().info("Reading initial TCP pose from TF...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.p0, self.q0 = self.get_tcp_pose()
            if self.p0 is not None and self.get_sim_time() > 0.0:
                break
            time.sleep(0.05)

        self.start_sim_time = self.get_sim_time()
        self.get_logger().info("=== Initial State ===")
        self.get_logger().info(f"  Start sim time: {self.start_sim_time:.4f} s")
        self.get_logger().info(f"  p0: [{self.p0[0]:.4f}, {self.p0[1]:.4f}, {self.p0[2]:.4f}]")
        self.get_logger().info(f"  q0: [{self.q0[0]:.4f}, {self.q0[1]:.4f}, {self.q0[2]:.4f}, {self.q0[3]:.4f}]")
        self.get_logger().info(f"  Target p: [{self.target_pos[0]:.4f}, {self.target_pos[1]:.4f}, {self.target_pos[2]:.4f}]")
        self.get_logger().info(f"  Target q: [{self.target_quat[0]:.4f}, {self.target_quat[1]:.4f}, {self.target_quat[2]:.4f}, {self.target_quat[3]:.4f}]")
        self.get_logger().info(f"Starting {self.rate_hz:.0f} Hz trajectory publishing (Ramp {self.ramp_duration:.1f}s -> Hold until Joy button 0)...")

        last_pub_sim = 0.0
        log_throttle = 0.0

        while rclpy.ok() and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.001)
            t_sim = self.get_sim_time()

            if t_sim - last_pub_sim >= self.dt:
                elapsed = t_sim - self.start_sim_time
                if elapsed < 0.0:
                    continue

                if elapsed < self.ramp_duration:
                    tau = elapsed / self.ramp_duration
                    s = min_jerk(tau)
                    p_curr = self.p0 + s * (self.target_pos - self.p0)
                    q_curr = quat_slerp(self.q0, self.target_quat, s)
                    self.active_phase = 'RAMP'
                else:
                    p_curr = self.target_pos
                    q_curr = self.target_quat
                    self.active_phase = 'HOLD'
                    if not self.hold_banner_printed:
                        banner = "READY: tenuta alla posa di cattura, ora si può cliccare"
                        print(banner)
                        self.get_logger().info(banner)
                        self.hold_banner_printed = True

                msg = PoseStamped()
                msg.header.frame_id = self.world_frame
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(p_curr[0])
                msg.pose.position.y = float(p_curr[1])
                msg.pose.position.z = float(p_curr[2])
                msg.pose.orientation.x = float(q_curr[0])
                msg.pose.orientation.y = float(q_curr[1])
                msg.pose.orientation.z = float(q_curr[2])
                msg.pose.orientation.w = float(q_curr[3])
                self.pub.publish(msg)
                last_pub_sim = t_sim

                # Check tracking error
                p_tcp, q_tcp = self.get_tcp_pose()
                if p_tcp is not None:
                    err_pos_m = np.linalg.norm(p_tcp - p_curr)
                    err_pos_mm = err_pos_m * 1000.0
                    err_rot_deg = quat_angular_diff_deg(q_tcp, q_curr)
                    self.latest_error_mm = err_pos_mm
                    self.latest_error_deg = err_rot_deg
                    self.final_p_tcp = p_tcp
                    self.final_q_tcp = q_tcp

                    if err_pos_mm > self.max_tracking_error_mm:
                        self.max_tracking_error_mm = err_pos_mm

                    if err_pos_mm > self.max_error_mm:
                        self.get_logger().error(f"ABORT: Tracking error {err_pos_mm:.2f} mm exceeded safety limit {self.max_error_mm:.2f} mm at t={elapsed:.2f}s!")
                        return 1

                # Log status periodically
                if t_sim - log_throttle >= 2.0:
                    if self.active_phase == 'RAMP':
                        self.get_logger().info(f"[RAMP] t={elapsed:.2f}s/{self.ramp_duration:.1f}s (s={s:.3f}), err={self.latest_error_mm:.2f} mm, {self.latest_error_deg:.2f} deg")
                    else:
                        hold_t = elapsed - self.ramp_duration
                        self.get_logger().info(f"[HOLD] hold_t={hold_t:.2f}s, err={self.latest_error_mm:.2f} mm, {self.latest_error_deg:.2f} deg")
                    log_throttle = t_sim

        if self.start_edge_during_ramp:
            return 2

        # Exiting normally
        total_sim_time = self.get_sim_time() - self.start_sim_time
        total_wall_time = time.time() - self.start_wall_time

        p_tcp_end, q_tcp_end = self.get_tcp_pose()
        if p_tcp_end is not None:
            final_err_mm = np.linalg.norm(p_tcp_end - self.target_pos) * 1000.0
            final_err_deg = quat_angular_diff_deg(q_tcp_end, self.target_quat)
        else:
            final_err_mm = self.latest_error_mm
            final_err_deg = self.latest_error_deg

        print("\n=======================================================")
        print("PREPOS CAPTURE COMPLETE (Normal Exit code 0)")
        print("=======================================================")
        print(f"Total sim time:      {total_sim_time:.3f} s")
        print(f"Total wall time:     {total_wall_time:.3f} s")
        print(f"Max tracking error:  {self.max_tracking_error_mm:.3f} mm")
        print(f"Final error pos:     {final_err_mm:.3f} mm")
        print(f"Final error rot:     {final_err_deg:.3f} deg")
        if self.joy_trigger_time is not None:
            print(f"Joy triggered at:    {self.joy_trigger_time:.3f} s sim")
        print("=======================================================\n")
        return 0

def main():
    parser = argparse.ArgumentParser(description="Prepositioning script towards capture pose.")
    parser.add_argument('--anchor-pos', type=float, nargs=3, default=[0.589, 0.344, 0.35], help="Target anchor position [x, y, z]")
    parser.add_argument('--anchor-quat', type=float, nargs=4, default=[0.6903455270798549, 0.15304591873303097, 0.15304591873303086, 0.6903455270798549], help="Target anchor quaternion [x, y, z, w]")
    parser.add_argument('--duration', type=float, default=20.0, help="Ramp duration in seconds")
    parser.add_argument('--target-topic', type=str, default='/target_pose', help="Target pose topic")
    parser.add_argument('--joy-topic', type=str, default='/touch0/buttons_synced', help="Joy button topic")
    parser.add_argument('--world-frame', type=str, default='world', help="World frame ID")
    parser.add_argument('--ee-frame', type=str, default='fer_hand_tcp', help="End-effector frame ID")
    parser.add_argument('--controller-name', type=str, default='cartesian_impedance_controller', help="Controller name")
    parser.add_argument('--max-error', type=float, default=0.100, help="Max tracking error before abort in meters")
    parser.add_argument('--rate', type=float, default=200.0, help="Publishing rate in Hz")

    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = PreposCaptureNode(args)
    ret_code = node.run_prepos()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret_code)

if __name__ == '__main__':
    main()
