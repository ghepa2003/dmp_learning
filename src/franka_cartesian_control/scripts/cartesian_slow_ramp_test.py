#!/usr/bin/env python3
"""
Standalone, use-once validation tool for capturing the sensorless
contact-wrench estimate's (~/contact_wrench_estimate) free-space tracking
noise under slow, constant-velocity motion (as opposed to
cartesian_push_test.py's quasi-static step).

Publishes a straight-line, constant-velocity Cartesian ramp on the exact
topic CartesianImpedanceController listens on for its reference target
(/target_pose by default, geometry_msgs/msg/PoseStamped -- see
target_pose_topic in franka_cartesian_control/config), starting from the
robot's current end-effector pose and moving at a fixed velocity along one
axis, orientation unchanged.

This script is intentionally independent of the DMP and Geomagic pipelines:
it does not start, subscribe to, or otherwise touch either one.

IMPORTANT PRECONDITION -- read before running:
CartesianImpedanceController uses a FrameAligner (ros_utils.hpp) that
captures a rigid SE(3) offset from the FIRST /target_pose message received
since the controller was last activated, anchoring it to the end-effector
pose *at activation*. That offset then stays locked for the rest of the
activation cycle. To get a predictable ramp starting from the CURRENT pose,
this script must be the first thing to publish on /target_pose since the
controller was activated (i.e. run it right after activating
cartesian_impedance_controller, before starting any DMP replay or haptic
streaming). If something else already published a target first, the
alignment offset is already locked to that publisher's frame and this
script's ramp will not start where printed below.

Current end-effector pose is read via tf2 (base_frame -> ee_frame), not via
the controller's own ~/actual_pose topic: that topic is only published by
CartesianImpedanceController AFTER it has received at least one target_pose
message (see its update() loop), so it cannot be used to bootstrap the very
first message. tf2 works immediately because robot_state_publisher broadcasts
it from /joint_states regardless of controller/target state.

Usage example:
    python3 cartesian_slow_ramp_test.py --axis z --velocity 0.015 --duration 15.0
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros


AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}

# Brief burst of "zero-shift" messages (raw pose == current pose) sent before
# the actual ramp, to reliably latch the FrameAligner's offset at ~0 even if
# the very first packet is lost. Not part of the ramp: every message in this
# burst carries the identical, unshifted pose.
ZERO_SHIFT_BURST_SECONDS = 0.5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a constant-velocity Cartesian ramp on the "
                    "CartesianImpedanceController target topic, to capture "
                    "free-space tracking noise for the contact-force "
                    "estimate.")
    parser.add_argument('--axis', choices=['x', 'y', 'z'], default='z',
                         help="Base-frame axis to move along (default: z).")
    parser.add_argument('--velocity', type=float, default=0.015,
                         help="Constant velocity in m/s along --axis "
                              "(default: 0.015).")
    parser.add_argument('--duration', type=float, default=15.0,
                         help="Ramp duration in seconds, after the "
                              "zero-shift burst (default: 15.0).")
    parser.add_argument('--rate', type=float, default=200.0,
                         help="Publish rate in Hz (default: 200.0, matching "
                              "this project's other /target_pose publishers: "
                              "dmp_gazebo_executor_node and "
                              "csv_master_pose_player_node).")
    parser.add_argument('--target-topic', default='/target_pose',
                         help="Topic CartesianImpedanceController listens on "
                              "for its Cartesian reference "
                              "(default: /target_pose, matching "
                              "target_pose_topic in the controller config).")
    parser.add_argument('--base-frame', default='fer_link0',
                         help="Robot base frame (default: fer_link0, matching "
                              "the controller's ee_frame_name_ convention).")
    parser.add_argument('--ee-frame', default='fer_hand_tcp',
                         help="End-effector frame (default: fer_hand_tcp, "
                              "matching ee_frame_name in the controller "
                              "config).")
    parser.add_argument('--tf-timeout', type=float, default=5.0,
                         help="Seconds to wait for the tf transform before "
                              "giving up (default: 5.0).")
    return parser.parse_args()


class CartesianSlowRampTest(Node):
    def __init__(self, args):
        super().__init__('cartesian_slow_ramp_test')
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, args.target_topic, 10)

    def lookup_current_pose(self):
        deadline = self.get_clock().now() + Duration(seconds=self.args.tf_timeout)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.base_frame, self.args.ee_frame, rclpy.time.Time())
                return tf
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def make_pose(self, tf, position):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation = tf.transform.rotation
        return msg


def main():
    args = parse_args()
    rclpy.init()
    node = CartesianSlowRampTest(args)

    try:
        tf = node.lookup_current_pose()
        if tf is None:
            node.get_logger().error(
                f"Timed out after {args.tf_timeout}s waiting for tf "
                f"{args.base_frame} -> {args.ee_frame}. Is robot_state_publisher "
                f"running and is the robot spawned?")
            sys.exit(1)

        t = tf.transform.translation
        current_position = [t.x, t.y, t.z]
        total_displacement = args.velocity * args.duration
        end_position = list(current_position)
        end_position[AXIS_INDEX[args.axis]] += total_displacement

        print(f"Start pose ({args.base_frame} -> {args.ee_frame}): "
              f"position=({current_position[0]:.5f}, {current_position[1]:.5f}, "
              f"{current_position[2]:.5f})  "
              f"orientation=(x={tf.transform.rotation.x:.5f}, "
              f"y={tf.transform.rotation.y:.5f}, z={tf.transform.rotation.z:.5f}, "
              f"w={tf.transform.rotation.w:.5f})")
        print(f"Ramp: velocity={args.velocity:+.5f} m/s along {args.axis}, "
              f"duration={args.duration:.1f}s "
              f"-> total displacement={total_displacement:+.5f} m, "
              f"end position=({end_position[0]:.5f}, {end_position[1]:.5f}, "
              f"{end_position[2]:.5f})  orientation unchanged")
        print(f"Publishing on '{args.target_topic}' at {args.rate:.1f} Hz: "
              f"{ZERO_SHIFT_BURST_SECONDS:.1f}s zero-shift burst (latches the "
              f"FrameAligner offset at ~0), then ramping at constant velocity "
              f"for {args.duration:.1f}s. Ctrl+C to stop early.")

        period = 1.0 / args.rate
        n_zero_shift = max(1, int(ZERO_SHIFT_BURST_SECONDS * args.rate))

        for _ in range(n_zero_shift):
            node.pub.publish(node.make_pose(tf, current_position))
            rclpy.spin_once(node, timeout_sec=period)

        ramp_start = node.get_clock().now()
        axis_idx = AXIS_INDEX[args.axis]
        duration = Duration(seconds=args.duration)
        while rclpy.ok():
            elapsed = node.get_clock().now() - ramp_start
            if elapsed >= duration:
                break
            elapsed_s = elapsed.nanoseconds * 1e-9
            target_position = list(current_position)
            target_position[axis_idx] += args.velocity * elapsed_s
            node.pub.publish(node.make_pose(tf, target_position))
            rclpy.spin_once(node, timeout_sec=period)

        print("Ramp duration elapsed. Stopping without publishing further.")

    except KeyboardInterrupt:
        print("\nInterrupted. Stopping without publishing further.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
