#!/usr/bin/env python3
"""
Sinusoidal Orientation Excitation Node for the Testing Plane in Gazebo.

Commands sinusoidal pitch/roll oscillatory motion to `rot_x` and `rot_y` joints
via `/testing_plane/testing_plane_position_controller/commands` at 500 Hz.
Used for evaluating end-effector compliance and surface adaptation under dynamic inclination changes.
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class SinusoidCommander(Node):
    def __init__(self):
        super().__init__('testing_plane_sinusoid_commander')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/testing_plane/testing_plane_position_controller/commands',
            10
        )

        self.start_time = time.time()

        # Publish commands at 500 Hz (0.002 s interval)
        self.timer = self.create_timer(0.002, self.publish_command)

        # Oscillation amplitudes in radians
        self.amplitude_x = 0.2   # ~11.5 deg around X axis
        self.amplitude_y = 0.17  # ~9.7 deg around Y axis

        # Oscillation periods in seconds
        self.t_f_x = 200.0
        self.frequency_x = 1.0 / self.t_f_x  # Hz

        self.t_f_y = 200.0
        self.frequency_y = 1.0 / self.t_f_y  # Hz

    def publish_command(self):
        t = time.time() - self.start_time
        x_rot = self.amplitude_x * math.sin(2 * math.pi * self.frequency_x * t)
        y_rot = self.amplitude_y * math.sin(2 * math.pi * self.frequency_y * t)

        # Command format: [x, y, z, rot_x, rot_y, rot_z]
        msg = Float64MultiArray()
        msg.data = [
            0.0,    # transl_x
            0.0,    # transl_y
            0.0,    # transl_z
            x_rot,  # rot_x
            y_rot,  # rot_y
            0.0     # rot_z
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()