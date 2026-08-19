#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class SinusoidCommander(Node):

    def __init__(self):
        super().__init__('testing_plane_sinusoid_commander')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/testing_plane/testing_plane_position_controller/commands',
            10
        )

        self.start_time = time.time()

        self.timer = self.create_timer(0.002, self.publish_command)  # 0.002 s = 500 Hz

        self.amplitude_x = 0.2 # [rad]
        self.amplitude_y = 0.17 # [rad]

        self.t_f_x = 200.0
        self.frequency_x = 1.0 / self.t_f_x # [Hz]

        self.t_f_y = 200.0
        self.frequency_y = 1.0 / self.t_f_y # [Hz]
        
    def publish_command(self):

        t = time.time() - self.start_time
        x_rot = self.amplitude_x * math.sin(2 * math.pi * self.frequency_x * t)
        y_rot = self.amplitude_y * math.sin(2 * math.pi * self.frequency_y * t)
        

        msg = Float64MultiArray()
        msg.data = [
            0.0,  # x
            0.0,  # y
            0.0,  # z
            x_rot,  # rot_x
            y_rot, # rot_y sinusoid
            0.0   # rot_z
        ]

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SinusoidCommander()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()