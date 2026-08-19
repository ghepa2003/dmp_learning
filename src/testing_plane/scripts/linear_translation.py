#!/usr/bin/env python3
"""
Linear point-to-point translation of the testing_plane object in Gazebo,
commanded via /testing_plane/testing_plane_position_controller/commands.

Orientation is held fixed at whatever the object's current orientation is
when this node starts (read once from /testing_plane/joint_states) - this
script only moves trasl_x/y/z, never rot_x/y/z.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


JOINT_ORDER = [
    "testing_plane_trasl_x_joint",
    "testing_plane_trasl_y_joint",
    "testing_plane_trasl_z_joint",
    "testing_plane_rot_x_joint",
    "testing_plane_rot_y_joint",
    "testing_plane_rot_z_joint",
]


def minimum_jerk_s(tau: float) -> float:
    """Minimum-jerk scalar profile s(tau) in [0,1] for tau in [0,1]: zero
    velocity and zero acceleration at both endpoints (standard 5th-order
    polynomial, same family as Hoffmann et al. canonical system smoothing -
    used here purely for Cartesian interpolation, not a full DMP)."""
    tau = min(max(tau, 0.0), 1.0)
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


class LinearTranslationNode(Node):
    def __init__(self):
        super().__init__("testing_plane_linear_translation")

        self.declare_parameter("explicit_point_a", False)
        self.declare_parameter("point_a", [0.0, 0.0, 0.0])
        self.declare_parameter("point_b", [0.0, 0.0, 0.0])
        self.declare_parameter("duration_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("profile", "minimum_jerk")  # or "linear"
        self.declare_parameter("hold_at_end", True)
        self.declare_parameter(
            "commands_topic",
            "/testing_plane/testing_plane_position_controller/commands",
        )
        self.declare_parameter("joint_states_topic", "/testing_plane/joint_states")
        self.declare_parameter("position_tolerance", 0.01)  # m

        self.explicit_point_a = bool(self.get_parameter("explicit_point_a").value)
        self.point_a_param = np.array(self.get_parameter("point_a").value, dtype=float)
        self.point_b = np.array(self.get_parameter("point_b").value, dtype=float)
        self.duration = float(self.get_parameter("duration_sec").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.profile = self.get_parameter("profile").value
        self.hold_at_end = bool(self.get_parameter("hold_at_end").value)
        self.tol = float(self.get_parameter("position_tolerance").value)

        if self.profile not in ("linear", "minimum_jerk"):
            raise ValueError(f"Unknown profile '{self.profile}'")
        if self.duration <= 0.0:
            raise ValueError(f"duration_sec must be > 0, got {self.duration}")

        commands_topic = self.get_parameter("commands_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value

        self.pub = self.create_publisher(Float64MultiArray, commands_topic, 10)

        self.current_state = None  # filled by _joint_states_cb
        self._js_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_states_cb, 10
        )

        self.get_logger().info(f"Waiting for one message on {joint_states_topic}...")
        while rclpy.ok() and self.current_state is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(self._js_sub)  # one-shot

        self.point_a, self.rot_fixed = self._resolve_start_state()

        self.get_logger().info(
            f"Translating A={self.point_a.tolist()} -> B={self.point_b.tolist()} "
            f"over {self.duration:.2f}s, profile={self.profile}, "
            f"orientation held at {self.rot_fixed.tolist()}"
        )

        self.start_time = self.get_clock().now()
        self.finished = False
        self.timer = self.create_timer(1.0 / self.rate_hz, self._step)

    def _joint_states_cb(self, msg: JointState):
        # Map by name, never assume ordering.
        if self.current_state is None:
            name_to_pos = dict(zip(msg.name, msg.position))
            try:
                self.current_state = np.array(
                    [name_to_pos[j] for j in JOINT_ORDER], dtype=float
                )
            except KeyError as e:
                self.get_logger().error(
                    f"joint_states message missing expected joint {e} - "
                    f"got names: {msg.name}"
                )

    def _resolve_start_state(self):
        current_pos = self.current_state[0:3]
        current_rot = self.current_state[3:6]

        if not self.explicit_point_a:
            return current_pos.copy(), current_rot

        point_a = self.point_a_param
        mismatch = np.linalg.norm(point_a - current_pos)
        if mismatch > self.tol:
            self.get_logger().warn(
                f"point_a {point_a.tolist()} differs from actual current "
                f"position {current_pos.tolist()} by {mismatch:.4f} m "
                f"(tolerance {self.tol} m) - this WILL cause a visible jump "
                f"at t=0, left as-is since point_a was explicitly set."
            )
        return point_a, current_rot

    def _step(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        tau = min(elapsed / self.duration, 1.0)
        s = tau if self.profile == "linear" else minimum_jerk_s(tau)

        pos = self.point_a + s * (self.point_b - self.point_a)

        msg = Float64MultiArray()
        msg.data = [
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(self.rot_fixed[0]), float(self.rot_fixed[1]), float(self.rot_fixed[2]),
        ]
        self.pub.publish(msg)

        if tau >= 1.0:
            self.get_logger().info("Translation complete.")
            if not self.hold_at_end:
                self.timer.cancel()
            self.finished = True


def main():
    rclpy.init()
    node = LinearTranslationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()