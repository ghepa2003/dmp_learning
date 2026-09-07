#!/usr/bin/env python3
"""
Geometric grasp monitor (standalone, non-real-time).

Compares the target object pose against the end-effector pose and emits a
single boolean "geometric grasp" signal. It is a pure observer:

  * SUBSCRIBES ONLY to /free_target_object/odometry (nav_msgs/Odometry) and to
    tf. It never publishes to /target_pose or any other control topic, so it
    cannot interfere with the running controller.
  * No dependency on franka_cartesian_control or free_target_object.
  * This node produces the GEOMETRIC signal only. Fusion with a force/contact
    criterion (aggregate state machine) is a later, separate step.

Two conditions, each thresholded by a documented ROS parameter:

  a) Position:  dist = || p_ee - p_target ||  <=  epsilon_pos
  b) Approach alignment:  the tool's local approach axis (default -Z of the
     TCP frame), rotated into the world frame by the EE orientation, must point
     towards the target: dot(a_world_hat, (p_target - p_ee)_hat) >= cos(theta),
     with theta = approach_axis_angle_deg (default 30 deg -> cos ~ 0.866).
     Gated by enable_alignment_check (default False): with a rotationally
     symmetric target (cylinder) the approach angle is not discriminative, so
     b) is computed for logging only and does not enter the confirmed signal
     unless the flag is set.

Output:
  * <topic> confirmed_topic  (std_msgs/Bool)  -- True IFF a) holds (and b) too
                             when enable_alignment_check is True).
                             Published every cycle; False whenever data or the
                             tf lookup is missing (conservative gate).
  * <topic> debug_topic  (std_msgs/Float64MultiArray) -- raw values for manual
    threshold tuning: [dist, align_dot, epsilon_pos, cos_threshold,
                       pos_ok, align_ok, data_ok]
  * a throttled INFO log of the same values.

Frames: the target Odometry pose is taken as expressed in `world_frame`
(nav_msgs/Odometry header.frame_id, "world" by default); the EE pose is
obtained with tf2 as world_frame -> ee_frame. Verified offline that the
franka description publishes the full chain world -> fer_link0 -> ... ->
fer_link8 -> fer_hand -> fer_hand_tcp; tf2 composes it in one lookup.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray

import tf2_ros


def _quat_rotate(q_xyzw, v_xyz):
    """Rotate 3-vector v by unit quaternion q = (x, y, z, w). Pure Python."""
    x, y, z, w = q_xyzw
    vx, vy, vz = v_xyz
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w * t + cross(q_xyz, t)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return (rx, ry, rz)


def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class GeometricGraspMonitor(Node):

    def __init__(self):
        super().__init__('geometric_grasp_monitor')

        # -- Parameters -------------------------------------------------------
        # Topic carrying the target object pose (nav_msgs/Odometry, world frame).
        self.target_odom_topic = self.declare_parameter(
            'target_odom_topic', '/free_target_object/odometry').value

        # tf frames. world_frame must match the target Odometry header.frame_id.
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.ee_frame = self.declare_parameter('ee_frame', 'fer_hand_tcp').value

        # Check rate. Non-real-time; 10-20 Hz is plenty.
        self.check_rate_hz = float(
            self.declare_parameter('check_rate_hz', 15.0).value)

        # Position threshold [m]. Default = half the test cube edge
        # (free_target_object default size 0.2 m -> 0.1 m from centre to face)
        # plus a 0.02 m grasp margin. EXPLICIT PARAMETER: update this when the
        # real target geometry changes (do NOT rely on the default).
        self.epsilon_pos = float(
            self.declare_parameter('epsilon_pos', 0.035).value)

        # Approach-axis alignment half-cone [deg]. Default 30 deg (cos ~ 0.866).
        self.approach_axis_angle_deg = float(
            self.declare_parameter('approach_axis_angle_deg', 30.0).value)

        # Tool local approach axis, in the ee_frame. Default -Z of the TCP
        # (which, given the known downward gripper orientation, is ~world -Z).
        # Parameter so the convention is explicit and easy to change.
        self.tool_approach_axis = list(self.declare_parameter(
            'tool_approach_axis', [0.0, 0.0, -1.0]).value)

        # Whether the approach-axis alignment constraint (b) enters the final
        # confirmed signal. Disabled by design (default False): the real target
        # is a cylinder, whose rotational symmetry about its principal axis
        # makes the approach angle non-discriminative for grasp quality. The
        # alignment math (align_dot / align_ok) is still computed for
        # logging/debug; only its use in `confirmed` is gated. Flip to True to
        # reinstate the constraint for a target of a different shape.
        self.enable_alignment_check = bool(self.declare_parameter(
            'enable_alignment_check', False).value)

        # Output topics (private by default).
        confirmed_topic = self.declare_parameter(
            'confirmed_topic', '~/geometric_grasp_confirmed').value
        debug_topic = self.declare_parameter(
            'debug_topic', '~/geometric_grasp_debug').value

        # Period [s] for the low-frequency tuning log.
        self.debug_log_period_s = float(
            self.declare_parameter('debug_log_period_s', 1.0).value)

        # -- Derived --------------------------------------------------------
        n = _norm(self.tool_approach_axis)
        if n < 1e-9:
            self.get_logger().warn(
                'tool_approach_axis has ~zero norm; falling back to (0, 0, -1)')
            self.tool_approach_axis = [0.0, 0.0, -1.0]
            n = 1.0
        self.tool_approach_axis = [c / n for c in self.tool_approach_axis]
        self.cos_threshold = math.cos(math.radians(self.approach_axis_angle_deg))

        # -- I/O ----------------------------------------------------------
        self._last_odom = None  # type: Odometry | None

        self.create_subscription(
            Odometry, self.target_odom_topic, self._on_odom, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_confirmed = self.create_publisher(Bool, confirmed_topic, 10)
        self.pub_debug = self.create_publisher(
            Float64MultiArray, debug_topic, 10)

        period = 1.0 / self.check_rate_hz if self.check_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self._last_log_t = self.get_clock().now()

        self.get_logger().info(
            'geometric_grasp_monitor started\n'
            f'  target_odom_topic        = {self.target_odom_topic}\n'
            f'  world_frame -> ee_frame   = {self.world_frame} -> {self.ee_frame}\n'
            f'  check_rate_hz             = {self.check_rate_hz:.1f}\n'
            f'  epsilon_pos               = {self.epsilon_pos:.4f} m\n'
            f'  approach_axis_angle_deg   = {self.approach_axis_angle_deg:.1f} '
            f'(cos = {self.cos_threshold:.4f})\n'
            f'  tool_approach_axis (unit) = {self.tool_approach_axis}\n'
            f'  enable_alignment_check    = {self.enable_alignment_check}\n'
            f'  confirmed_topic           = {self.pub_confirmed.topic_name}\n'
            f'  debug_topic               = {self.pub_debug.topic_name}')

    # ---------------------------------------------------------------------
    def _on_odom(self, msg):
        self._last_odom = msg

    def _publish(self, confirmed, dist, align_dot, pos_ok, align_ok, data_ok):
        self.pub_confirmed.publish(Bool(data=bool(confirmed)))

        # Keep the 7-element debug layout stable for existing consumers:
        # [dist, align_dot, epsilon_pos, cos_threshold, pos_ok, align_ok, data_ok].
        # When the alignment gate is disabled its three fields (align_dot,
        # cos_threshold, align_ok) are zeroed rather than filled with the real
        # values, so an observer sees at a glance they do not feed `confirmed`.
        if self.enable_alignment_check:
            align_dot_dbg = float(align_dot)
            cos_threshold_dbg = float(self.cos_threshold)
            align_ok_dbg = 1.0 if align_ok else 0.0
        else:
            align_dot_dbg = 0.0
            cos_threshold_dbg = 0.0
            align_ok_dbg = 0.0

        dbg = Float64MultiArray()
        dbg.data = [
            float(dist), align_dot_dbg,
            float(self.epsilon_pos), cos_threshold_dbg,
            1.0 if pos_ok else 0.0,
            align_ok_dbg,
            1.0 if data_ok else 0.0,
        ]
        self.pub_debug.publish(dbg)

        now = self.get_clock().now()
        if (now - self._last_log_t) >= Duration(seconds=self.debug_log_period_s):
            self._last_log_t = now
            if data_ok:
                if self.enable_alignment_check:
                    align_str = (f'align={align_dot:.4f}/{self.cos_threshold:.3f} '
                                 f'(align_ok={align_ok})')
                else:
                    align_str = 'align=disabled'
                self.get_logger().info(
                    f'dist={dist:.4f}/{self.epsilon_pos:.3f} '
                    f'(pos_ok={pos_ok})  '
                    f'{align_str}  -> confirmed={confirmed}')
            else:
                self.get_logger().warn(
                    'geometric grasp NOT evaluated (missing target odom or tf '
                    f'{self.world_frame}->{self.ee_frame}); publishing False')

    def _on_timer(self):
        odom = self._last_odom
        if odom is None:
            self._publish(False, float('nan'), float('nan'),
                          False, False, data_ok=False)
            return

        # Guard: the target position must be expressed in world_frame so it is
        # directly comparable with the tf'd EE position.
        odom_frame = (odom.header.frame_id or '').lstrip('/')
        if odom_frame and odom_frame != self.world_frame.lstrip('/'):
            self.get_logger().warn(
                f"target odometry frame_id '{odom.header.frame_id}' != "
                f"world_frame '{self.world_frame}'; publishing False",
                throttle_duration_sec=5.0)
            self._publish(False, float('nan'), float('nan'),
                          False, False, data_ok=False)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.ee_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as ex:
            self.get_logger().warn(
                f'tf lookup {self.world_frame}->{self.ee_frame} failed: {ex}',
                throttle_duration_sec=5.0)
            self._publish(False, float('nan'), float('nan'),
                          False, False, data_ok=False)
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        p_ee = (t.x, t.y, t.z)
        q_ee = (r.x, r.y, r.z, r.w)

        p = odom.pose.pose.position
        p_target = (p.x, p.y, p.z)

        # (a) Euclidean distance.
        d = (p_target[0] - p_ee[0],
             p_target[1] - p_ee[1],
             p_target[2] - p_ee[2])
        dist = _norm(d)
        pos_ok = dist <= self.epsilon_pos

        # (b) Approach-axis alignment.
        a_world = _quat_rotate(q_ee, self.tool_approach_axis)
        a_n = _norm(a_world)
        if a_n < 1e-9:
            align_dot = -1.0
        elif dist < 1e-6:
            # EE coincides with the target centre: direction undefined, treat
            # as aligned (position condition already dominates).
            align_dot = 1.0
        else:
            a_world = (a_world[0] / a_n, a_world[1] / a_n, a_world[2] / a_n)
            d_hat = (d[0] / dist, d[1] / dist, d[2] / dist)
            align_dot = (a_world[0] * d_hat[0]
                         + a_world[1] * d_hat[1]
                         + a_world[2] * d_hat[2])
        align_ok = align_dot >= self.cos_threshold

        # Alignment gate is optional (see enable_alignment_check): when off, the
        # geometric signal is the position condition alone. align_dot/align_ok
        # stay computed above for the debug topic / log.
        if self.enable_alignment_check:
            confirmed = pos_ok and align_ok
        else:
            confirmed = pos_ok
        self._publish(confirmed, dist, align_dot, pos_ok, align_ok, data_ok=True)


def main(args=None):
    rclpy.init(args=args)
    node = GeometricGraspMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
