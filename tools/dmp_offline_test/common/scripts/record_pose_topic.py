#!/usr/bin/env python3
"""Records a geometry_msgs/PoseStamped topic to a t,epoch,x,y,z,qw,qx,qy,qz
CSV - a superset of the t,x,y,z,qw,qx,qy,qz schema used everywhere else in
this project (demo_raw.csv, live_demo_raw.csv, replay_from_yaml.csv), so the
output plugs directly into plot_real_demo.py / plot_pose_csvs.py.

Typical use: capture what the robot actually executed in Gazebo, by pointing
this at a Cartesian controller's own feedback topic (e.g.
/cartesian_impedance_controller/actual_pose or .../target_pose_aligned) -
as opposed to /target_pose, which is only the commanded reference.

Three modes:

1. Manual (default): records everything from startup to Ctrl+C. The `t`
   column is relative to whenever the recorder's own subscription first
   received a message, which is NOT necessarily "whenever the motion
   started" - two independent recordings will have different amounts of
   idle padding, making later comparison awkward (see plot_pose_csvs.py's
   --duration option to work around this after the fact).

2. --buttons-trigger <topic> (for a live teleop demo): only records between
   the same button[0]/button[1] rising-edge start/stop events that
   live_demo_recorder_node itself reacts to on /touch0/buttons - so this
   recording is trimmed to the exact same active window with no manual
   timing needed, and the node exits automatically once stopped.

3. --pose-trigger <topic> --duration <seconds>|--duration-from-weights
   <weights.yaml> (for a DMP replay): starts recording on the first message
   received on <topic> (e.g. /target_pose, whose first publish marks the
   start of dmp_gazebo_executor_node's rollout) and stops automatically
   after the given duration - which can be read directly from a saved
   weights YAML's tau instead of having to copy dmp_gazebo_executor_node's
   own "tau: ..." log line by hand. Both ends are timed by this node's own
   wall clock, so no cross-node/cross-clock timestamp correlation is
   involved (relevant since a Gazebo-driven controller's feedback topic
   commonly runs on sim time, not wall-clock time - check by eyeballing
   whether a CSV's `epoch` column looks like a ~1.7-billion Unix timestamp
   or small numbers starting near 0).

Usage:
    python3 record_pose_topic.py <topic> <output_csv>
    python3 record_pose_topic.py <topic> <output_csv> --buttons-trigger <buttons_topic>
    python3 record_pose_topic.py <topic> <output_csv> --pose-trigger <trigger_topic> --duration <seconds>
    python3 record_pose_topic.py <topic> <output_csv> --pose-trigger <trigger_topic> --duration-from-weights <weights.yaml>

Ctrl+C always stops and closes the file cleanly; --buttons-trigger and
--pose-trigger additionally stop (and exit) on their own once done.
"""
import sys

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def tau_from_weights(weights_path: str) -> float:
    with open(weights_path) as f:
        root = yaml.safe_load(f)
    node = root['position_dmp'] if 'position_dmp' in root else root
    return float(node['tau'])


class PoseTopicRecorder(Node):
    def __init__(self, topic: str, out_path: str, buttons_topic: str = None,
                 pose_trigger_topic: str = None, duration: float = None):
        super().__init__('pose_topic_recorder')
        self._f = open(out_path, 'w')
        self._f.write('t,epoch,x,y,z,qw,qx,qy,qz\n')
        self._t0 = None
        self._count = 0
        self.done = False

        # Gate: None means "always active" (manual mode); otherwise only
        # samples received while _active is True get written.
        self._active = (buttons_topic is None and pose_trigger_topic is None)
        self._prev_buttons = None
        self._duration = duration
        self._stop_timer = None

        self._sub = self.create_subscription(PoseStamped, topic, self._pose_cb, qos_profile_sensor_data)

        if buttons_topic:
            self._buttons_sub = self.create_subscription(
                Joy, buttons_topic, self._buttons_cb, 10)
            self.get_logger().info(
                f'Recording {topic} -> {out_path}, gated by button[0]/[1] rising edges on {buttons_topic}')
        elif pose_trigger_topic:
            self._trigger_sub = self.create_subscription(
                PoseStamped, pose_trigger_topic, self._trigger_cb, qos_profile_sensor_data)
            self.get_logger().info(
                f'Recording {topic} -> {out_path}, starting on first message on '
                f'{pose_trigger_topic}, stopping after {self._duration:.3f}s')
        else:
            self.get_logger().info(f'Recording {topic} -> {out_path} (Ctrl+C to stop)')

    def _pose_cb(self, msg: PoseStamped) -> None:
        if not self._active:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp
        t = stamp - self._t0
        p, o = msg.pose.position, msg.pose.orientation
        self._f.write(f'{t},{stamp},{p.x},{p.y},{p.z},{o.w},{o.x},{o.y},{o.z}\n')
        self._count += 1

    def _buttons_cb(self, msg: Joy) -> None:
        if len(msg.buttons) < 2:
            return
        if self._prev_buttons is None:
            self._prev_buttons = list(msg.buttons)
            return
        rising0 = msg.buttons[0] != 0 and self._prev_buttons[0] == 0
        rising1 = msg.buttons[1] != 0 and self._prev_buttons[1] == 0
        self._prev_buttons = list(msg.buttons)

        if rising0 and not self._active:
            self._active = True
            self.get_logger().info('Start edge detected - recording.')
        elif rising1 and self._active:
            self._active = False
            self.get_logger().info('Stop edge detected - done.')
            self.done = True

    def _trigger_cb(self, msg: PoseStamped) -> None:
        if self._active or self.done:
            return
        self._active = True
        self.get_logger().info(f'Trigger message received - recording for {self._duration:.3f}s.')
        self._stop_timer = self.create_timer(self._duration, self._stop_after_duration)

    def _stop_after_duration(self) -> None:
        self._active = False
        self.done = True
        if self._stop_timer:
            self._stop_timer.cancel()
        self.get_logger().info('Duration elapsed - done.')

    def close(self) -> None:
        self._f.flush()
        self._f.close()
        # print(), not get_logger(): by the time we get here, rclpy's own
        # SIGINT handler has often already torn down the context, so logging
        # via rosout would just print a harmless "Failed to publish log
        # message to rosout" warning.
        print(f'Saved {self._count} samples.')


def _parse_args(argv):
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    topic, out_path = argv[0], argv[1]
    rest = argv[2:]

    buttons_topic = None
    pose_trigger_topic = None
    duration = None

    if '--buttons-trigger' in rest:
        buttons_topic = rest[rest.index('--buttons-trigger') + 1]
    if '--pose-trigger' in rest:
        pose_trigger_topic = rest[rest.index('--pose-trigger') + 1]
        if '--duration' in rest:
            duration = float(rest[rest.index('--duration') + 1])
        elif '--duration-from-weights' in rest:
            duration = tau_from_weights(rest[rest.index('--duration-from-weights') + 1])
        else:
            print('--pose-trigger requires --duration <seconds> or --duration-from-weights <weights.yaml>')
            sys.exit(1)

    return topic, out_path, buttons_topic, pose_trigger_topic, duration


def main():
    topic, out_path, buttons_topic, pose_trigger_topic, duration = _parse_args(sys.argv[1:])

    rclpy.init()
    node = PoseTopicRecorder(topic, out_path, buttons_topic, pose_trigger_topic, duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        # A trigger mode's own done-condition exits the loop normally, but
        # Ctrl+C is often already handled by rclpy's own SIGINT handler by
        # the time we get here, which shuts the context down itself -
        # calling shutdown() again on an already-shut-down context raises
        # RCLError, so guard it.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
