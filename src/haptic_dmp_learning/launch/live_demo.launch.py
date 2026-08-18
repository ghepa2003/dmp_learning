# Live dry-run demo pipeline: visualize a demonstration in Gazebo before it
# is learned, then auto-train the DMP (ridge + velocity filter) as soon as
# the demo ends.
#
# This launch file only starts the two haptic_dmp_learning nodes below. It
# does NOT bring up Gazebo or velocity_cartesian_controller: those run in a
# separate container/workspace (franka_ws -> franka_gazebo_bringup, see
# scripts/setup_velocity_cartesian_control.sh) and are assumed to already be
# active - "ros2 launch franka_gazebo_bringup gazebo_velocity_cartesian_control.launch.py".
#
# use_csv_playback:=true (default): csv_master_pose_player_node stands in for
#   the Geomagic Touch device, replaying a recorded demo CSV onto
#   /master_pose_raw with a zero-order hold, and emitting the equivalent
#   /touch0/buttons start/stop events.
# use_csv_playback:=false: only live_demo_recorder_node is started. Launch the
#   real Geomagic Touch driver separately, remapping its native PoseStamped
#   topic onto /master_pose_raw (-r <driver_topic>:=/master_pose_raw) - the
#   driver already publishes its own /touch0/buttons, so no other change is
#   needed.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('haptic_dmp_learning'), 'config', 'params.yaml')

    use_csv_playback = LaunchConfiguration('use_csv_playback')
    params_file = LaunchConfiguration('params_file')

    use_csv_playback_arg = DeclareLaunchArgument(
        'use_csv_playback',
        default_value='true',
        description='Launch csv_master_pose_player_node as a stand-in for the '
                     'Geomagic Touch driver. Set to false once the real driver '
                     'is launched separately with the /master_pose_raw remap.')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='YAML file with ros__parameters for both nodes below.')

    live_demo_recorder_node = Node(
        package='haptic_dmp_learning',
        executable='live_demo_recorder_node',
        name='live_demo_recorder_node',
        output='screen',
        parameters=[params_file],
    )

    csv_master_pose_player_node = Node(
        package='haptic_dmp_learning',
        executable='csv_master_pose_player_node',
        name='csv_master_pose_player_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(use_csv_playback),
    )

    return LaunchDescription([
        use_csv_playback_arg,
        params_file_arg,
        live_demo_recorder_node,
        csv_master_pose_player_node,
    ])
