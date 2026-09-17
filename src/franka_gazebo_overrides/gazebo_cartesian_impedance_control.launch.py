# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch file for Franka Emika / FER Cartesian Impedance Control in Gazebo Sim.

Orchestrates:
1. Robot description rendering from xacro with `gazebo_effort=true` for torque/effort control.
2. Launching Gazebo simulator world (gz_sim / ros_gz_sim).
3. Spawning the robot model in Gazebo.
4. Starting `robot_state_publisher` and `joint_state_publisher`.
5. Loading and activating controllers in sequence:
   - `joint_state_broadcaster` (after spawn exits)
   - `cartesian_impedance_controller` (after state broadcaster activates)
6. Optional RViz2 visualization (disabled if `headless=true`).
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    """
    Renders the Franka Xacro robot description into XML URDF with effort interface enabled.
    """
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    # Process xacro with effort interfaces enabled for torque impedance control
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]


def prepare_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    headless_name = 'headless'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    headless = LaunchConfiguration(headless_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name,
        default_value='fer',
        description='Available values: fr3, fp3 and fer')
    headless_launch_argument = DeclareLaunchArgument(
        headless_name,
        default_value='false',
        description='Run headless without RViz (true/false)')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim (headless server with -s)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-s -r empty.sdf'}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Bridge Gazebo's /clock to ROS 2. The stock Franka Gazebo bringup does NOT
    # do this: neither gz_sim.launch.py nor gz_ros2_control exposes a /clock
    # bridge in this configuration, so every ROS-side node started with
    # use_sim_time:=true (controller_manager, the controllers, and downstream
    # haptic_dmp_learning nodes) gets a clock that never advances. Confirmed only
    # at runtime (`ros2 topic info /clock --verbose` -> Publisher count: 0).
    # gz-side topic is the plain /clock (Ignition Clock system, global under
    # `-r`); there is no per-world /world/<name>/clock reference anywhere in this
    # file. See franka_cartesian_control/DESIGN_NOTES.md.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ROS2 -> Ignition bridge for gripper position command (workaround for a bug
    # in franka_ign_ros2_control where joint_position_cmd never updates for
    # fer_finger_joint1 via the standard ros2_control chain; diagnosed via
    # RCLCPP_INFO_THROTTLE instrumentation, see thesis notes). Direction is
    # ROS2 -> Gazebo only (']'), opposite of clock_bridge above.
    gripper_position_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gripper_position_bridge',
        output='screen',
        arguments=[
            '/gripper_position_cmd@std_msgs/msg/Float64]ignition.msgs.Double'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Visualize in RViz (only if not headless)
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file, '-f', 'world'],
        condition=UnlessCondition(headless),
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_cartesian_impedance_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_impedance_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Gripper: NON piu' via ros2_control/gripper_controller (ForwardCommandController).
    # Bug non risolto in franka_ign_ros2_control: joint_position_cmd non si
    # aggiorna mai per fer_finger_joint1 attraverso questa catena, nonostante
    # interfaccia dichiarata/esportata/reclamata correttamente (diagnosi via
    # RCLCPP_INFO_THROTTLE instrumentation, vedi thesis notes). Workaround:
    # ignition::gazebo::systems::JointPositionController (plugin nativo, vedi
    # franka_arm.ros2_control.xacro) + gripper_position_bridge sopra.

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        headless_launch_argument,
        gazebo_empty_world,
        clock_bridge,
        gripper_position_bridge,
        robot_state_publisher,
        rviz,
        spawn,
        load_joint_state_broadcaster,
        load_cartesian_impedance_controller,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30,
                 'use_sim_time': True}],
        ),
    ])


def generate_launch_description():
    launch_description = prepare_launch_description()

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('franka_description')))

    launch_description.add_action(set_env_vars_resources)

    return launch_description