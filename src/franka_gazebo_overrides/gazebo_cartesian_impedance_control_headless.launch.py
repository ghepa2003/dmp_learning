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
Headless launcher for Franka Cartesian Impedance Control in Gazebo Sim.
Invokes `gazebo_cartesian_impedance_control.launch.py` with `headless=true` (disabling RViz2 GUI).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_franka_gazebo = get_package_share_directory('franka_gazebo_bringup')

    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_id',
            default_value='fer',
            description='Available values: fr3, fp3 and fer'),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='true/false for activating the gripper'),
        DeclareLaunchArgument(
            'franka_hand',
            default_value='franka_hand',
            description='Default value: franka_hand'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_franka_gazebo, 'launch', 'gazebo_cartesian_impedance_control.launch.py')
            ),
            launch_arguments={
                'arm_id': arm_id,
                'load_gripper': load_gripper,
                'franka_hand': franka_hand,
                'headless': 'true',
            }.items(),
        ),
    ])