"""
Launch file for the Testing Plane simulation asset in Gazebo Sim.

Orchestrates:
1. Processing `testing_plane.xacro` into URDF for the 6-DOF moving obstacle / surface.
2. Publishing robot state with `robot_state_publisher` under `/testing_plane` namespace.
3. Spawning the testing plane model into the active Gazebo simulator.
4. Activating ros2_control controller manager, `joint_state_broadcaster`, and `testing_plane_position_controller`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Locate and process xacro description
    testing_plane_xacro_file = PathJoinSubstitution([
        FindPackageShare("testing_plane"),
        "xacro",
        "testing_plane.xacro"
    ])
    testing_plane_xacro_urdf = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            testing_plane_xacro_file
        ]),
        value_type=str
    )

    ns = "testing_plane"

    # 2. Position launch arguments
    testing_plane_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial X position for the testing plane in Gazebo")
    testing_plane_y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Initial Y position for the testing plane in Gazebo")
    testing_plane_z_arg = DeclareLaunchArgument(
        "z", default_value="0.0", description="Initial Z position for the testing plane in Gazebo")

    # 3. State publisher for the testing plane kinematics
    testing_plane_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": testing_plane_xacro_urdf}],
        remappings=[
            ("robot_description", "testing_plane_description")
        ]
    )

    # 4. Spawn entity in Gazebo Sim
    spawn_testing_plane_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=ns,
        output="screen",
        arguments=[
            "-name", ns,
            "-topic", f"/{ns}/testing_plane_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z")
        ]
    )

    # 5. Load joint state broadcaster for testing plane joints
    load_box_joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster',
            '--controller-manager', f"/{ns}/controller_manager"
        ],
        output='screen'
    )

    # 6. Load position controller with small delay to ensure controller manager is ready
    delayed_position_controller = TimerAction(
        period=1.0,  # seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'testing_plane_position_controller',
                    '--controller-manager', f"/{ns}/controller_manager"
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        testing_plane_x_arg,
        testing_plane_y_arg,
        testing_plane_z_arg,
        testing_plane_description_publisher,
        spawn_testing_plane_cmd,
        load_box_joint_state_controller,
        delayed_position_controller,
    ])