from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable


def generate_launch_description():

    testing_plane_xacro_file = PathJoinSubstitution([
        FindPackageShare("testing_plane"),
        "xacro",
        "testing_plane.xacro"
    ])
    testing_plane_xacro_urdf = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        testing_plane_xacro_file
    ])
    
    ns = "testing_plane"

    testing_plane_x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position for the testing plane")
    testing_plane_y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position for the testing plane")
    testing_plane_z_arg = DeclareLaunchArgument("z", default_value="0.0", description="Z position for the testing plane")
        
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": testing_plane_xacro_urdf,
            "use_sim_time": True
        }]
    )

    load_box_joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster',
            '--controller-manager', f"/{ns}/controller_manager"
        ],
        output='screen'
    )

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