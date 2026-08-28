"""
Spawn a single free-floating rigid target object into an ALREADY-RUNNING
Gazebo (Ignition Fortress) simulation.

What it does:
  1. Expands models/free_target_object.sdf.xacro with the requested
     mass / size / inertia / initial-twist arguments into a temporary .sdf.
  2. Spawns it via `ros_gz_sim create` at the given pose
     (x, y, z, roll, pitch, yaw).
  3. Starts a `ros_gz_bridge parameter_bridge` (same launch, no separate
     command) that bridges the model's OdometryPublisher output
     /model/<name>/odometry (ignition.msgs.Odometry) to nav_msgs/msg/Odometry
     on /<name>/odometry: sim-time-stamped world pose (frame world -> <name>)
     PLUS the body's linear/angular twist.
     (`world` is still resolved from /gazebo/worlds when not given, but only
     for `ros_gz_sim create -world`; the odometry bridge needs no world name.)
  4. Initial twist:
       - wx/wy/wz (rad/s) and vx/vy/vz (m/s), all default 0.0.
       - If they are ALL zero  -> the only plugin on the model is the
         observation-only OdometryPublisher: the body is completely free
         (F = m a, I omega_dot = tau). This is the config for VIM validation
         ("body at rest").
       - If ANY is non-zero -> the SDF also gains a VelocityControl plugin
         seeded with <initial_linear>/<initial_angular>. VelocityControl ships
         with Fortress (loads with no extra setup) but HOLDS the commanded
         twist (see the note in the xacro). Use it for the "rotating target"
         case.

This launch does not start Gazebo and never touches testing_plane.
"""

import os
import re
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _resolve_world_name():
    """Ask the running Gazebo server for its world name.

    Mirrors what `ros_gz_sim create` does ("Requesting list of world names"):
    calls the /gazebo/worlds service (ignition.msgs.Empty -> StringMsg_V) and
    returns the first world. Used only when the `world` launch argument is
    left empty; requires Gazebo to be already running (as does the rest of
    this launch).
    """
    try:
        completed = subprocess.run(
            ["ign", "service", "-s", "/gazebo/worlds",
             "--reqtype", "ignition.msgs.Empty",
             "--reptype", "ignition.msgs.StringMsg_V",
             "--timeout", "5000", "--req", ""],
            capture_output=True, text=True, timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        raise RuntimeError(
            "free_target_object: failed to call /gazebo/worlds to resolve the "
            "world name. Pass world:=<name> explicitly, or start Gazebo first. "
            f"({exc})"
        )
    match = re.search(r'data:\s*"([^"]+)"', completed.stdout)
    if not match:
        raise RuntimeError(
            "free_target_object: /gazebo/worlds returned no world name "
            f"(stdout={completed.stdout!r} stderr={completed.stderr!r}). "
            "Pass world:=<name> explicitly."
        )
    return match.group(1)


_ARG_DEFAULTS = {
    "name": ("free_target_object", "Model name in Gazebo"),
    "world": ("", "Target Gazebo world name (empty => resolved from /gazebo/worlds)"),
    "x": ("0.0", "Spawn X in the world frame [m]"),
    "y": ("0.0", "Spawn Y in the world frame [m]"),
    "z": ("0.0", "Spawn Z in the world frame [m]"),
    "roll": ("0.0", "Spawn roll [rad]"),
    "pitch": ("0.0", "Spawn pitch [rad]"),
    "yaw": ("0.0", "Spawn yaw [rad]"),
    "mass": ("5.0", "Link mass [kg]"),
    "size_x": ("0.2", "Box edge length along X [m]"),
    "size_y": ("0.2", "Box edge length along Y [m]"),
    "size_z": ("0.2", "Box edge length along Z [m]"),
    "ixx": ("0.0", "Principal inertia Ixx [kg m^2]; 0 => homogeneous-box formula"),
    "iyy": ("0.0", "Principal inertia Iyy [kg m^2]; 0 => homogeneous-box formula"),
    "izz": ("0.0", "Principal inertia Izz [kg m^2]; 0 => homogeneous-box formula"),
    "wx": ("0.0", "Initial angular velocity about world X [rad/s]"),
    "wy": ("0.0", "Initial angular velocity about world Y [rad/s]"),
    "wz": ("0.0", "Initial angular velocity about world Z [rad/s]"),
    "vx": ("0.0", "Initial linear velocity along world X [m/s]"),
    "vy": ("0.0", "Initial linear velocity along world Y [m/s]"),
    "vz": ("0.0", "Initial linear velocity along world Z [m/s]"),
}


def _spawn(context, *_args, **_kwargs):
    def cfg(key):
        return LaunchConfiguration(key).perform(context)

    name = cfg("name")
    wx, wy, wz = cfg("wx"), cfg("wy"), cfg("wz")
    vx, vy, vz = cfg("vx"), cfg("vy"), cfg("vz")

    twist_vals = [float(v) for v in (wx, wy, wz, vx, vy, vz)]
    use_velocity_control = any(abs(v) > 0.0 for v in twist_vals)

    xacro_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "models",
        "free_target_object.sdf.xacro",
    )
    # Fall back to the installed share path when run from the install space.
    if not os.path.exists(xacro_path):
        from ament_index_python.packages import get_package_share_directory
        xacro_path = os.path.join(
            get_package_share_directory("free_target_object"),
            "models",
            "free_target_object.sdf.xacro",
        )

    mappings = {
        "name": name,
        "mass": cfg("mass"),
        "size_x": cfg("size_x"),
        "size_y": cfg("size_y"),
        "size_z": cfg("size_z"),
        "ixx": cfg("ixx"),
        "iyy": cfg("iyy"),
        "izz": cfg("izz"),
        "use_velocity_control": "true" if use_velocity_control else "false",
        "initial_linear": f"{vx} {vy} {vz}",
        "initial_angular": f"{wx} {wy} {wz}",
    }

    sdf_xml = xacro.process_file(xacro_path, mappings=mappings).toxml()

    tmp = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"{name}_", suffix=".sdf", delete=False
    )
    tmp.write(sdf_xml)
    tmp.close()

    # Resolve the world name (needed only for `ros_gz_sim create -world`).
    world = cfg("world") or _resolve_world_name()

    create_args = ["-file", tmp.name, "-name", name, "-world", world]
    create_args += [
        "-x", cfg("x"), "-y", cfg("y"), "-z", cfg("z"),
        "-R", cfg("roll"), "-P", cfg("pitch"), "-Y", cfg("yaw"),
    ]

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=create_args,
    )

    # Gazebo -> ROS 2 odometry bridge.
    # The model's OdometryPublisher plugin publishes ignition.msgs.Odometry on
    # /model/<name>/odometry with a real sim-time stamp and proper frame ids
    # (world -> <name>) plus the body's linear/angular twist. Bridge it 1:1 to
    # nav_msgs/msg/Odometry and remap onto /<name>/odometry.
    #
    # This replaces the earlier dynamic_pose/info -> TFMessage bridge, which
    # produced zero-stamp / empty-frame_id transforms: SceneBroadcaster fills
    # only the Pose_V container header, and ros_gz_bridge's Pose_V->TFMessage
    # conversion reads only the (empty) per-entity Pose headers.
    gz_odom_topic = f"/model/{name}/odometry"
    ros_odom_topic = f"/{name}/odometry"
    odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{name}_odometry_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[f"{gz_odom_topic}@nav_msgs/msg/Odometry[ignition.msgs.Odometry"],
        remappings=[(gz_odom_topic, ros_odom_topic)],
    )

    twist_msg = (
        f"initial twist: linear=({vx}, {vy}, {vz}) m/s, "
        f"angular=({wx}, {wy}, {wz}) rad/s -> "
        + ("VelocityControl plugin ENABLED (twist is held constant)"
           if use_velocity_control
           else "no plugin: fully free body")
    )

    return [
        LogInfo(msg=f"[free_target_object] world: {world}"),
        LogInfo(msg=f"[free_target_object] expanded SDF: {tmp.name}"),
        LogInfo(msg=f"[free_target_object] {twist_msg}"),
        LogInfo(msg=f"[free_target_object] odometry bridge: {gz_odom_topic} "
                    f"(ignition.msgs.Odometry) -> {ros_odom_topic} "
                    f"(nav_msgs/msg/Odometry); frames world -> {name}"),
        spawn,
        odom_bridge,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(key, default_value=default, description=desc)
        for key, (default, desc) in _ARG_DEFAULTS.items()
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_spawn)])
