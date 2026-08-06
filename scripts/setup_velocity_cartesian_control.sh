#!/usr/bin/env bash
# Da lanciare dentro il container franka, dopo ogni ricreazione del container.
set -e

FRANKA_WS=/root/ros_workspaces/ros2/franka_ws
OVERRIDES=/root/thesis_ws/src/franka_gazebo_overrides

echo "[1/4] Ripristino config e launch file..."
cp "$OVERRIDES/franka_gazebo_controllers.yaml" \
   "$FRANKA_WS/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/"
cp "$OVERRIDES/gazebo_velocity_cartesian_control.launch.py" \
   "$FRANKA_WS/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/launch/"

echo "[2/4] Ricreo il link simbolico del pacchetto..."
if [ ! -L "$FRANKA_WS/src/velocity_cartesian_control" ]; then
    ln -s /root/thesis_ws/src/velocity_cartesian_control "$FRANKA_WS/src/velocity_cartesian_control"
fi

echo "[3/4] Build..."
source /opt/ros/humble/setup.bash
cd "$FRANKA_WS"
colcon build --packages-select velocity_cartesian_control franka_gazebo_bringup \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[4/4] Fatto. Per lanciare:"
echo "  source $FRANKA_WS/install/setup.bash"
echo "  ros2 launch franka_gazebo_bringup gazebo_velocity_cartesian_control.launch.py"
