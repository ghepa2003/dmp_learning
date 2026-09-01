#!/usr/bin/env bash
# Da lanciare dentro il container franka, dopo ogni ricreazione del container
# o dopo un aggiornamento upstream di franka_ros2/franka_description.
set -euo pipefail

THESIS_WS="${THESIS_WS:-/root/thesis_ws}"
FRANKA_WS="${FRANKA_WS:-/root/ros_workspaces/ros2/franka_ws}"
OVERRIDES="${OVERRIDES:-$THESIS_WS/src/franka_description_overrides}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ ! -d "$FRANKA_WS" ]; then
    echo "[ERRORE] Workspace franka_ws non trovato in $FRANKA_WS"
    exit 1
fi

DESC_COMMON_DIR="$FRANKA_WS/src/franka_ros2/franka_description/end_effectors/common"
if [ ! -d "$DESC_COMMON_DIR" ]; then
    echo "[ERRORE] franka_description/end_effectors/common non trovato in $DESC_COMMON_DIR"
    exit 1
fi

DESC_ROBOTS_COMMON_DIR="$FRANKA_WS/src/franka_ros2/franka_description/robots/common"
if [ ! -d "$DESC_ROBOTS_COMMON_DIR" ]; then
    echo "[ERRORE] franka_description/robots/common non trovato in $DESC_ROBOTS_COMMON_DIR"
    exit 1
fi

SRC_XACRO="$OVERRIDES/franka_hand.xacro"
if [ ! -f "$SRC_XACRO" ]; then
    echo "[ERRORE] Override non trovato: $SRC_XACRO"
    exit 1
fi

SRC_ARM_ROS2_CONTROL_XACRO="$OVERRIDES/franka_arm.ros2_control.xacro"
if [ ! -f "$SRC_ARM_ROS2_CONTROL_XACRO" ]; then
    echo "[ERRORE] Override non trovato: $SRC_ARM_ROS2_CONTROL_XACRO"
    exit 1
fi

echo "[1/3] Copio gli override xacro (franka_hand.xacro, franka_arm.ros2_control.xacro)..."
cp "$SRC_XACRO" "$DESC_COMMON_DIR/franka_hand.xacro"
cp "$SRC_ARM_ROS2_CONTROL_XACRO" "$DESC_ROBOTS_COMMON_DIR/franka_arm.ros2_control.xacro"

echo "[2/3] Build..."
set +u
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
cd "$FRANKA_WS"
colcon build --packages-select franka_description \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
set -u

echo "[3/3] Fatto."
echo
echo "  PROMEMORIA: rilancia il bringup con load_gripper:=true per vedere l'effetto."
echo "  Senza quel flag la mano non viene caricata affatto, indipendentemente da"
echo "  questa modifica. Esempio:"
echo "    source $FRANKA_WS/install/setup.bash"
echo "    ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control.launch.py load_gripper:=true"
