#!/usr/bin/env bash
# Unico entry point per il provisioning del container franka dopo ogni
# ricreazione: ripristina gli override di config/launch nel bringup vendored,
# ricrea i symlink dei pacchetti di tesi, builda, e verifica che il plugin
# controller sia scopribile.
#
# Uso:
#   full_reset_franka_container.sh              # perimetro completo (default):
#                                               #   franka_cartesian_control
#                                               #   franka_gazebo_bringup
#                                               #   haptic_dmp_learning
#                                               #   free_target_object
#                                               #   grasp_monitoring
#   full_reset_franka_container.sh --no-haptic  # perimetro ridotto: salta
#                                               #   symlink e build di
#                                               #   haptic_dmp_learning,
#                                               #   free_target_object,
#                                               #   grasp_monitoring
set -euo pipefail

WITH_HAPTIC=1
for arg in "$@"; do
    case "$arg" in
        --no-haptic)
            WITH_HAPTIC=0
            ;;
        -h|--help)
            echo "Uso: $0 [--no-haptic]"
            echo "  (nessun flag)  Perimetro completo: franka_cartesian_control,"
            echo "                 franka_gazebo_bringup, haptic_dmp_learning,"
            echo "                 free_target_object, grasp_monitoring."
            echo "  --no-haptic    Salta symlink e build di haptic_dmp_learning,"
            echo "                 free_target_object, grasp_monitoring"
            echo "                 (solo franka_cartesian_control + bringup)."
            exit 0
            ;;
        *)
            echo "[ERRORE] Argomento non riconosciuto: $arg"
            echo "Uso: $0 [--no-haptic]"
            exit 1
            ;;
    esac
done

THESIS_WS="${THESIS_WS:-/root/thesis_ws}"
FRANKA_WS="${FRANKA_WS:-/root/ros_workspaces/ros2/franka_ws}"
OVERRIDES="${OVERRIDES:-$THESIS_WS/src/franka_gazebo_overrides}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ ! -d "$FRANKA_WS" ]; then
    echo "[ERRORE] Workspace franka_ws non trovato in $FRANKA_WS"
    exit 1
fi

BRINGUP_DIR="$FRANKA_WS/src/franka_ros2/franka_gazebo/franka_gazebo_bringup"
if [ ! -d "$BRINGUP_DIR" ]; then
    echo "[ERRORE] franka_gazebo_bringup non trovato in $BRINGUP_DIR"
    exit 1
fi

# Perimetro di build (haptic_dmp_learning solo se non --no-haptic).
BUILD_PACKAGES=(franka_cartesian_control franka_gazebo_bringup)
if [ "$WITH_HAPTIC" -eq 1 ]; then
    BUILD_PACKAGES+=(haptic_dmp_learning free_target_object grasp_monitoring)
    echo "=== Perimetro: completo (franka_cartesian_control + bringup + haptic_dmp_learning + free_target_object + grasp_monitoring) ==="
else
    echo "=== Perimetro: ridotto --no-haptic (franka_cartesian_control + bringup) ==="
fi

echo "=== [1/6] Ripristino file di config/launch dal backup ==="
mkdir -p "$BRINGUP_DIR/config" "$BRINGUP_DIR/launch"
cp "$OVERRIDES/franka_gazebo_controllers.yaml" "$BRINGUP_DIR/config/franka_gazebo_controllers.yaml"
cp "$OVERRIDES/gazebo_velocity_cartesian_control.launch.py" "$BRINGUP_DIR/launch/"
cp "$OVERRIDES/gazebo_cartesian_impedance_control.launch.py" "$BRINGUP_DIR/launch/"
cp "$OVERRIDES/gazebo_cartesian_impedance_control_headless.launch.py" "$BRINGUP_DIR/launch/"

echo "=== [2/6] Ricreo i link simbolici dei pacchetti ==="
mkdir -p "$FRANKA_WS/src"
[ -L "$FRANKA_WS/src/franka_cartesian_control" ] || \
    ln -s "$THESIS_WS/src/franka_cartesian_control" "$FRANKA_WS/src/franka_cartesian_control"
if [ "$WITH_HAPTIC" -eq 1 ]; then
    [ -L "$FRANKA_WS/src/haptic_dmp_learning" ] || \
        ln -s "$THESIS_WS/src/haptic_dmp_learning" "$FRANKA_WS/src/haptic_dmp_learning"
    [ -L "$FRANKA_WS/src/free_target_object" ] || \
        ln -s "$THESIS_WS/src/free_target_object" "$FRANKA_WS/src/free_target_object"
    [ -L "$FRANKA_WS/src/grasp_monitoring" ] || \
        ln -s "$THESIS_WS/src/grasp_monitoring" "$FRANKA_WS/src/grasp_monitoring"
else
    echo "  (--no-haptic: symlink haptic_dmp_learning, free_target_object, grasp_monitoring saltati)"
fi

echo "=== [3/6] Verifico che il plugin XML contenga entrambe le classi ==="
if ! grep -q "CartesianImpedanceController" "$THESIS_WS/src/franka_cartesian_control/controllers_plugin.xml"; then
    echo "  [ERRORE] controllers_plugin.xml non contiene CartesianImpedanceController!"
    echo "  Controlla manualmente: $THESIS_WS/src/franka_cartesian_control/controllers_plugin.xml"
    exit 1
fi
echo "  OK: entrambe le classi presenti nel sorgente"

echo "=== [4/6] Build completo ==="
set +u
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
cd "$FRANKA_WS"
colcon build --packages-select "${BUILD_PACKAGES[@]}" \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
set -u

echo "=== [5/6] Verifico che il plugin sia scopribile dall'indice installato ==="
if ! grep -q "CartesianImpedanceController" \
    "$FRANKA_WS/install/franka_cartesian_control/share/franka_cartesian_control/controllers_plugin.xml"; then
    echo "  [ERRORE] Il plugin non risulta installato correttamente!"
    exit 1
fi
echo "  OK: plugin installato e scopribile"

echo "=== [6/6] Verifico che i pacchetti siano risolvibili ==="
ros2 pkg prefix franka_cartesian_control > /dev/null && echo "  OK: franka_cartesian_control"
if [ "$WITH_HAPTIC" -eq 1 ]; then
    ros2 pkg prefix haptic_dmp_learning > /dev/null && echo "  OK: haptic_dmp_learning"
    ros2 pkg prefix free_target_object > /dev/null && echo "  OK: free_target_object"
    ros2 pkg prefix grasp_monitoring > /dev/null && echo "  OK: grasp_monitoring"
fi

echo ""
echo "=== FATTO. Ambiente pronto. ==="
echo "Per modificare i guadagni del controller: edita direttamente"
echo "  $BRINGUP_DIR/config/franka_gazebo_controllers.yaml"
echo "(e' un symlink verso install/, NON serve rebuild dopo averlo modificato)."
echo ""
echo "Per lanciare:"
echo "  ros2 launch franka_gazebo_bringup gazebo_cartesian_impedance_control_headless.launch.py"
