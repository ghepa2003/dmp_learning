"""Test puri di tools/s1_env.py (riordino dei path)."""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "tools"))
import s1_env  # noqa: E402

R = "/root/thesis_ws/install"


def test_thesis_first_relative_order_kept():
    p = f"/opt/ros/humble:/root/ros_workspaces/ros2/franka_ws/install/a:{R}/haptic_dmp_learning:{R}/free_target_object"
    assert s1_env.reorder_path(p, R) == (
        f"{R}/haptic_dmp_learning:{R}/free_target_object:/opt/ros/humble:/root/ros_workspaces/ros2/franka_ws/install/a")


def test_already_ordered_dedup_and_empty():
    assert s1_env.reorder_path(f"{R}/a::{R}/a:/x:/x", R) == f"{R}/a:/x"


def test_root_itself_and_prefix_lookalike():
    assert s1_env.reorder_path(f"/x:{R}:{R}_old/a", R) == f"{R}:/x:{R}_old/a"


def test_empty():
    assert s1_env.reorder_path("", R) == ""


def test_ld_library_path():
    ament = f"{R}/p1:/opt/ros/humble"
    ld = "/opt/ros/humble/lib:/root/ros_workspaces/ros2/franka_ws/install/p1/lib"
    assert s1_env.ld_library_path(ld, ament, R) == f"{R}/p1/lib:/opt/ros/humble/lib:/root/ros_workspaces/ros2/franka_ws/install/p1/lib"


def test_ld_library_path_existing_thesis_entry_goes_first_once():
    ament = f"{R}/p1"
    ld = f"/lib/x:{R}/p1/lib"
    assert s1_env.ld_library_path(ld, ament, R) == f"{R}/p1/lib:/lib/x"


def test_exports_quoting():
    out = s1_env.exports({"AMENT_PREFIX_PATH": f"/a:{R}/p", "LD_LIBRARY_PATH": ""}, R)
    assert out.splitlines() == [f"export AMENT_PREFIX_PATH={R}/p:/a",
                                f"export LD_LIBRARY_PATH={R}/p/lib"]
