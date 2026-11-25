#!/usr/bin/env python3
"""Genera il GraspCommand per l'hardware con grasp_planner_node (senza Gazebo) e lo scrive in YAML "grasp_command_v2".

Gira nel container "franka" (ROS2 Humble, source /tmp/pl_install/setup.bash). Il nodo parte con use_sim_time; questo script
pubblica /clock e 2 campioni di odometria sintetici del satellite, riceve /grasp_command (reliable + transient_local),
spegne il nodo e scrive il YAML. Le funzioni di conversione NON importano ROS (testate in tools/test_make_hw_grasp_command.py).

Uso:  python3 tools/make_hw_grasp_command.py --theta0-deg -10 --omega-deg-s -2 --out real_replay/hw/examples/x.yaml
"""
import argparse
import hashlib
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS = WS / "config/grasp_planner_hardware_j1p90.yaml"
DEFAULT_WEIGHTS_DIR = Path("/root/thesis_ws/runs/20261124T165020_prodmp")
EXPECTED_SHA = "50326451fe00f8ee8264d54105e9a79268d322f3def62215a6b655876c37ba4a"
CENTER = (0.0, 0.72, 0.70)
STATUS_NAMES = {0: "UNSET", 1: "SELECTED", 2: "NO_FEASIBLE_CANDIDATE", 3: "POSTHOC_FAILED"}
Q_START_REF_WXYZ = (0.494585, -0.505353, -0.505353, -0.494593)   # FK(Q_MEAS) attesa, tol 1e-3
Q_MEAS = [1.570787, -0.786572, 0.000002, -2.378914, -0.000006, 3.141609, 0.785398]


# ---- conversione (senza ROS) -----------------------------------------------------------------------------------------
def quat_xyzw_to_wxyz(q):
    """ROS (x, y, z, w) -> hardware (w, x, y, z)."""
    x, y, z, w = q
    return [w, x, y, z]


def quat_wxyz_to_xyzw(q):
    w, x, y, z = q
    return [x, y, z, w]


def status_name(code):
    try:
        return STATUS_NAMES[int(code)]
    except KeyError:
        raise ValueError("status %r fuori da 0..3" % (code,))


def wrap_branch_deg(x_deg, lo=-270.0):
    """Angolo in [lo, lo+360)."""
    y = x_deg - 360.0 * math.floor((x_deg - lo) / 360.0)
    if y >= lo + 360.0:
        y -= 360.0
    return y


def convert_command(m, expected_sha=EXPECTED_SHA, now_s=None):
    """m: dict con i campi di GraspCommand (quaternione goal_quat_xyzw in ordine ROS). Ritorna il dict del YAML v2.
    Rifiuta (ValueError) se weights_sha256 != expected_sha."""
    sha = str(m["weights_sha256"]).lower()
    if expected_sha is not None and sha != expected_sha.lower():
        raise ValueError("weights_sha256 del messaggio %s != atteso %s" % (sha, expected_sha))
    r = lambda v: round(float(v), 9)
    return {
        "format": "grasp_command_v2",
        "status": status_name(m["status"]),
        "frame_id": "world",
        "stamp_unix_s": float(time.time() if now_s is None else now_s),
        "theta_star_deg": r(wrap_branch_deg(math.degrees(float(m["theta_star_rad"])))),
        "contact_time_s": r(m["contact_time_s"]),
        "omega_deg_s": r(math.degrees(float(m["omega_rad_s"]))),
        "weights_sha256": sha,
        "start_pose": {"position_m": [r(v) for v in m["start_position"]],
                       "quaternion_wxyz": [r(v) for v in m["start_quat_wxyz"]]},
        "goal_pose": {"position_m": [r(v) for v in m["goal_position"]],
                      "quaternion_wxyz": [r(v) for v in quat_xyzw_to_wxyz(m["goal_quat_xyzw"])]},
        "k": int(m["k"]),
        "wrist_flip": bool(m["wrist_flip"]),
    }


def dump_yaml(cmd, diag, header=""):
    """YAML a mano: le chiavi di v2 sono solo quelle di convert_command; la diagnostica e' in commenti."""
    f = lambda v: "%.9g" % v if isinstance(v, float) else str(v)
    vec = lambda v: "[" + ", ".join("%.9g" % x for x in v) + "]"
    L = [header.rstrip("\n")] if header else []
    L += ["# Diagnostica (solo commenti, non chiavi):"]
    for k, v in diag.items():
        L.append("#   %s: %s" % (k, v))
    L += ["format: grasp_command_v2", "status: %s" % cmd["status"], "frame_id: %s" % cmd["frame_id"],
          "stamp_unix_s: %.3f" % cmd["stamp_unix_s"], "theta_star_deg: %.9g" % cmd["theta_star_deg"],
          "contact_time_s: %.9g" % cmd["contact_time_s"], "omega_deg_s: %.9g" % cmd["omega_deg_s"],
          'weights_sha256: "%s"' % cmd["weights_sha256"],
          "start_pose: {position_m: %s, quaternion_wxyz: %s}" % (vec(cmd["start_pose"]["position_m"]), vec(cmd["start_pose"]["quaternion_wxyz"])),
          "goal_pose: {position_m: %s, quaternion_wxyz: %s}" % (vec(cmd["goal_pose"]["position_m"]), vec(cmd["goal_pose"]["quaternion_wxyz"])),
          "k: %d" % cmd["k"], "wrist_flip: %s" % ("true" if cmd["wrist_flip"] else "false")]
    return "\n".join(L) + "\n"


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 16), b""):
            h.update(c)
    return h.hexdigest()


# ---- parte ROS -------------------------------------------------------------------------------------------------------
def fk_start_quat_wxyz(urdf_path, q):
    """Orientazione (w,x,y,z) della FK di q sul frame fer_hand_tcp del planner (stesso URDF) via pinocchio-free parser URDF."""
    import numpy as np
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf_path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    child_of = {j.find("child").get("link"): j for j in joints.values()}

    def rpy(r, p, y):
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])

    def axis_angle(a, t):
        a = np.array(a, float); a /= np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K

    chain, link = [], "fer_hand_tcp"
    while link in child_of:
        j = child_of[link]; chain.append(j); link = j.find("parent").get("link")
    R, p = np.eye(3), np.zeros(3)
    for j in reversed(chain):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0, 0, 0]
        rp = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0]
        p = p + R @ np.array(xyz); R = R @ rpy(*rp)
        n = j.get("name")
        if j.get("type") in ("revolute", "continuous") and n.startswith("fer_joint"):
            idx = int(n.replace("fer_joint", "")) - 1
            ax = [float(v) for v in j.find("axis").get("xyz").split()]
            R = R @ axis_angle(ax, q[idx])
    # matrice -> quaternione (w,x,y,z), w >= 0 poi segno coerente col riferimento
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    w = math.sqrt(max(0.0, 1 + tr)) / 2
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2 * (1 if R[2, 1] - R[1, 2] >= 0 else -1)
    y = math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2 * (1 if R[0, 2] - R[2, 0] >= 0 else -1)
    z = math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2 * (1 if R[1, 0] - R[0, 1] >= 0 else -1)
    return [w, x, y, z], p.tolist()


def run_planner(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from satellite_grasp_msgs.msg import GraspCommand

    omega = math.radians(a.omega_deg_s)
    params = [("use_sim_time", "true"), ("satellite_rotation_angular_velocity_deg_s", repr(a.omega_deg_s)),
              ("demo_params_path", str(a.weights_dir / "weights_demo_params.yaml"))]
    cmd = ["ros2", "run", "satellite_grasp_planner", "grasp_planner_node", "--ros-args", "--params-file", str(a.params)]
    for k, v in params:
        cmd += ["-p", "%s:=%s" % (k, v), "-p", "grasp_planner_node:%s:=%s" % (k, v)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    rclpy.init()
    node = Node("make_hw_grasp_command", parameter_overrides=[])
    got = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(GraspCommand, "/grasp_command", lambda m: got.setdefault("msg", m), qos)
    clk = node.create_publisher(Clock, "/clock", 10)
    odo = node.create_publisher(Odometry, "/free_target_object/odometry", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    th0 = math.radians(a.theta0_deg)
    t0 = 100.0   # tempo simulato del primo campione
    samples = [(t0, th0), (t0 + 0.1, th0 + omega * 0.1)]

    def odom(t, th):
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
        m.header.frame_id = "world"; m.child_frame_id = "satellite"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = CENTER
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        m.twist.twist.angular.z = omega
        return m

    def tick(t):
        c = Clock(); c.clock.sec, c.clock.nanosec = int(t), int(round((t - int(t)) * 1e9)); clk.publish(c)

    # il nodo deve vedere /clock e i due campioni; il suo clock simulato avanza solo con /clock
    t_begin = time.time(); t_end = t_begin + a.timeout
    sim = t0 - 1.0
    err = None
    try:
        while (not (clk.get_subscription_count() and odo.get_subscription_count()) or time.time() - t_begin < 3.0) \
                and time.time() < t_end and proc.poll() is None:
            tick(sim); rclpy.spin_once(node, timeout_sec=0.05)
        for t, th in samples:   # campione 1 a t0, campione 2 a t0 + 0.1 (stamp = clock simulato)
            sim = t; tick(sim); rclpy.spin_once(node, timeout_sec=0.05)
            odo.publish(odom(t, th))
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)
            tick(sim)
        while time.time() < t_end and "msg" not in got and proc.poll() is None:
            sim += 0.02; tick(sim); rclpy.spin_once(node, timeout_sec=0.02)
        if "msg" not in got:
            err = "nessun /grasp_command entro %.0f s (nodo %s)" % (a.timeout, "terminato rc=%s" % proc.returncode if proc.poll() is not None else "vivo")
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            log = proc.communicate(timeout=15)[0]
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL); log = proc.communicate()[0]
        node.destroy_node(); rclpy.shutdown()
    if err:
        sys.stderr.write(log[-3000:] + "\n"); raise SystemExit(err)
    return got["msg"], log


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--theta0-deg", type=float, default=-10.0, help="fase iniziale del satellite al primo campione [deg]")
    ap.add_argument("--omega-deg-s", type=float, default=-2.0, help="velocita' di fase [deg/s]")
    ap.add_argument("--out", required=True, help="YAML di uscita")
    ap.add_argument("--weights-dir", type=Path, default=DEFAULT_WEIGHTS_DIR, help="cartella con weights.yaml e weights_demo_params.yaml")
    ap.add_argument("--params", type=Path, default=DEFAULT_PARAMS, help="YAML dei parametri del planner")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--log", type=Path, default=None, help="scrive qui il log del nodo (opzionale)")
    a = ap.parse_args(argv)
    sha = sha256_file(a.weights_dir / "weights.yaml")
    if sha != EXPECTED_SHA:
        raise SystemExit("sha256(weights.yaml) = %s != atteso %s" % (sha, EXPECTED_SHA))
    msg, log = run_planner(a)
    if a.log:
        a.log.write_text(log)
    q_wxyz, p_fk = fk_start_quat_wxyz(str(WS / "fer_flat_effort.urdf"), Q_MEAS)
    d = {"status": msg.status, "theta_star_rad": msg.theta_star_rad, "contact_time_s": msg.contact_time_s,
         "omega_rad_s": msg.omega_rad_s, "weights_sha256": msg.weights_sha256, "k": msg.k, "wrist_flip": msg.wrist_flip,
         "start_position": [msg.start_position.x, msg.start_position.y, msg.start_position.z],
         "start_quat_wxyz": q_wxyz,
         "goal_position": [msg.goal_pose.position.x, msg.goal_pose.position.y, msg.goal_pose.position.z],
         "goal_quat_xyzw": [msg.goal_pose.orientation.x, msg.goal_pose.orientation.y, msg.goal_pose.orientation.z, msg.goal_pose.orientation.w]}
    cmd = convert_command(d)
    dq = max(abs(x - y) for x, y in zip(q_wxyz, Q_START_REF_WXYZ))
    dq_neg = max(abs(x + y) for x, y in zip(q_wxyz, Q_START_REF_WXYZ))
    diag = {"cost": msg.cost, "d_pos_m": msg.d_pos_m, "w_hat": msg.w_hat, "min_joint_margin_rad": msg.min_joint_margin_rad,
            "posthoc_dv_m_s": msg.posthoc_dv_m_s, "evaluations": msg.evaluations, "selection_elapsed_s": msg.selection_elapsed_s,
            "provisional_delay_s": msg.provisional_delay_s, "snapshot_theta_deg": math.degrees(msg.snapshot_theta_rad),
            "start_quat_vs_ref_max_abs": min(dq, dq_neg), "theta0_deg": a.theta0_deg}
    out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(dump_yaml(cmd, diag, "# GraspCommand hardware (grasp_command_v2) generato da tools/make_hw_grasp_command.py\n"
                                       "# params: %s  weights: %s" % (a.params, a.weights_dir)))
    print(out); print("\n".join("%s=%s" % kv for kv in diag.items()))
    print("status=%s k=%d theta*=%.4f deg wrist_flip=%s goal=%s" % (cmd["status"], cmd["k"], cmd["theta_star_deg"], cmd["wrist_flip"], cmd["goal_pose"]["position_m"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
