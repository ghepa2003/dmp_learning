#!/usr/bin/env python3
"""
tools/s1_run.py - UN run S1 (bringup Gazebo, spawn cubo rotante, bag, executor, planner) + s1_report.

Estratto e parametrizzato da analysis_s1_1015/run_final2.py (stessi comandi di bringup, spawn, bag,
executor e planner). Da lanciare nel container con ROS (serve rclpy per /clock e /grasp_command).

    python3 tools/s1_run.py --weights-dir runs/<run> --depth 0.0125 --kt 2000 --tag prova1

Esce con 0 se il run e' completo e il report e' stato scritto (il successo della presa e' in
report.json["success"], non nel codice di uscita); !=0 con UNA riga "S1_RUN FAIL: <motivo>" su stderr.
  codice 1: run fallito; 3: run/report ok ma restano processi (il run successivo NON e' sicuro).

--stop-cube: dopo "follow: ended" nel log dell'executor pubblica 3 volte uno Twist nullo sul topic
cmd_vel del cubo (VelocityControl, trovato con `ign topic -l`, nessun bridge) e 1 s simulato dopo legge
wz dall'odometria (run_meta.json: stop_cube_topic, stop_cube_sim_t, wz_after_stop_degps).

Output in --outdir (default s1_runs/<tag>): bag/, *.log, run_meta.json, report.json, report.txt.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s1_report  # noqa: E402

WS_DIR = Path(os.environ.get("THESIS_WS", Path(__file__).resolve().parents[1]))
ENV_SH = os.environ.get("S1_ENV_SH", "/root/env_thesis.sh")
CONFIG_YAML_FILE = os.environ.get(
    "S1_CONTROLLERS_YAML",
    "/root/ros_workspaces/ros2/franka_ws/src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/"
    "franka_gazebo_controllers.yaml")
INSTALL_DIR = WS_DIR / "install"          # i pacchetti del repo devono venire da qui (vedi s1_env.py)
ENV_PY = Path(__file__).resolve().parent / "s1_env.py"
PREFLIGHT_PKGS = ["haptic_dmp_learning", "satellite_grasp_planner", "free_target_object", "grasp_monitoring"]
# (pacchetto, eseguibile): binari di cui si registrano sha256/mtime
PREFLIGHT_BINS = {"executor": ("haptic_dmp_learning", "prodmp_gazebo_executor_node"),
                  "planner": ("satellite_grasp_planner", "grasp_planner_node")}
EXECUTOR_MARKER = b"then frozen"          # stringa del log di startPostGraspHold (follow limitato)
URDF_PATH = WS_DIR / "fer_flat_effort.urdf"
PLANNER_YAML = WS_DIR / "src/satellite_grasp_planner/config/grasp_planner_example.yaml"
EXECUTOR_YAML = WS_DIR / "src/haptic_dmp_learning/config/params.yaml"

KT_DEFAULT = 200.0                       # valore a riposo del YAML, ripristinato sempre
SAT_CENTER = (0.70, 0.0, 0.77)           # centro di rotazione / spawn del cubo (default di --center-xyz)
OMEGA_DEG_S_DEFAULT = -2.0               # default di --omega-deg-s
# Parametro di omega, STESSO nome nell'executor e nel planner [deg/s con segno]; in continuous l'executor lo
# vuole esplicito (omega_explicit): il runner lo passa a entrambi con -p, sopra il -2.0 del PLANNER_YAML.
OMEGA_PARAM = "satellite_rotation_angular_velocity_deg_s"
OMEGA_TOL_RAD_S = 0.0017                 # omega_consistency_tol_deg_per_s = 0.1 deg/s (YAML) in rad/s
THETA_STAR_REF_DEG = -163.5              # --theta-star-ref-deg (formula di --spawn-yaw-deg auto)
T_TOTAL_S = 45.2                         # --t-total-s: tau 24.47 + attesa 10.9 + 9.85 tra spawn e pubblicazione
# --arm: braccio di selezione -> --planner-param (stesse chiavi del planner, vedi docs/S3_SPEC.md)
ARMS = {
    "legacy": [("s1.mode", "legacy")],
    "d_only": [("s1.mode", "v2"), ("s1.use_w", "false"), ("s1.branch_rule", "rotation")],
    "v2": [("s1.mode", "v2"), ("s1.branch_rule", "rotation")],
}
START_JOINTS = "[-0.000009,-0.786572,0.000002,-2.378914,-0.000006,3.141609,0.785398]"
RIM_X_C_MM = 155.1                       # x_c atteso del goal = RIM_X_C_MM - depth[mm]
GATE_TOL_MM = 0.5
CONTROLLER_WAIT_S = 70                   # attese di setup (wall): /clock non esiste ancora
EXECUTOR_READY_WALL_S = 60.0             # avvio executor: "gripper: OPEN" + "WAITING for a GraspCommand" (wall, dal Popen)
CUBE_TOPIC_WAIT_S = 30                   # --stop-cube: attesa del topic cmd_vel del cubo (wall)
SPAWN_WAIT_S = 30                        # conferma dello spawn del cubo (wall)
CLOCK_WAIT_S = 30.0                      # primo messaggio su /clock dopo l'avvio dell'executor (wall)
EXECUTOR_READY_MARKERS = ("gripper: OPEN", "WAITING for a GraspCommand")
POST_S_MIN = 45.0                        # minimo del default di --post-s [s sim]
CUBE_NAME = "free_target_object"         # nome del modello (default del launch)
CUBE_ODOM_TOPIC = f"/{CUBE_NAME}/odometry"
STOP_WZ_TOL_DEGPS = 0.2                  # |wz| oltre questo => STOP-CUBE: NOT EFFECTIVE
STOP_READ_AFTER_SIM_S = 1.0              # lettura di wz dopo lo Twist nullo [s sim]
STOP_N_PUB = 3                           # pubblicazioni dello Twist nullo (la prima puo' perdersi: discovery)
STOP_PUB_GAP_S = 0.2
ZERO_TWIST = "linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}"
CLOCK_STALL_WALL_S = 120.0               # /clock fermo per tanto tempo reale => simulazione bloccata

# Pattern (pgrep -f) dei processi che un run lascia vivi se qualcosa va storto
LEFTOVER_PATTERNS = [
    "ros2 bag record", "ign gazebo", "gz sim", "gazebo_cartesian_impedance_control",
    "robot_state_publisher", "joint_state_publisher", "spawner", "controller_manager",
    "joint_state_broadcaster", "cartesian_impedance_controller", "velocity_cartesian_controller",
    "prodmp_gazebo_executor_node", "dmp_gazebo_executor_node", "grasp_planner_node",
    "free_target_object", "parameter_bridge",
]


class RunError(RuntimeError):
    pass


def log(msg):
    print(msg, flush=True)


# ------------------------------------------------------------------ YAML del controllore

def set_yaml_kt(kt: float):
    """Sostituisce il valore di translational_stiffness mantenendo indentazione e commenti."""
    text = Path(CONFIG_YAML_FILE).read_text()
    new, n = re.subn(r"^(\s*translational_stiffness:\s*)[^\s#]+", rf"\g<1>{kt:.1f}", text, flags=re.M)
    if n == 0:
        raise RunError(f"translational_stiffness non trovato in {CONFIG_YAML_FILE}")
    Path(CONFIG_YAML_FILE).write_text(new)


def read_yaml_kt() -> float:
    m = re.search(r"^\s*translational_stiffness:\s*([^\s#]+)", Path(CONFIG_YAML_FILE).read_text(), flags=re.M)
    if not m:
        raise RunError(f"translational_stiffness non trovato in {CONFIG_YAML_FILE}")
    return float(m.group(1))


# ------------------------------------------------------------------ omega, yaw di spawn, centro, braccio (puri)

def omega_rad_s(omega_deg_s: float) -> float:
    return math.radians(omega_deg_s)


def omega_values(omega_deg_s: float) -> dict:
    """I tre punti in cui entra omega, derivati dallo stesso valore: wz del launch del cubo [rad/s],
    parametro dell'executor e parametro del planner [deg/s]. Stringhe come finiscono nei comandi."""
    deg = f"{omega_deg_s:.6f}"
    return {"spawn_wz_rad_s": f"{omega_rad_s(omega_deg_s):.9f}", "executor_param": f"{OMEGA_PARAM}:={deg}",
            "planner_param": f"{OMEGA_PARAM}:={deg}"}


def check_omega_consistent(omega_deg_s: float, tol_rad_s: float = OMEGA_TOL_RAD_S):
    """Il wz di spawn e i due parametri (riletti dalle stringhe dei comandi) devono coincidere entro tol."""
    v = omega_values(omega_deg_s)
    wz = float(v["spawn_wz_rad_s"])
    ex = math.radians(float(v["executor_param"].split(":=")[1]))
    pl = math.radians(float(v["planner_param"].split(":=")[1]))
    if max(abs(wz - ex), abs(wz - pl), abs(ex - pl)) > tol_rad_s:
        raise RunError(f"omega non coerente tra spawn/executor/planner: {wz} {ex} {pl} rad/s")


def wrap_deg_half_open(a: float) -> float:
    """Riporta un angolo [deg] in (-180, 180]."""
    r = (a + 180.0) % 360.0 - 180.0   # [-180, 180)
    return 180.0 if r == -180.0 else r


def auto_spawn_yaw_deg(omega_deg_s: float, theta_star_ref_deg: float = THETA_STAR_REF_DEG,
                       t_total_s: float = T_TOTAL_S) -> float:
    """yaw0 = theta_star_ref - omega*T_total, in (-180, 180]. Per -2 deg/s: ~ -73.1 deg.
    Formula derivata da UN solo punto (tau 24.47 + attesa 10.9 + 9.85 tra spawn e pubblicazione):
    verificare wait_s_actual a ogni run (il report avvisa se esce da [5, 5 + 360/|omega|] s)."""
    return wrap_deg_half_open(theta_star_ref_deg - omega_deg_s * t_total_s)


def parse_spawn_yaw(text: str):
    """'auto' oppure un numero [deg]."""
    if str(text).strip().lower() == "auto":
        return "auto"
    try:
        return float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--spawn-yaw-deg '{text}': atteso un numero o 'auto'")


def parse_center_xyz(text: str):
    """'X,Y,Z' -> (x, y, z) float finiti."""
    parts = str(text).split(",")
    try:
        c = tuple(float(x) for x in parts)
    except ValueError:
        c = ()
    if len(c) != 3 or not all(math.isfinite(x) for x in c):
        raise argparse.ArgumentTypeError(f"--center-xyz '{text}': atteso X,Y,Z (tre numeri)")
    return c


def center_str(c) -> str:
    """Formato dei comandi: [x,y,z] a 3 decimali (0.70 -> 0.700; stessi valori di prima per il default)."""
    return f"[{c[0]:.3f},{c[1]:.3f},{c[2]:.3f}]"


def arm_planner_params(arm) -> list:
    if arm is None:
        return []
    if arm not in ARMS:
        raise RunError(f"--arm '{arm}' sconosciuto (ammessi: {', '.join(ARMS)})")
    return list(ARMS[arm])


# ------------------------------------------------------------------ processi

def _ancestors() -> set[int]:
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def leftover_pids() -> dict[str, list[int]]:
    mine = _ancestors()
    out = {}
    for pat in LEFTOVER_PATTERNS:
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
        pids = [int(x) for x in r.stdout.split() if int(x) not in mine]
        if pids:
            out[pat] = pids
    return out


def ensure_clean(what: str):
    """Uccide eventuali residui e verifica con pgrep che non ne restino."""
    for _ in range(3):
        left = leftover_pids()
        if not left:
            return
        for pids in left.values():
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        time.sleep(1.5)
    left = leftover_pids()
    if left:
        raise RunError(f"processi residui {what}: {', '.join(left)}")


class Proc:
    """Processo bash in una sessione propria: si uccide l'intero gruppo."""

    def __init__(self, name, cmd, logfile, env):
        self.name = name
        self.logfile = Path(logfile)
        self._f = open(self.logfile, "w")
        self.t_start = time.time()
        self.p = subprocess.Popen(["bash", "-c", cmd], env=env, stdout=self._f, stderr=subprocess.STDOUT,
                                  start_new_session=True)

    def poll(self):
        return self.p.poll()

    def text(self) -> str:
        return self.logfile.read_text(errors="replace") if self.logfile.exists() else ""

    def signal_group(self, sig):
        try:
            os.killpg(self.p.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def kill(self):
        self.signal_group(signal.SIGTERM)
        try:
            self.p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self.signal_group(signal.SIGKILL)   # anche se il leader e' uscito, i figli del gruppo possono vivere
        try:
            self.p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self._f.close()


def ros_cmd(body: str) -> str:
    """Source dell'ambiente, poi i prefissi di THESIS_WS/install davanti in AMENT_PREFIX_PATH e
    LD_LIBRARY_PATH (tools/s1_env.py): l'ordine non dipende dalla shell del chiamante ne' da quello che
    fa il source. Tutti i processi figli passano da qui."""
    return (f"source {shlex.quote(ENV_SH)} && "
            f"eval \"$(python3 {shlex.quote(str(ENV_PY))} --root {shlex.quote(str(INSTALL_DIR))})\" && {body}")


def _under_install(prefix: str) -> bool:
    root = os.path.realpath(INSTALL_DIR)
    p = os.path.realpath(prefix)
    return p == root or p.startswith(root + os.sep)


def preflight(env) -> dict:
    """Prima di qualsiasi processo: prefisso dei 4 pacchetti (stesso ambiente dei figli) sotto
    THESIS_WS/install, binari dell'executor e del planner presenti, executor con follow limitato.
    Ritorna il dict per run_meta.json; RunError con UNA riga se qualcosa non va."""
    import hashlib
    prefixes = {}
    for pkg in PREFLIGHT_PKGS:
        r = subprocess.run(["bash", "-c", ros_cmd(f"ros2 pkg prefix {shlex.quote(pkg)}")], env=env,
                           capture_output=True, text=True, timeout=60)
        prefix = r.stdout.strip().splitlines()[-1] if r.returncode == 0 and r.stdout.strip() else ""
        if not prefix:
            raise RunError(f"preflight: pacchetto {pkg} non trovato (ros2 pkg prefix, codice {r.returncode})")
        prefixes[pkg] = prefix
        if not _under_install(prefix):
            raise RunError(f"preflight: {pkg} prefisso {prefix} non sotto {INSTALL_DIR}")
    info = {"pkg_prefixes": prefixes}
    for key, (pkg, exe) in PREFLIGHT_BINS.items():
        path = Path(prefixes[pkg]) / "lib" / pkg / exe
        if not path.is_file():
            raise RunError(f"preflight: binario {path} non trovato")
        data = path.read_bytes()
        st = path.stat()
        info[key] = {"path": str(path), "sha256": hashlib.sha256(data).hexdigest(), "mtime": st.st_mtime,
                     "mtime_iso": datetime.fromtimestamp(st.st_mtime).isoformat(timespec="seconds")}
        if key == "executor":
            info[key]["then_frozen_count"] = data.count(EXECUTOR_MARKER)
            if info[key]["then_frozen_count"] == 0:
                raise RunError("executor senza follow limitato: binario vecchio")
    return info


def parse_planner_param(text: str) -> tuple[str, str]:
    """'chiave=valore' -> (chiave, valore). Chiave vuota/con spazi o ':', '=' mancante o valore vuoto: errore."""
    key, sep, val = text.partition("=")
    key, val = key.strip(), val.strip()
    if not sep:
        raise argparse.ArgumentTypeError(f"--planner-param '{text}': manca '=' (atteso chiave=valore)")
    if not key:
        raise argparse.ArgumentTypeError(f"--planner-param '{text}': chiave vuota")
    if any(c.isspace() or c == ":" for c in key):
        raise argparse.ArgumentTypeError(f"--planner-param '{text}': chiave non valida (spazi o ':')")
    if not val:
        raise argparse.ArgumentTypeError(f"--planner-param '{text}': valore mancante")
    return key, val


def planner_param_dict(pairs: list[tuple[str, str]]) -> dict[str, str]:
    """Come run_meta.json['planner_param']; una chiave ripetuta e' un errore (nessun 'vince l'ultima')."""
    out: dict[str, str] = {}
    for key, val in pairs:
        if key in out:
            raise RunError(f"--planner-param '{key}' ripetuto")
        out[key] = val
    return out


def ros_args(params: list[str]) -> str:
    return " ".join(f"-p {shlex.quote(p)}" for p in params)


# ------------------------------------------------------------------ monitor /clock e /grasp_command

def make_monitor():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from satellite_grasp_msgs.msg import GraspCommand

    class SimMonitor(Node):
        def __init__(self):
            super().__init__("s1_run_monitor")
            self.sim_time = None
            self.wall_last_advance = time.time()
            self.grasp_cmd = None
            self.cube_wz = None   # velocita' angolare del cubo attorno a z [rad/s], da odometria
            self.create_subscription(Odometry, CUBE_ODOM_TOPIC, self._odom, 10)
            self.create_subscription(Clock, "/clock", self._on_clock, 10)
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(GraspCommand, "/grasp_command", self._gc, qos)

        def _on_clock(self, msg):
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.sim_time is None or t > self.sim_time:
                self.wall_last_advance = time.time()
            self.sim_time = t

        def _odom(self, msg):
            self.cube_wz = msg.twist.twist.angular.z

        def _gc(self, msg):
            self.grasp_cmd = msg

    rclpy.init()
    node = SimMonitor()
    from rclpy.executors import ExternalShutdownException

    def _spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:   # rclpy.shutdown() dal thread principale: uscita normale
            pass

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return node, rclpy


# ------------------------------------------------------------------ stop del cubo

def find_cmd_vel_topic(env, model=CUBE_NAME):
    """Topic Ignition che finisce con /cmd_vel e contiene il nome del modello (da `ign topic -l`)."""
    r = subprocess.run(["bash", "-c", ros_cmd("ign topic -l")], env=env, capture_output=True, text=True,
                       timeout=30)
    cands = sorted(t.strip() for t in r.stdout.splitlines() if t.strip().endswith("/cmd_vel") and model in t)
    if cands:
        return next((t for t in cands if t == f"/model/{model}/cmd_vel"), cands[0])
    return f"/model/{model}/cmd_vel"


def publish_zero_twist(topic, env):
    cmd = ros_cmd(f"ign topic -t {shlex.quote(topic)} -m ignition.msgs.Twist -p {shlex.quote(ZERO_TWIST)}")
    subprocess.run(["bash", "-c", cmd], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   timeout=15)


class CubeStopper:
    """Macchina a stati chiamata dal loop post AT_END: attesa "follow: ended" -> delay sim -> 3 Twist nulli
    -> +1 s sim -> lettura wz. Non fatale: l'esito va nel meta e nel log."""

    def __init__(self, a, topic, env, meta, save_meta):
        self.a, self.topic, self.env, self.meta, self.save_meta = a, topic, env, meta, save_meta
        self.t_ended = None   # tempo sim a cui il loop ha visto "follow: ended" (granularita' ~0.5 s wall)
        self.t_pub = None
        self.done = False

    def step(self, log_text, monitor):
        if self.done:
            return
        now = monitor.sim_time
        if self.t_ended is None:
            if "follow: ended" not in log_text:
                return
            self.t_ended = now
            log(f"[sim t={now:.1f}] follow: ended")
        if self.t_pub is None:
            if now < self.t_ended + self.a.stop_cube_delay_s:
                return
            self.t_pub = monitor.sim_time
            for i in range(STOP_N_PUB):
                publish_zero_twist(self.topic, self.env)
                if i < STOP_N_PUB - 1:
                    time.sleep(STOP_PUB_GAP_S)
            self.meta["stop_cube_sim_t"] = self.t_pub
            self.save_meta()
            log(f"[sim t={self.t_pub:.1f}] STOP-CUBE: Twist nullo su {self.topic} x{STOP_N_PUB}")
            return
        if now < self.t_pub + STOP_READ_AFTER_SIM_S:
            return
        wz = monitor.cube_wz
        if wz is None:
            log("STOP-CUBE: NOT EFFECTIVE (nessun campione di odometria del cubo)")
        else:
            wz_deg = math.degrees(wz)
            self.meta["wz_after_stop_degps"] = wz_deg
            log(f"STOP-CUBE: wz = {wz_deg:.4f} deg/s a +{STOP_READ_AFTER_SIM_S:.0f} s sim")
            if abs(wz_deg) > STOP_WZ_TOL_DEGPS:
                log(f"STOP-CUBE: NOT EFFECTIVE (|wz| > {STOP_WZ_TOL_DEGPS} deg/s)")
        self.save_meta()
        self.done = True


# ------------------------------------------------------------------ gate sul goal

def goal_x_c_mm(goal_xy, theta_star_rad, center=SAT_CENTER) -> float:
    """x_c = (goal_xy - centro_xy) . (cos theta*, sin theta*), in mm."""
    dx, dy = goal_xy[0] - center[0], goal_xy[1] - center[1]
    return (dx * math.cos(theta_star_rad) + dy * math.sin(theta_star_rad)) * 1000.0


def goal_from_executor_log(text: str):
    mg = re.search(r"goal=\[([0-9.\-]+),\s*([0-9.\-]+),\s*([0-9.\-]+)\]", text)
    mt = re.search(r"theta_star=([0-9.\-]+)\s*deg", text)
    if mg and mt:
        return (float(mg.group(1)), float(mg.group(2))), math.radians(float(mt.group(1)))
    return None


# ------------------------------------------------------------------ run

def build_commands(a, paths):
    weights = paths["weights"]
    demo = paths["demo_params"]
    depth = f"{a.depth:.6f}"
    c = a.center_xyz
    ov = omega_values(a.omega_deg_s)
    check_omega_consistent(a.omega_deg_s)

    spawn = ros_cmd(
        "exec ros2 launch free_target_object free_target_object.launch.py "
        f"x:={c[0]:.3f} y:={c[1]:.3f} z:={c[2]:.3f} "
        # yaw0 del cubo allo spawn. Per ridurre l'attesa del planner (da ~138 s a ~15 s) conviene
        # sceglierlo cosi': il cubo ruota a omega (<0), e al tempo t_snap dello snapshot del planner
        # deve avere la fase theta_snap_target (quella da cui il planner calcola il TRIGGER):
        #     yaw(t_snap) = yaw0 + omega*(t_snap - t_spawn) = theta_snap_target
        #  => yaw0 = theta_snap_target - omega*(t_snap - t_spawn)
        #          = theta_snap_target + |omega|*(t_snap - t_spawn)        (omega < 0)
        # --spawn-yaw-deg auto: yaw0 = theta_star_ref - omega*T_total (auto_spawn_yaw_deg), ~ -73.1 deg
        # per -2 deg/s; il default 180 deg riproduce run_final2.py (attesa ~138 s).
        # wz = omega del run [rad/s], coerente con il parametro dell'executor e del planner (omega_values).
        f"yaw:={math.radians(a.spawn_yaw_deg):.8f} wz:={ov['spawn_wz_rad_s']}")

    bag = ros_cmd(
        f"exec ros2 bag record -o {shlex.quote(str(paths['bag']))} "
        "/clock /tf /tf_static /joint_states /free_target_object/odometry /target_pose "
        "/grasp_command /gripper_position_cmd")

    exec_params = [
        "use_sim_time:=true",
        f"weights_yaml_path:={weights}",
        f"demo_params_path:={demo}",
        "grasp_goal_source:=grasp_command",
        "grasp_orientation_mode:=planner_goal",
        "hold_target_enabled:=true",
        "axis_deviation_fatal_after_trigger:=false",
        "satellite_rotation_enabled:=true",
        "satellite_rotation_mode:=continuous",
        "satellite_phase_source:=measured",
        "target_odom_required:=false",
        "robot_base_world:=[0.0,0.0,0.0]",
        f"satellite_rotation_center:={center_str(c)}",
        ov["executor_param"],
        f"post_grasp_follow_s:={a.follow_s}",
        f"post_grasp_hold_s:={a.hold_s}",
    ]
    if a.squeeze is not None:
        exec_params.append(f"gripper_squeeze_position:={a.squeeze}")
    exec_params += a.extra_exec_param
    executor = ros_cmd(
        "exec ros2 run haptic_dmp_learning prodmp_gazebo_executor_node --ros-args "
        f"--params-file {shlex.quote(str(EXECUTOR_YAML))} --params-file {shlex.quote(str(PLANNER_YAML))} "
        + ros_args(exec_params))

    centre = center_str(c)
    planner_params = ["use_sim_time:=true"]
    for key, val in [
        ("demo_params_path", str(demo)),
        ("urdf_path", str(URDF_PATH)),
        ("start_joints_override_rad", START_JOINTS),
        ("satellite_rotation_center", centre),
        (OMEGA_PARAM, ov["planner_param"].split(":=")[1]),
        ("cube_geometry.center_world", centre),
        ("cube_geometry.grasp_depth_m", depth),
        ("min_delay_s", "5.0"),
        ("s1.posthoc_velocity_check", "false"),
    ]:
        # le due forme (con e senza prefisso del nodo), come in run_final2.py
        planner_params += [f"grasp_planner_node:{key}:={val}", f"{key}:={val}"]
    # --planner-param: dopo i default (nei -p l'ultimo valore vince), stesse due forme
    for key, val in a.planner_param:
        planner_params += [f"grasp_planner_node:{key}:={val}", f"{key}:={val}"]
    planner = ros_cmd(
        "exec ros2 run satellite_grasp_planner grasp_planner_node --ros-args "
        f"--params-file {shlex.quote(str(PLANNER_YAML))} " + ros_args(planner_params))

    bringup = ros_cmd("exec ros2 launch franka_gazebo_bringup "
                      "gazebo_cartesian_impedance_control_headless.launch.py load_gripper:=true")
    return {"bringup": bringup, "spawn": spawn, "bag": bag, "executor": executor, "planner": planner}


EXIT_INTERRUPTED = 130
BAG_CLOSE_WAIT_S = 15.0


def close_bag(proc, bag_dir, timeout=BAG_CLOSE_WAIT_S):
    """SIGINT al processo del bag, attende la chiusura e che esista metadata.yaml."""
    proc.signal_group(signal.SIGINT)
    try:
        proc.p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        raise RunError(f"bag non chiuso entro {timeout:.0f} s (bag possibilmente corrotto)")
    if not (Path(bag_dir) / "metadata.yaml").exists():
        raise RunError(f"bag chiuso ma {Path(bag_dir) / 'metadata.yaml'} non esiste")


def interrupted_meta(meta: dict, sim_now, t_at_end) -> dict:
    """Marca il meta di un run interrotto; interrupted_rel_s = secondi simulati dopo AT_END (None se
    l'interruzione e' arrivata prima di AT_END o senza /clock). Funzione pura."""
    meta["interrupted"] = True
    meta["interrupted_rel_s"] = (None if sim_now is None or t_at_end is None
                                 else float(sim_now) - float(t_at_end))
    return meta


def run_experiment(a, paths, on_bag_saved):
    outdir = paths["outdir"]
    meta, save_meta = paths["meta"], paths["save_meta"]
    env = os.environ.copy()
    env["FRANKA_START_POSE"] = "ready_q6pi"
    env["FRANKA_FINGERS"] = "prismatic"
    env.pop("QT_QPA_PLATFORM", None)
    if a.gui:
        env["DISPLAY"] = ":0"
    cmds = build_commands(a, paths)

    procs: dict[str, Proc] = {}
    rclpy_mod = None
    monitor = None
    t_at_end = None
    try:
        meta["preflight"] = preflight(env)   # prima di ogni processo (il finally ripristina solo il YAML)
        save_meta()
        log("preflight: " + ", ".join(f"{k}={v}" for k, v in meta["preflight"]["pkg_prefixes"].items()))
        subprocess.run(["bash", "-c", ros_cmd("ros2 daemon stop")], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        ensure_clean("prima del run")
        subprocess.run("rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*", shell=True)
        time.sleep(2.0)

        set_yaml_kt(a.kt)
        log(f"YAML: translational_stiffness = {read_yaml_kt()}")

        procs["bringup"] = Proc("bringup", cmds["bringup"], outdir / "bringup.log", env)
        log("attesa cartesian_impedance_controller...")
        for i in range(CONTROLLER_WAIT_S):
            time.sleep(1.0)
            if procs["bringup"].poll() is not None:
                raise RunError("bringup terminato prematuramente (vedi bringup.log)")
            r = subprocess.run(["bash", "-c", ros_cmd("ros2 control list_controllers 2>/dev/null")],
                               capture_output=True, text=True)
            if "cartesian_impedance_controller" in r.stdout and "active" in r.stdout:
                log(f"controller attivo (dopo {i + 1} s)")
                break
        else:
            raise RunError(f"controller non attivo entro {CONTROLLER_WAIT_S} s")

        if a.gui:
            procs["gui"] = Proc("gui", "DISPLAY=:0 exec ign gazebo -g", outdir / "gui.log", env)
        time.sleep(5.0)   # assestamento del robot

        monitor, rclpy_mod = make_monitor()
        time.sleep(2.0)

        procs["spawn"] = Proc("spawn", cmds["spawn"], outdir / "spawn.log", env)
        for _ in range(SPAWN_WAIT_S):
            time.sleep(1.0)
            if "OK creation of entity." in procs["spawn"].text():
                break
        else:
            raise RunError("spawn del cubo non confermato ('OK creation of entity.' assente)")
        time.sleep(2.0)

        stopper = None
        if a.stop_cube:
            topic = None
            for _ in range(CUBE_TOPIC_WAIT_S):   # il plugin sottoscrive qualche istante dopo lo spawn
                topic = find_cmd_vel_topic(env)
                if topic:
                    break
                time.sleep(1.0)
            if not topic:
                raise RunError(f"--stop-cube: nessun topic */cmd_vel contenente '{CUBE_NAME}' in `ign topic -l`")
            meta["stop_cube_topic"] = topic
            save_meta()
            log(f"stop-cube: topic {topic}")
            stopper = CubeStopper(a, topic, env, meta, save_meta)

        procs["bag"] = Proc("bag", cmds["bag"], outdir / "bag.log", env)
        time.sleep(2.0)

        procs["executor"] = Proc("executor", cmds["executor"], outdir / "executor.log", env)
        t0 = procs["executor"].t_start
        while True:
            time.sleep(0.5)
            txt = procs["executor"].text()
            if procs["executor"].poll() is not None:
                raise RunError(f"executor terminato prematuramente (codice {procs['executor'].poll()})")
            if "exception" in txt.lower():
                raise RunError("eccezione nel log dell'executor")
            missing = [m for m in EXECUTOR_READY_MARKERS if m not in txt]
            if not missing:
                log("executor pronto, 'gripper: OPEN'")
                break
            if time.time() - t0 > EXECUTOR_READY_WALL_S:
                tail = " | ".join(txt.splitlines()[-5:])
                raise RunError(f"executor non pronto entro {EXECUTOR_READY_WALL_S:.0f} s dall'avvio: "
                               f"riga/e mancante/i: {', '.join(repr(m) for m in missing)}; "
                               f"ultime 5 righe del log: {tail!r}")

        procs["planner"] = Proc("planner", cmds["planner"], outdir / "planner.log", env)

        # --- monitor su tempo simulato
        t_wait = time.time()
        while monitor.sim_time is None:
            time.sleep(0.5)
            if time.time() - t_wait > CLOCK_WAIT_S:
                raise RunError("/clock non pubblicato")
        sim_start = monitor.sim_time
        gate_done = accepted = False
        expected_xc = RIM_X_C_MM - a.depth * 1000.0
        while True:
            time.sleep(1.0)
            now = monitor.sim_time
            if now - sim_start > a.sim_timeout:
                raise RunError(f"timeout in tempo simulato ({now - sim_start:.0f} s > {a.sim_timeout:.0f} s) "
                               "senza AT_END")
            if time.time() - monitor.wall_last_advance > CLOCK_STALL_WALL_S:
                raise RunError(f"/clock fermo da {CLOCK_STALL_WALL_S:.0f} s reali (simulazione bloccata)")
            txt = procs["executor"].text()
            if procs["executor"].poll() is not None:
                raise RunError(f"executor terminato (codice {procs['executor'].poll()}) prima di AT_END")

            if not gate_done:
                gc = monitor.grasp_cmd
                if gc is not None:
                    goal = ((gc.goal_pose.position.x, gc.goal_pose.position.y), gc.theta_star_rad)
                elif "GraspCommand ACCEPTED" in txt:
                    goal = goal_from_executor_log(txt)
                else:
                    goal = None
                if goal is not None:
                    xc = goal_x_c_mm(*goal, center=a.center_xyz)
                    log(f"gate goal: x_c = {xc:.4f} mm (atteso {expected_xc:.4f} +- {GATE_TOL_MM} mm)")
                    if abs(xc - expected_xc) > GATE_TOL_MM:
                        raise RunError(f"gate goal fallito: x_c={xc:.4f} mm != {expected_xc:.4f}+-{GATE_TOL_MM} mm "
                                       "(grasp_depth_m non applicato?)")
                    gate_done = True
            if not accepted and "GraspCommand ACCEPTED" in txt:
                accepted = True
                log(f"[sim t={now:.1f}] GraspCommand ACCEPTED")
            m = s1_report._RE_AT_END.search(txt)
            if m:
                t_at_end = float(m.group(1))
                log(f"[sim t={now:.1f}] AT_END a t={t_at_end:.3f}")
                break
            if "completed at goal" in txt:
                t_at_end = now
                log(f"[sim t={now:.1f}] 'completed at goal' (AT_END non trovato nel log)")
                break
        if not gate_done:
            raise RunError("gate goal mai eseguito (nessun GraspCommand prima di AT_END)")

        after_end = a.post_s
        while monitor.sim_time < t_at_end + after_end:
            time.sleep(0.5)
            if time.time() - monitor.wall_last_advance > CLOCK_STALL_WALL_S:
                raise RunError("/clock fermo durante la registrazione post AT_END")
            if stopper:
                stopper.step(procs["executor"].text(), monitor)
        if stopper and not stopper.done:
            log("STOP-CUBE: NOT EFFECTIVE (sequenza non completata prima della fine della registrazione)")
        log(f"{after_end:.0f} s simulati dopo AT_END: stop bag")
        close_bag(procs["bag"], paths["bag"])

        on_bag_saved()

        if a.keep_alive:
            stop = outdir / "STOP"
            log(f"--keep-alive: Gazebo resta attivo, `touch {stop}` per terminare")
            while not stop.exists():
                time.sleep(1.0)
    except KeyboardInterrupt:
        # SIGINT/SIGTERM: un secondo segnale non deve interrompere chiusura del bag e cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        log("INTERROTTO: chiusura del bag e cleanup")
        interrupted_meta(meta, monitor.sim_time if monitor is not None else None, t_at_end)
        if "bag" in procs:
            try:
                close_bag(procs["bag"], paths["bag"])
            except RunError as e:
                meta["bag_close_error"] = str(e)
        save_meta()
        raise
    finally:
        for p in reversed(list(procs.values())):
            p.kill()
        if rclpy_mod is not None:
            try:
                rclpy_mod.shutdown()
            except Exception:
                pass
        try:
            ensure_clean("dopo il run")
        finally:
            # Kt torna a 200 anche su eccezione
            set_yaml_kt(KT_DEFAULT)
            log(f"YAML: translational_stiffness ripristinato a {KT_DEFAULT}")


def resolve_weights(weights_dir: Path):
    w, d = weights_dir / "weights.yaml", weights_dir / "weights_demo_params.yaml"
    for f in (w, d):
        if not f.exists():
            raise RunError(f"file mancante: {f}")
    return str(w.resolve()), str(d.resolve())


def make_parser():
    ap = argparse.ArgumentParser(description="Un run S1 + report", formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--weights-dir", required=True, help="cartella con weights.yaml e weights_demo_params.yaml")
    ap.add_argument("--kt", type=float, default=2000.0, help="translational_stiffness [N/m] (2000)")
    ap.add_argument("--depth", type=float, default=0.0125, help="cube_geometry.grasp_depth_m [m] (0.0125)")
    ap.add_argument("--follow-s", type=float, default=3.0, help="post_grasp_follow_s [s] (3.0)")
    ap.add_argument("--hold-s", type=float, default=120.0, help="post_grasp_hold_s [s] (120)")
    ap.add_argument("--squeeze", type=float, default=None,
                    help="gripper_squeeze_position [m]; se omesso il parametro NON viene passato")
    ap.add_argument("--stop-cube", action="store_true",
                    help="dopo 'follow: ended' pubblica uno Twist nullo sul cmd_vel del cubo (default: no);\n"
                         "richiede --follow-s > 0; senza topic */cmd_vel in `ign topic -l` il run fallisce")
    ap.add_argument("--stop-cube-delay-s", type=float, default=0.0,
                    help="attesa in tempo simulato dopo 'follow: ended' prima dello stop [s] (0.0)")
    ap.add_argument("--post-s", type=float, default=None,
                    help="registrazione dopo AT_END [s di tempo SIMULATO] (default max(45, follow_s + delay + 4)).\n"
                         "Senza --keep-alive il runner si ferma da solo dopo --post-s secondi di tempo simulato\n"
                         "dopo AT_END: chiude il bag, scrive il report e termina; non serve alcun file STOP.")
    ap.add_argument("--spawn-yaw-deg", type=parse_spawn_yaw, default=180.0, metavar="DEG|auto",
                    help="yaw iniziale del cubo [deg] (180 = run_final2; ~-73 riduce l'attesa a ~15 s,\n"
                         "vedi la derivazione nel commento di build_commands). 'auto': yaw0 =\n"
                         "theta_star_ref - omega*T_total in (-180, 180] (--theta-star-ref-deg, --t-total-s);\n"
                         "formula da UN solo punto: controllare wait_s_actual a ogni run")
    ap.add_argument("--omega-deg-s", type=float, default=OMEGA_DEG_S_DEFAULT,
                    help="velocita' angolare del cubo [deg/s, con segno] (-2). Impostata nello stesso valore nel wz\n"
                         f"del launch del cubo (rad/s) e nel parametro {OMEGA_PARAM}\n"
                         "di executor e planner")
    ap.add_argument("--theta-star-ref-deg", type=float, default=THETA_STAR_REF_DEG,
                    help="theta* di riferimento per --spawn-yaw-deg auto [deg] (-163.5)")
    ap.add_argument("--t-total-s", type=float, default=T_TOTAL_S,
                    help="tempo spawn->contatto per --spawn-yaw-deg auto [s] (45.2)")
    ap.add_argument("--center-xyz", type=parse_center_xyz, default=SAT_CENTER, metavar="X,Y,Z",
                    help="centro di rotazione / spawn del cubo [m] (0.70,0.0,0.77); stesso valore in spawn, executor,\n"
                         "planner (satellite_rotation_center, cube_geometry.center_world) e gate sul goal")
    ap.add_argument("--arm", choices=sorted(ARMS), default=None,
                    help="braccio di selezione, espanso in --planner-param: "
                         + "; ".join(f"{k}: " + " ".join(f"{a}={b}" for a, b in v) for k, v in ARMS.items()))
    ap.add_argument("--gui", action=argparse.BooleanOptionalAction, default=False,
                    help="apre `ign gazebo -g` su DISPLAY=:0 (default: no, batch)")
    ap.add_argument("--keep-alive", action=argparse.BooleanOptionalAction, default=False,
                    help="solo con --keep-alive il runner, dopo bag e report, lascia Gazebo attivo e aspetta il file STOP\n"
                         "in outdir. Senza --keep-alive (default) il runner si ferma da solo dopo --post-s secondi\n"
                         "di tempo simulato dopo AT_END e non serve alcun file STOP.")
    ap.add_argument("--outdir", help="default: s1_runs/<tag>")
    ap.add_argument("--tag", help="default: s1_<timestamp>")
    ap.add_argument("--extra-exec-param", action="append", default=[], metavar="NOME:=VALORE",
                    help="parametro aggiuntivo dell'executor (-p), ripetibile")
    ap.add_argument("--planner-param", action="append", default=[], type=parse_planner_param,
                    metavar="CHIAVE=VALORE",
                    help="parametro ROS aggiuntivo del planner (ripetibile), es. --planner-param s1.force=true;\n"
                         "aggiunto dopo i default del runner; salvato in run_meta.json e nel report")
    ap.add_argument("--sim-timeout", type=float, default=500.0,
                    help="timeout in tempo simulato (/clock) per arrivare ad AT_END [s] (500)")
    ap.add_argument("--overwrite", action="store_true", help="permette di riusare un outdir con bag/ esistente")
    return ap


def main(argv=None) -> int:
    a = make_parser().parse_args(argv)
    a.tag = a.tag or datetime.now().strftime("s1_%Y%m%dT%H%M%S")
    outdir = Path(a.outdir) if a.outdir else WS_DIR / "s1_runs" / a.tag
    outdir = outdir.resolve()

    def _on_sigterm(*_):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    rc = 0
    meta = None
    report = None
    try:
        if not (math.isfinite(a.omega_deg_s) and a.omega_deg_s != 0.0):
            raise RunError("--omega-deg-s deve essere finito e != 0")
        a.spawn_yaw_mode = "auto" if a.spawn_yaw_deg == "auto" else "fixed"
        if a.spawn_yaw_deg == "auto":
            a.spawn_yaw_deg = auto_spawn_yaw_deg(a.omega_deg_s, a.theta_star_ref_deg, a.t_total_s)
            log(f"spawn yaw auto = {a.theta_star_ref_deg:g} - ({a.omega_deg_s:g})*{a.t_total_s:g} "
                f"= {a.spawn_yaw_deg:.3f} deg (in (-180, 180])")
        a.planner_param = arm_planner_params(a.arm) + list(a.planner_param)   # il braccio per primo
        if a.stop_cube and not a.follow_s > 0.0:
            raise RunError("--stop-cube richiede --follow-s > 0")
        if a.post_s is None:
            a.post_s = max(POST_S_MIN, a.follow_s + a.stop_cube_delay_s + 4.0)
        if a.post_s <= 0.0:
            raise RunError("--post-s deve essere > 0")
        if a.stop_cube and a.post_s < a.follow_s + a.stop_cube_delay_s + STOP_READ_AFTER_SIM_S:
            raise RunError("--post-s troppo corto per follow_s + stop-cube-delay-s + 1 s")
        if a.stop_cube_delay_s < 0.0:
            raise RunError("--stop-cube-delay-s deve essere >= 0")
        weights, demo = resolve_weights(Path(a.weights_dir).resolve())
        bag = outdir / "bag"
        if bag.exists():
            if not a.overwrite:
                raise RunError(f"{bag} esiste gia' (usa --overwrite o un altro --tag)")
            shutil.rmtree(bag)
        outdir.mkdir(parents=True, exist_ok=True)
        paths = {"outdir": outdir, "bag": bag, "weights": weights, "demo_params": demo}
        meta = {"tag": a.tag, "depth": a.depth, "kt": a.kt, "follow_s": a.follow_s, "hold_s": a.hold_s,
                "squeeze": a.squeeze, "spawn_yaw_deg": a.spawn_yaw_deg, "weights_dir": str(weights),
                "spawn_yaw_mode": a.spawn_yaw_mode, "theta_star_ref_deg": a.theta_star_ref_deg,
                "t_total_s": a.t_total_s, "omega_deg_s": a.omega_deg_s, "center_xyz": list(a.center_xyz),
                "arm": a.arm,
                "extra_exec_param": a.extra_exec_param, "sim_timeout": a.sim_timeout,
                "planner_param": planner_param_dict(a.planner_param),
                "stop_cube": a.stop_cube, "stop_cube_delay_s": a.stop_cube_delay_s, "post_s": a.post_s}

        def save_meta():
            (outdir / "run_meta.json").write_text(json.dumps(meta, indent=2))

        save_meta()
        paths["meta"], paths["save_meta"] = meta, save_meta

        def report():
            res = s1_report.analyze(bag, outdir / "executor.log", meta)
            s1_report.write_outputs(res, outdir)
            log(f"report: {outdir / 'report.txt'}  success={res['success']}")

        run_experiment(a, paths, report)
    except RunError as e:
        # run_experiment ha gia' ripulito; "processi residui dopo il run" => codice 3
        msg = str(e)
        rc = 3 if "processi residui dopo il run" in msg else 1
        if e.__context__ is not None:   # errore del run mascherato dal controllo dei residui
            msg += f" (causa: {str(e.__context__).splitlines()[0] if str(e.__context__) else type(e.__context__).__name__})"
        print(f"S1_RUN FAIL: {msg.splitlines()[0]}", file=sys.stderr)
    except s1_report.ReportError as e:
        rc = 1
        print(f"S1_RUN FAIL: report: {str(e).splitlines()[0]}", file=sys.stderr)
    except KeyboardInterrupt:
        rc = EXIT_INTERRUPTED
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print("S1_RUN INTERRUPTED: run interrotto (SIGINT/SIGTERM)", file=sys.stderr)
        if meta is not None and report is not None and meta.get("interrupted") \
                and (outdir / "bag" / "metadata.yaml").exists():
            try:
                report()
            except Exception as e:  # noqa: BLE001
                print(f"S1_RUN INTERRUPTED: report non generato: {type(e).__name__}: "
                      f"{str(e).splitlines()[0] if str(e) else ''}", file=sys.stderr)
    except Exception as e:  # noqa: BLE001
        rc = 1
        print(f"S1_RUN FAIL: {type(e).__name__}: {str(e).splitlines()[0] if str(e) else ''}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())
