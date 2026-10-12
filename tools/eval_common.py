#!/usr/bin/env python3
"""
tools/eval_common.py — Modulo comune per la valutazione canonica dei controllori Franka in Gazebo Sim.
Include gestione processi, sincronizzazione clock/sim_time, estrazione bag, cinematica e metriche.
"""

import os
import sys
import hashlib
import time
import subprocess
import signal
import math
import sqlite3
import glob
import bisect
import numpy as np

WS_ROOT = "/root/thesis_ws"
FRANKA_WS = "/root/ros_workspaces/ros2/franka_ws"
BRINGUP_SHARE = os.path.join(FRANKA_WS, "install/franka_gazebo_bringup/share/franka_gazebo_bringup")
CONFIG_YAML_SRC = os.path.join(FRANKA_WS, "src/franka_ros2/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml")
CONFIG_YAML_OVERRIDE = os.path.join(WS_ROOT, "src/franka_gazebo_overrides/franka_gazebo_controllers.yaml")
CONFIG_YAML_INSTALL = os.path.join(BRINGUP_SHARE, "config/franka_gazebo_controllers.yaml")

WEIGHTS_PRODMP = os.path.join(WS_ROOT, "runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml")
ORIENTATION_PATH = os.path.join(WS_ROOT, "reach_task_baseline_nbasis200.yaml")

# Override xacro: file nel repo -> file installato (percorsi di scripts/setup_franka_description_overrides.sh)
FRANKA_DESCRIPTION_SRC = os.path.join(FRANKA_WS, "src/franka_ros2/franka_description")
XACRO_OVERRIDES = {
    "franka_hand.xacro": (
        os.path.join(WS_ROOT, "src/franka_description_overrides/franka_hand.xacro"),
        os.path.join(FRANKA_DESCRIPTION_SRC, "end_effectors/common/franka_hand.xacro"),
    ),
    "franka_arm.ros2_control.xacro": (
        os.path.join(WS_ROOT, "src/franka_description_overrides/franka_arm.ros2_control.xacro"),
        os.path.join(FRANKA_DESCRIPTION_SRC, "robots/common/franka_arm.ros2_control.xacro"),
    ),
}


# ==============================================================================
# 1. GESTIONE PROCESSI E WORKSPACE
# ==============================================================================

def clean_processes():
    """Arresta tutti i processi Gazebo, ROS 2, bridge e nodi controller."""
    cmds = [
        "pkill -9 -f 'ros2 bag record'",
        "pkill -9 -f 'ign gazebo'",
        "pkill -9 -f 'gz_sim'",
        "pkill -9 -f 'ruby'",
        "pkill -9 -f 'robot_state_publisher'",
        "pkill -9 -f 'joint_state_publisher'",
        "pkill -9 -f 'spawner'",
        "pkill -9 -f 'controller_manager'",
        "pkill -9 -f 'joint_state_broadcaster'",
        "pkill -9 -f 'cartesian_impedance_controller'",
        "pkill -9 -f 'velocity_cartesian_controller'",
        "pkill -9 -f 'prodmp_gazebo_executor_node'",
        "pkill -9 -f 'dmp_gazebo_executor_node'",
        "pkill -9 -f 'parameter_bridge'",
        "rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*",
    ]
    for c in cmds:
        try:
            subprocess.run(c, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
        except subprocess.TimeoutExpired:
            print(f"[ATTENZIONE] timeout (15 s) in clean_processes: {c}", flush=True)
    time.sleep(1.0)


def _sha256_file(path):
    """(sha256 hex, percorso risolto, None) oppure (None, percorso risolto, motivo) se illeggibile."""
    real = os.path.realpath(path)
    try:
        h = hashlib.sha256()
        with open(real, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest(), real, None
    except OSError as e:
        return None, real, f"{type(e).__name__}: {e.strerror or e}"


def check_overrides_in_sync():
    """Confronta lo sha256 degli override xacro nel repo con quelli installati in franka_description.

    Ritorna (ok, dettagli): ok = True (allineati), False (almeno uno diverso), None (NON VERIFICABILE:
    nessuna differenza trovata ma almeno un file illeggibile). dettagli = {"status", "messages", "files"}.
    """
    files = {}
    messages = []
    any_diff = False
    any_unverifiable = False
    for name, (repo_path, inst_path) in XACRO_OVERRIDES.items():
        repo_sha, repo_real, repo_err = _sha256_file(repo_path)
        inst_sha, inst_real, inst_err = _sha256_file(inst_path)
        entry = {"repo_path": repo_real, "installed_path": inst_real,
                 "sha256_repo": repo_sha, "sha256_installed": inst_sha}
        if repo_sha is None or inst_sha is None:
            entry["status"] = "NON VERIFICABILE"
            reason = "; ".join(r for r in (
                f"repo: {repo_err}" if repo_err else None,
                f"installato ({inst_real}): {inst_err}" if inst_err else None) if r)
            entry["reason"] = reason
            any_unverifiable = True
            messages.append(f"NON VERIFICABILE: {name} ({reason})")
        elif repo_sha != inst_sha:
            entry["status"] = "DIVERSO"
            any_diff = True
            messages.append(
                f"OVERRIDE NON ALLINEATO: {name} (sha repo {repo_sha} sha installato {inst_sha}). "
                f"Esegui scripts/setup_franka_description_overrides.sh")
        else:
            entry["status"] = "ALLINEATO"
        files[name] = entry
    if any_diff:
        ok, status = False, "NON ALLINEATO"
    elif any_unverifiable:
        ok, status = None, "NON VERIFICABILE"
    else:
        ok, status = True, "ALLINEATO"
    return ok, {"status": status, "messages": messages, "files": files}


def setup_bringup_share():
    """Sincronizza gli override dei file di launch e config in install/franka_gazebo_bringup."""
    os.makedirs(os.path.join(BRINGUP_SHARE, "launch"), exist_ok=True)
    os.makedirs(os.path.join(BRINGUP_SHARE, "config"), exist_ok=True)
    for fname in [
        "gazebo_cartesian_impedance_control.launch.py",
        "gazebo_cartesian_impedance_control_headless.launch.py",
        "gazebo_velocity_cartesian_control.launch.py",
    ]:
        src = os.path.join(WS_ROOT, "src/franka_gazebo_overrides", fname)
        dst = os.path.join(BRINGUP_SHARE, "launch", fname)
        if os.path.exists(src):
            import shutil
            shutil.copyfile(src, dst)
    if os.path.exists(CONFIG_YAML_OVERRIDE):
        import shutil
        if os.path.islink(CONFIG_YAML_INSTALL):
            os.remove(CONFIG_YAML_INSTALL)
        shutil.copyfile(CONFIG_YAML_OVERRIDE, CONFIG_YAML_INSTALL)
        if os.path.exists(CONFIG_YAML_SRC):
            shutil.copyfile(CONFIG_YAML_OVERRIDE, CONFIG_YAML_SRC)


class RunInvalid(Exception):
    """La run non e' valida e va chiusa (stato non pulito, launch morto, ...)."""


class SigtermAsException:
    """Context manager: SIGTERM (e SIGHUP) alzano un'eccezione, cosi' i finally/__exit__ eseguono sempre."""
    def __enter__(self):
        def _raise(signum, frame):
            raise KeyboardInterrupt(f"segnale {signum} ricevuto")
        self._old = {}
        for sig in (signal.SIGTERM, signal.SIGHUP):
            self._old[sig] = signal.signal(sig, _raise)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        for sig, old in self._old.items():
            signal.signal(sig, old)


class ProcessGuard:
    """Context manager: all'uscita (anche per eccezione/segnale) chiude Gazebo, bag, executor."""
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        clean_processes()


GAIN_KEYS = {"Kt": "translational_stiffness", "Kr": "rotational_stiffness",
             "Dt": "translational_damping", "Dr": "rotational_damping"}


def read_gains_yaml(path=None):
    """sha256 e guadagni (Kt, Kr, Dt, Dr) letti dallo yaml che il controller_manager carica.

    Se una chiave compare con valori diversi nel file, il valore e' la lista dei valori trovati.
    """
    path = path or CONFIG_YAML_INSTALL
    sha, real, err = _sha256_file(path)
    info = {"path": real, "sha256": sha}
    if err:
        info["error"] = err
        return info
    with open(real, "r") as f:
        lines = f.read().splitlines()
    for short, key in GAIN_KEYS.items():
        vals = []
        for line in lines:
            code = line.split("#", 1)[0].strip()
            if code.startswith(key + ":"):
                vals.append(float(code.split(":", 1)[1]))
        info[short] = vals[0] if len(set(vals)) == 1 else (vals or None)
    return info


def check_gains_yaml_clean():
    """Il file tracciato in src non deve differire da HEAD. Ritorna (ok, messaggio)."""
    import difflib
    rel = os.path.relpath(CONFIG_YAML_OVERRIDE, WS_ROOT)
    try:
        res = subprocess.run(["git", "-C", WS_ROOT, "show", f"HEAD:{rel}"],
                             capture_output=True, text=True, timeout=20)
    except (subprocess.TimeoutExpired, OSError) as e:
        return False, f"YAML DEI GUADAGNI NON VERIFICABILE: git show fallito ({type(e).__name__})"
    if res.returncode != 0:
        return False, f"YAML DEI GUADAGNI NON VERIFICABILE: git show HEAD:{rel}: {res.stderr.strip()}"
    try:
        with open(CONFIG_YAML_OVERRIDE, "r") as f:
            cur = f.read()
    except OSError as e:
        return False, f"YAML DEI GUADAGNI NON VERIFICABILE: {e}"
    if cur == res.stdout:
        return True, "YAML dei guadagni identico a HEAD"
    diff = [l for l in difflib.unified_diff(res.stdout.splitlines(), cur.splitlines(),
                                            "HEAD", "working tree", lineterm="", n=0)
            if l[:1] in "+-" and not l.startswith(("+++", "---"))]
    return False, "YAML DEI GUADAGNI MODIFICATO RISPETTO A HEAD: " + " | ".join(diff[:10])


def find_other_eval_runs():
    """PID di altri processi python che eseguono eval_controllers.py (escluso questo)."""
    pids = []
    me = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
        except OSError:
            continue
        if argv and os.path.basename(argv[0]).startswith("python") and \
                any(os.path.basename(a) == "eval_controllers.py" for a in argv[1:]):
            pids.append(int(entry))
    return pids


class YamlGainManager:
    """Context manager per impostare e ripristinare i guadagni del controller impedance.

    Modifica SOLO la copia regolare nel bringup (CONFIG_YAML_INSTALL), cioe' il file che il
    controller_manager carica via $(find franka_gazebo_bringup)/config/... nello xacro ros2_control.
    I file in src (override tracciato, franka_ws/src) non vengono mai scritti.
    """
    def __init__(self, kt=200.0, dt=10.0, kr=150.0, dr=1.0):
        self.kt = kt
        self.dt = dt
        self.kr = kr
        self.dr = dr
        self.saved_contents = {}

    def __enter__(self):
        path = CONFIG_YAML_INSTALL
        if os.path.islink(path):
            raise RunInvalid(f"{path} e' un symlink: rifiuto di modificarlo (scriverebbe il file sorgente)")
        try:
            with open(path, "r") as f:
                original = f.read()
            self.saved_contents[path] = original
            new_lines = []
            for line in original.splitlines(keepends=True):
                if "translational_stiffness:" in line:
                    new_lines.append(f"    translational_stiffness: {self.kt:.1f}\n")
                elif "translational_damping:" in line:
                    new_lines.append(f"    translational_damping: {self.dt:.1f}\n")
                elif "rotational_stiffness:" in line:
                    new_lines.append(f"    rotational_stiffness: {self.kr:.1f}\n")
                elif "rotational_damping:" in line:
                    new_lines.append(f"    rotational_damping: {self.dr:.1f}\n")
                else:
                    new_lines.append(line)
            with open(path, "w") as f:
                f.writelines(new_lines)
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        for path, content in self.saved_contents.items():
            with open(path, "w") as f:
                f.write(content)
        return False


LAUNCH_ERROR_MARKERS = ("[ERROR]", "process has died")


def _log_tail(path, n=20):
    try:
        with open(path, "r", errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError as e:
        return f"(log illeggibile: {e})"


def wait_for_controller(launch_log_path, controller_name="cartesian_impedance_controller", timeout_sec=60,
                        launch_proc=None, poll_timeout_sec=10):
    """Attende che il controller sia attivo. True = attivo, False = timeout.

    Ogni giro controlla che il processo di launch sia vivo e che il log non contenga ERROR di launch:
    in tal caso alza RunInvalid con le ultime 20 righe del log. Ogni chiamata a ros2 control ha timeout.
    """
    check_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {FRANKA_WS}/install/setup.bash && "
        f"exec ros2 control list_controllers 2>/dev/null"
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        log_text = ""
        if os.path.exists(launch_log_path):
            with open(launch_log_path, "r", errors="replace") as f:
                log_text = f.read()
        activated = (f"Configured and activated {controller_name}" in log_text or
                     f"Successfully loaded controller {controller_name}" in log_text)
        if activated:
            return True
        if launch_proc is not None and launch_proc.poll() is not None:
            raise RunInvalid(f"processo di launch terminato (rc={launch_proc.returncode}) prima dell'attivazione "
                             f"del controller. Ultime 20 righe del log:\n{_log_tail(launch_log_path)}")
        bad = [m for m in LAUNCH_ERROR_MARKERS if m in log_text]
        if bad:
            raise RunInvalid(f"errore di launch nel log ({', '.join(bad)}). "
                             f"Ultime 20 righe del log:\n{_log_tail(launch_log_path)}")
        try:
            res = subprocess.run(check_cmd, shell=True, executable="/bin/bash", capture_output=True,
                                 text=True, timeout=poll_timeout_sec)
            if controller_name in res.stdout and "active" in res.stdout:
                return True
        except subprocess.TimeoutExpired:
            pass  # ros2 control appeso: si riprova fino alla scadenza del deadline
        time.sleep(1.0)
    return False


# Posa di partenza: q6 atteso [rad] per ogni valore di FRANKA_START_POSE
# (src/franka_description_overrides/franka_arm.ros2_control.xacro).
EXPECTED_Q6_RAD = {
    "ready": math.pi / 2.0,
    "ready_q6pi": math.pi,
    "capture": 1.2155,
}
BUILD_PACKAGES = ["haptic_dmp_learning", "satellite_grasp_planner", "franka_cartesian_control"]


ROS_ENV_HINT = ("source /opt/ros/humble/setup.bash && source /root/ros_workspaces/ros2/franka_ws/install/setup.bash && "
                "source /root/thesis_ws/install/setup.bash")


def check_ros_python_env():
    """None se rclpy e sensor_msgs.msg.JointState sono importabili, altrimenti la descrizione dell'eccezione."""
    try:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import JointState  # noqa: F401
    except Exception as e:
        return f"{type(e).__name__}: {e}"
    return None


def check_start_pose(q6_rad, requested_pose, tol_rad=0.01, unread_reason=None):
    """Confronta q6 letto con quello atteso dalla posa richiesta. Ritorna (ok, messaggio).

    q6 non letto -> "POSA DI PARTENZA NON VERIFICABILE: <motivo>"; q6 letto ma diverso -> "POSA DI PARTENZA ERRATA".
    """
    expected = EXPECTED_Q6_RAD.get(requested_pose)
    if expected is None:
        return False, (f"POSA DI PARTENZA ERRATA: richiesta {requested_pose}, q6 = {q6_rad} "
                       f"(posa richiesta sconosciuta; valori noti: {sorted(EXPECTED_Q6_RAD)})")
    if q6_rad is None or not math.isfinite(q6_rad):
        return False, f"POSA DI PARTENZA NON VERIFICABILE: {unread_reason or 'q6 non letto (motivo non indicato)'}"
    if abs(q6_rad - expected) > tol_rad:
        return False, f"POSA DI PARTENZA ERRATA: richiesta {requested_pose}, q6 = {q6_rad:.4f}"
    return True, (f"Posa di partenza OK: richiesta {requested_pose}, q6 = {q6_rad:.4f} rad "
                  f"(atteso {expected:.4f}, tol {tol_rad})")


def read_joint6_from_joint_states(timeout_sec=15.0):
    """Legge un campione di /joint_states. Ritorna (q6 [rad], motivo): (valore, None) se letto,
    (None, motivo) altrimenti."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception as e:
        return None, f"rclpy/sensor_msgs non importabili ({type(e).__name__}: {e})"
    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception as e:
        return None, f"rclpy.init fallito ({type(e).__name__}: {e})"
    holder = {"msgs": 0}

    class _JsNode(Node):
        def __init__(self):
            super().__init__('start_pose_guard')
            self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        def cb(self, msg):
            holder["msgs"] += 1
            holder["names"] = list(msg.name)
            pos = dict(zip(msg.name, msg.position))
            if 'fer_joint6' in pos:
                holder['q6'] = float(pos['fer_joint6'])

    node = _JsNode()
    try:
        t_end = time.monotonic() + timeout_sec
        while 'q6' not in holder and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
    if 'q6' in holder:
        return holder['q6'], None
    if holder["msgs"] == 0:
        return None, f"nessun messaggio su /joint_states entro {timeout_sec:g} s"
    return None, f"giunto fer_joint6 assente nel messaggio (nomi: {holder['names']})"


def extract_joint6_initial_value_lines(launch_log_path):
    """Dal log di launch estrae le righe del giunto 6 (sesta occorrenza di 'found initial value'
    e la riga 'Loading joint' che la precede). Ritorna (righe, valore) oppure (["NON TROVATO"], None)."""
    try:
        with open(launch_log_path, "r", errors="ignore") as f:
            lines = f.read().splitlines()
    except OSError:
        return ["NON TROVATO"], None
    count = 0
    for i, line in enumerate(lines):
        if "found initial value" in line:
            count += 1
            if count == 6:
                block = []
                for j in range(i, -1, -1):
                    if "Loading joint" in lines[j]:
                        block.append(lines[j].strip())
                        break
                block.append(line.strip())
                try:
                    value = float(line.split("found initial value:")[-1].strip())
                except ValueError:
                    value = None
                return block, value
    return ["NON TROVATO"], None


def run_metadata():
    """git HEAD, flag dirty (file tracciati modificati) e CMAKE_BUILD_TYPE dei pacchetti C++."""
    def _git(*args):
        try:
            res = subprocess.run(["git", "-C", WS_ROOT] + list(args), capture_output=True, text=True, timeout=20)
            return res.stdout.strip() if res.returncode == 0 else None
        except Exception:
            return None

    head = _git("rev-parse", "HEAD")
    porcelain = _git("status", "--porcelain", "--untracked-files=no")
    build_types = {}
    for pkg in BUILD_PACKAGES:
        cache = os.path.join(WS_ROOT, "build", pkg, "CMakeCache.txt")
        value = "NON TROVATO"
        try:
            with open(cache, "r", errors="ignore") as f:
                for line in f:
                    if line.startswith("CMAKE_BUILD_TYPE:"):
                        value = line.split("=", 1)[1].strip() or "(vuoto)"
                        break
        except OSError:
            pass
        build_types[pkg] = value
    return {
        "git_head": head if head else "NON TROVATO",
        "git_dirty": (len(porcelain) > 0) if porcelain is not None else None,
        "cmake_build_type": build_types,
    }


class SimTimeClockMonitor:
    """Monitora il tempo simulato da /clock usando un nodo rclpy in un background thread."""
    def __init__(self):
        self.sim_time = None
        self._stop = False
        self._thread = None
        self._node = None
        try:
            import rclpy
            from rclpy.node import Node
            from rosgraph_msgs.msg import Clock
            import threading

            if not rclpy.ok():
                rclpy.init()

            class _ClockNode(Node):
                def __init__(self, outer):
                    super().__init__('sim_time_clock_monitor')
                    self.outer = outer
                    self.sub = self.create_subscription(Clock, '/clock', self.cb, 10)
                def cb(self, msg):
                    self.outer.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

            self._node = _ClockNode(self)
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception as e:
            pass

    def _spin(self):
        import rclpy
        while not self._stop and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                break

    def get_time(self):
        if self.sim_time is not None:
            return self.sim_time
        return get_current_sim_time()

    def stop(self):
        self._stop = True
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass


def get_current_sim_time():
    """Legge il tempo simulato istantaneo dal topic /clock via ROS 2."""
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic echo /clock rosgraph_msgs/msg/Clock --once 2>/dev/null"
    )
    try:
        res = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=5)
        if res.returncode == 0:
            lines = res.stdout.splitlines()
            sec, nanosec = 0, 0
            for l in lines:
                if "sec:" in l and "nanosec" not in l:
                    sec = int(l.split(":")[-1].strip())
                elif "nanosec:" in l:
                    nanosec = int(l.split(":")[-1].strip())
            return sec + nanosec * 1e-9
    except Exception:
        pass
    return None


# ==============================================================================
# 2. CINEMATICA DIRETTA (FRANKA FER TCP)
# ==============================================================================

def rpy_to_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def tf_mat(xyz, rpy, angle):
    T = np.eye(4)
    R_fixed = rpy_to_mat(*rpy)
    c, s = np.cos(angle), np.sin(angle)
    R_joint = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T[:3, :3] = R_fixed @ R_joint
    T[:3, 3] = xyz
    return T


def fixed_tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(*rpy)
    T[:3, 3] = xyz
    return T


def forward_kinematics_tcp(q):
    """Calcola posizione e matrice di rotazione del frame fer_hand_tcp data la conf q (7 dof)."""
    pi = np.pi
    T01 = tf_mat([0, 0, 0.333], [0, 0, 0], q[0])
    T12 = tf_mat([0, 0, 0], [-pi/2, 0, 0], q[1])
    T23 = tf_mat([0, -0.316, 0], [pi/2, 0, 0], q[2])
    T34 = tf_mat([0.0825, 0, 0], [pi/2, 0, 0], q[3])
    T45 = tf_mat([-0.0825, 0.384, 0], [-pi/2, 0, 0], q[4])
    T56 = tf_mat([0, 0, 0], [pi/2, 0, 0], q[5])
    T67 = tf_mat([0.088, 0, 0], [pi/2, 0, 0], q[6])
    T78 = fixed_tf([0, 0, 0.107], [0, 0, 0])
    T8h = fixed_tf([0, 0, 0], [0, 0, -pi/4])
    Thtcp = fixed_tf([0, 0, 0.1034], [0, 0, 0])
    T = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 @ T78 @ T8h @ Thtcp
    return T[:3, 3], T[:3, :3]


def rot_to_quat(R):
    """Converte matrice di rotazione (3x3) in quaternione [x, y, z, w]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return [x, y, z, w]


# ==============================================================================
# 3. ESTRAZIONE DATI DA ROSBAG
# ==============================================================================

def extract_bag_data(bag_dir):
    """
    Estrae i topic dal rosbag SQLite3 (.db3):
    Restituisce un dizionario contenente i flussi campionati con sim_time sincronizzato dal /clock.
    """
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    db_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db_files:
        raise FileNotFoundError(f"Nessun file .db3 trovato in {bag_dir}")

    conn = sqlite3.connect(db_files[0])
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = dict(cur.fetchall())
    topics_rev = {v: k for k, v in topics.items()}

    # 1. Clock mapping
    clock_records = []
    if "/clock" in topics_rev:
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topics_rev["/clock"],)
        )
        for ts_ns, raw in cur.fetchall():
            c = deserialize_message(raw, Clock)
            sim_t = c.clock.sec + c.clock.nanosec * 1e-9
            clock_records.append((ts_ns * 1e-9, sim_t))

    def get_sim_time(rec_t_s):
        if not clock_records:
            return rec_t_s
        l, r = 0, len(clock_records) - 1
        while l <= r:
            m = (l + r) // 2
            if clock_records[m][0] == rec_t_s:
                return clock_records[m][1]
            elif clock_records[m][0] < rec_t_s:
                l = m + 1
            else:
                r = m - 1
        idx = max(0, min(r, len(clock_records) - 1))
        return clock_records[idx][1]

    # Determinazione t0 sim
    t_wall_0 = None
    sim_0 = None

    # Topic target_pose_aligned o target_pose
    target_records = []
    tgt_topic = None
    for cand in ["/cartesian_impedance_controller/target_pose_aligned",
                 "/velocity_cartesian_controller/target_pose_aligned",
                 "/target_pose"]:
        if cand in topics_rev:
            tgt_topic = cand
            break

    if tgt_topic:
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topics_rev[tgt_topic],)
        )
        rows = cur.fetchall()
        if rows:
            if t_wall_0 is None:
                t_wall_0 = rows[0][0] * 1e-9
                sim_0 = get_sim_time(t_wall_0)
            for ts_ns, raw in rows:
                p = deserialize_message(raw, PoseStamped)
                t_wall = ts_ns * 1e-9 - t_wall_0
                t_sim = get_sim_time(ts_ns * 1e-9) - sim_0
                target_records.append({
                    "t_sim": t_sim, "t_wall": t_wall,
                    "pos": np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]),
                    "quat": [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z]
                })

    # Topic actual_pose o calcolato da joint_states
    actual_records = []
    act_topic = None
    for cand in ["/cartesian_impedance_controller/actual_pose",
                 "/velocity_cartesian_controller/actual_pose"]:
        if cand in topics_rev:
            act_topic = cand
            break

    if act_topic:
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topics_rev[act_topic],)
        )
        rows = cur.fetchall()
        if rows:
            if t_wall_0 is None:
                t_wall_0 = rows[0][0] * 1e-9
                sim_0 = get_sim_time(t_wall_0)
            for ts_ns, raw in rows:
                p = deserialize_message(raw, PoseStamped)
                t_wall = ts_ns * 1e-9 - t_wall_0
                t_sim = get_sim_time(ts_ns * 1e-9) - sim_0
                actual_records.append({
                    "t_sim": t_sim, "t_wall": t_wall,
                    "pos": np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]),
                    "quat": [p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z]
                })

    # Joint states
    joint_records = []
    if "/joint_states" in topics_rev:
        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topics_rev["/joint_states"],)
        )
        rows = cur.fetchall()
        if rows:
            if t_wall_0 is None:
                t_wall_0 = rows[0][0] * 1e-9
                sim_0 = get_sim_time(t_wall_0)
            joint_names = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"]
            for ts_ns, raw in rows:
                js = deserialize_message(raw, JointState)
                name_to_pos = dict(zip(js.name, js.position))
                if all(jn in name_to_pos for jn in joint_names):
                    q = [name_to_pos[jn] for jn in joint_names]
                    pos, R = forward_kinematics_tcp(q)
                    quat_xyzw = rot_to_quat(R) # [x, y, z, w]
                    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
                    t_wall = ts_ns * 1e-9 - t_wall_0
                    t_sim = get_sim_time(ts_ns * 1e-9) - sim_0
                    joint_records.append({
                        "t_sim": t_sim, "t_wall": t_wall,
                        "q": q, "pos": pos, "quat": quat_wxyz
                    })

    conn.close()

    return {
        "clock_records": clock_records,
        "target_records": target_records,
        "actual_records": actual_records if actual_records else joint_records,
        "joint_records": joint_records
    }


# ==============================================================================
# 4. CRITERI DI VALIDAZIONE E CALCOLO METRICHE CANONICHE
# ==============================================================================

def validate_run(target_records, clock_records, tau_nom, exclude_window_sec=5.0):
    """
    Verifica se la run è VALIDA secondo i criteri canonici:
    1. Frazione completata = 1.0 (t_sim_ultimo >= tau_nom)
    2. Nessun gap > 100 ms su /target_pose dopo i primi 5 s simulati
    3. Nessun buco > 3*dt nel /clock dopo i primi 5 s simulati
    """
    if not target_records:
        return False, "Nessun dato target_pose registrato."

    t_sim_ultimo = target_records[-1]["t_sim"]
    completed_frac = min(1.0, t_sim_ultimo / tau_nom)
    if completed_frac < 0.999:
        return False, f"Frazione completata insufficiente: {completed_frac:.3f} < 1.0 (t_sim_ultimo={t_sim_ultimo:.2f}s, tau={tau_nom:.2f}s)"

    # Gap su target_pose dopo exclude_window_sec
    t_tgts = np.array([r["t_sim"] for r in target_records])
    mask_post_5s = t_tgts >= exclude_window_sec
    if np.sum(mask_post_5s) > 1:
        dts_post = np.diff(t_tgts[mask_post_5s])
        max_gap_target = float(np.max(dts_post)) if len(dts_post) > 0 else 0.0
        if max_gap_target > 0.100: # 100 ms
            return False, f"Gap su target_pose > 100ms rilevato dopo {exclude_window_sec}s: max_gap = {max_gap_target*1000:.1f} ms"
    else:
        max_gap_target = 0.0

    # Buchi nel clock dopo exclude_window_sec
    if len(clock_records) > 2:
        clk_sims = np.array([c[1] for c in clock_records])
        clk_sims_rel = clk_sims - clk_sims[0]
        mask_clk = clk_sims_rel >= exclude_window_sec
        if np.sum(mask_clk) > 1:
            dt_clks = np.diff(clk_sims_rel[mask_clk])
            dt_med = np.median(dt_clks)
            clock_gaps = int(np.sum(dt_clks > 3.0 * dt_med))
            if clock_gaps > 0:
                return False, f"Buchi > 3*dt nel clock rilevati dopo {exclude_window_sec}s: {clock_gaps} buchi (dt_mediano={dt_med*1000:.2f}ms)"
        else:
            clock_gaps = 0
    else:
        clock_gaps = 0

    return True, "VALIDA"


def compute_metrics(target_records, actual_records, clock_records):
    """Calcola l'insieme completo delle metriche di inseguimento e di sistema."""
    if not target_records or not actual_records:
        return None

    t_tgt = np.array([r["t_sim"] for r in target_records])
    pos_tgt = np.array([r["pos"] for r in target_records])
    quat_tgt = np.array([r["quat"] for r in target_records])

    t_act = np.array([r["t_sim"] for r in actual_records])
    pos_act = np.array([r["pos"] for r in actual_records])
    quat_act = np.array([r["quat"] for r in actual_records])

    # Tau nominale: istante in cui il target si stabilizza entro 0.1 mm dall'ultimo
    dist_from_end = np.linalg.norm(pos_tgt - pos_tgt[-1], axis=1)
    settle_indices = np.where(dist_from_end > 1e-4)[0]
    tau_nom = float(t_tgt[settle_indices[-1] + 1]) if len(settle_indices) > 0 else float(t_tgt[-1])

    t_sim_ultimo = float(t_tgt[-1])
    completed_frac = min(1.0, t_sim_ultimo / tau_nom)

    # Gap su target_pose
    dts_tgt = np.diff(t_tgt)
    dt_mediano_tgt = float(np.median(dts_tgt)) if len(dts_tgt) > 0 else 0.0
    n_gaps_tgt_3dt = int(np.sum(dts_tgt > 3.0 * dt_mediano_tgt)) if len(dts_tgt) > 0 else 0
    max_gap_tgt_ms = float(np.max(dts_tgt) * 1000.0) if len(dts_tgt) > 0 else 0.0

    # Matching temporale campioni actual -> target
    pos_errs = []
    ori_errs = []
    times_matched = []

    for i, ta in enumerate(t_act):
        if ta < 0:
            continue
        idx = bisect.bisect_left(t_tgt, ta)
        if idx >= len(t_tgt):
            idx = len(t_tgt) - 1
        dp = np.linalg.norm(pos_act[i] - pos_tgt[idx]) * 1000.0 # mm

        q1 = quat_act[i] # [w, x, y, z]
        q2 = quat_tgt[idx]
        dot = abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3])
        dot = min(1.0, max(-1.0, dot))
        do = 2.0 * math.acos(dot) * 180.0 / math.pi # deg

        pos_errs.append(dp)
        ori_errs.append(do)
        times_matched.append(ta)

    pos_errs = np.array(pos_errs)
    ori_errs = np.array(ori_errs)
    times_matched = np.array(times_matched)

    pos_media = float(np.mean(pos_errs))
    pos_mediana = float(np.median(pos_errs))
    pos_p95 = float(np.percentile(pos_errs, 95))
    pos_max = float(np.max(pos_errs))
    t_max = float(times_matched[np.argmax(pos_errs)])

    mask_1s = times_matched >= 1.0
    pos_max_1s = float(np.max(pos_errs[mask_1s])) if np.any(mask_1s) else pos_max

    pos_finale_ultimo = float(pos_errs[-1])
    idx_tau = np.argmin(np.abs(times_matched - tau_nom))
    pos_finale_tau = float(pos_errs[idx_tau])

    ori_media = float(np.mean(ori_errs))
    ori_max = float(np.max(ori_errs))
    ori_finale = float(ori_errs[-1])

    # RTF
    if len(clock_records) > 2:
        t_sim_dur = clock_records[-1][1] - clock_records[0][1]
        t_wall_dur = clock_records[-1][0] - clock_records[0][0]
        rtf = float(t_sim_dur / t_wall_dur) if t_wall_dur > 0 else 1.0
    else:
        rtf = 1.0

    return {
        "tau_nom": tau_nom,
        "t_sim_ultimo": t_sim_ultimo,
        "completed_frac": completed_frac,
        "dt_mediano_tgt_ms": dt_mediano_tgt * 1000.0,
        "max_gap_tgt_ms": max_gap_tgt_ms,
        "n_gaps_tgt_3dt": n_gaps_tgt_3dt,
        "rtf": rtf,
        "pos_media_mm": pos_media,
        "pos_mediana_mm": pos_mediana,
        "pos_p95_mm": pos_p95,
        "pos_max_mm": pos_max,
        "pos_max_1s_mm": pos_max_1s,
        "pos_finale_ultimo_mm": pos_finale_ultimo,
        "pos_finale_tau_mm": pos_finale_tau,
        "ori_media_deg": ori_media,
        "ori_max_deg": ori_max,
        "ori_finale_deg": ori_finale,
        "num_samples": len(pos_errs)
    }
