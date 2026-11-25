#!/usr/bin/env python3
"""End-to-end del task layer in dry-run (nessun robot).

  test_task_e2e.py <build_dir> <hw_config.yaml> <weights.yaml (veri, di TEST)>

Casi:
 1) hw.py run --dry-run, omega -2 deg/s: HOLD_SAFE -> DONE, exit 0; errore di scatto e errore di fase al contatto
 2) idem con omega -3 deg/s (config temporanea)
 3) comando non fattibile: hw.py run rifiuta al plan, nessun processo avviato (exit 1)
 4) comando non fattibile con --no-plan-gate: real_replay si ferma in VALIDATE (FAULT_NOMOTION, exit 5) SENZA START
 5) satellite_node ucciso (SIGKILL) in WAIT_TRIGGER: real_replay FAULT_NOMOTION, resta vivo, poi esce (fault-linger)
 7) control loop FERMO (--hang-loop-after-s, thread vivi, nessuna eccezione): il heartbeat si interrompe e satellite_node
    si ferma da solo dopo T_hb (rampa, faults heartbeat, exit 3)
 6) stadio 3 con FakeGripper (gripper.enabled, config _ff_dry): apertura, APPROACH, CLOSE_FOLLOW, SQUEEZE, HOLD, RELEASE
    (STOP a rampa, apertura solo dopo STOPPED), HOLD_SAFE, DONE, exit 0
Porte e file temporanei: non disturbano un satellite_node vero.
"""

import hashlib
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import yaml

HW_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HW_PY = os.path.join(HW_DIR, "hw.py")
PORTS = (27601, 27602, 27603)


def fail(msg):
    print("FAIL: " + msg)
    sys.exit(1)


def make_config(src, tmp, omega_deg=None, grasp=False, rr_dry=None):
    with open(src) as f:
        c = yaml.safe_load(f)
    d = os.path.dirname(os.path.abspath(src))
    ab = lambda p: p if os.path.isabs(p) else os.path.normpath(os.path.join(d, p))
    c["robot2"]["real_replay_config"] = ab(c["robot2"]["real_replay_config"])
    c["robot1"]["real_replay_config"] = ab(c["robot1"]["real_replay_config"])
    c["robot1"]["real_replay_config_dry"] = ab(c["robot1"]["real_replay_config_dry"])
    for k in ("libfranka_examples_dir", "build_dir", "log_dir"):
        c["paths"][k] = ab(c["paths"][k])
    for k in ("grasp_command_yaml", "weights_yaml"):
        c["planner"][k] = ab(c["planner"][k])
    c["udp"]["publish_port"], c["udp"]["command_port"], c["udp"]["monitor_port"] = PORTS
    if grasp:
        c.setdefault("gripper", {})["enabled"] = True
    if rr_dry:
        c["robot1"]["real_replay_config_dry"] = rr_dry
    if omega_deg is not None:
        c["scene"]["omega_deg_s"] = omega_deg
    path = os.path.join(tmp, "hw_config_%s.yaml" % ("nom" if omega_deg is None else str(omega_deg)))
    with open(path, "w") as f:
        yaml.safe_dump(c, f, sort_keys=False)
    return path


def make_command(weights, tmp, theta_star, omega, name, start_offset=(0.0, 0.0, 0.0)):
    with open(weights) as f:
        tau = float(yaml.safe_load(f)["tau"])
    with open(weights, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    sp = [0.000002 + start_offset[0], 0.432494 + start_offset[1], 0.772246 + start_offset[2]]  # TCP di Q_MEAS
    q = [0.494585, -0.505353, -0.505353, -0.494593]
    gp = [0.000002 + 0.1168, 0.432494 + 0.0238, 0.772246 + 0.0025]  # start + spostamento della demo
    cmd = dict(format="grasp_command_v2", status="SELECTED", frame_id="world", stamp_unix_s=time.time(), theta_star_deg=theta_star,
               contact_time_s=tau, omega_deg_s=omega, weights_sha256=sha, start_pose=dict(position_m=sp, quaternion_wxyz=q),
               goal_pose=dict(position_m=gp, quaternion_wxyz=q), k=0, wrist_flip=False)
    path = os.path.join(tmp, name)
    with open(path, "w") as f:
        yaml.safe_dump(cmd, f, sort_keys=False)
    return path


def hw_run(cfg, cmd, weights, extra=(), timeout=180):
    argv = [sys.executable, HW_PY, "--config", cfg, "run", "--dry-run", "--grasp-command", cmd, "--weights", weights,
            "--fault-linger-s", "1"] + list(extra)
    t0 = time.monotonic()
    r = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
    return r.returncode, r.stdout, time.monotonic() - t0


def figure(out, pattern):
    m = re.search(pattern, out)
    return float(m.group(1)) if m else float("nan")


def nominal_case(name, cfg, cmd, weights, omega):
    print("== %s ==" % name)
    rc, out, dt = hw_run(cfg, cmd, weights)
    lines = [l for l in out.splitlines() if re.search(r"TRIGGER|PHASE ERROR|duration trigger|implied|final state|exit|START sent|omega_valid", l)]
    print("\n".join(lines))
    if rc != 0:
        print(out[-4000:])
        fail("%s: hw.py run exit %d" % (name, rc))
    if "final state: DONE" not in out:
        fail("%s: real_replay non arrivato a DONE" % name)
    e_trig = figure(out, r"TRIGGER ERROR \(theta_trig - theta measured at the trigger\): ([-+0-9.]+) deg")
    e_ph = figure(out, r"PHASE ERROR AT CONTACT = theta\(end of approach\) - theta\*: ([-+0-9.]+) deg")
    dur = figure(out, r"duration trigger -> end of APPROACH: ([0-9.]+) s")
    tol_trig = abs(omega) * 0.015  # un campione a 100 Hz + latenza del thread del task
    print("%s: errore di scatto %+.4f deg (tol %.3f), durata trigger->fine approach %.4f s, errore di fase al contatto %+.4f deg (%.1f s)"
          % (name, e_trig, tol_trig, dur, e_ph, dt))
    if not abs(e_trig) <= tol_trig:
        fail("%s: errore di scatto fuori tolleranza" % name)
    if not abs(e_ph) <= abs(omega) * 0.05:
        fail("%s: errore di fase al contatto %.4f deg oltre |omega|*50 ms" % (name, e_ph))
    print("OK")
    return e_trig, dur, e_ph


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    build, cfg_src, weights = (os.path.abspath(a) for a in sys.argv[1:])
    tmp = tempfile.mkdtemp(prefix="task_e2e_")
    cfg2 = make_config(cfg_src, tmp)
    cfg3 = make_config(cfg_src, tmp, omega_deg=-3.0)

    r1 = nominal_case("1) omega -2 deg/s, theta* -90", cfg2, make_command(weights, tmp, -90.0, -2.0, "c1.yaml"), weights, -2.0)
    r2 = nominal_case("2) omega -3 deg/s, theta* -120", cfg3, make_command(weights, tmp, -120.0, -3.0, "c2.yaml"), weights, -3.0)

    print("== 3) non fattibile (theta* -60: theta_trig -11.05, theta0 -10): rifiuto al plan ==")
    c3 = make_command(weights, tmp, -60.0, -2.0, "c3.yaml")
    rc, out, _ = hw_run(cfg2, c3, weights)
    if rc != 1 or "plan NON fattibile: non avvio nulla" not in out or "satellite_node --config" in out:
        print(out[-3000:])
        fail("3) atteso exit 1 senza avviare processi (rc %d)" % rc)
    print("OK: exit 1, nessun processo avviato")

    print("== 4) non fattibile con --no-plan-gate: VALIDATE di real_replay, nessuno START ==")
    rc, out, _ = hw_run(cfg2, c3, weights, extra=["--no-plan-gate"])
    print("\n".join(l for l in out.splitlines() if re.search(r"VALIDATE failed|FAULT_NOMOTION|START sent|STOP sent|exit", l)))
    if rc != 5 or "START sent" in out or "FAULT_NOMOTION" not in out:
        print(out[-4000:])
        fail("4) atteso exit 5 (FAULT_NOMOTION) senza START (rc %d)" % rc)
    if not re.search(r"satellite_node: exit 0", out):
        fail("4) satellite_node doveva fermarsi in quiete (STOP in WAIT_START, exit 0)")
    print("OK: FAULT_NOMOTION in VALIDATE, START mai inviato, satellite fermo")

    print("== 5) satellite_node ucciso in WAIT_TRIGGER ==")
    c5 = make_command(weights, tmp, -90.0, -2.0, "c5.yaml")
    with open(cfg2) as f:
        hc = yaml.safe_load(f)
    sat = subprocess.Popen([os.path.join(build, "satellite_node"), "--config", cfg2, "--dry-run"], stdout=subprocess.DEVNULL,
                           stderr=subprocess.STDOUT)
    time.sleep(1.0)
    rr = subprocess.Popen([os.path.join(build, "real_replay"), "--config", hc["robot1"]["real_replay_config_dry"], "--mode", "task", "--dry-run",
                           "--grasp-command", c5, "--hw-config", cfg2, "--weights", weights, "--yes", "--fault-linger-s", "4"],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = ""
    t_end = time.monotonic() + 30
    killed_at = None
    while time.monotonic() < t_end:
        line = rr.stdout.readline()
        if not line:
            break
        out += line
        if killed_at is None and "START sent" in line:
            time.sleep(1.0)
            sat.send_signal(signal.SIGKILL)
            killed_at = time.monotonic()
        if "FAULT_NOMOTION" in line and killed_at is not None:
            t_fault = time.monotonic()
            break
    else:
        t_fault = None
    alive_after_fault = rr.poll() is None
    time.sleep(2.0)
    alive_2s = rr.poll() is None
    # keep reading with readline(): communicate() would read the raw pipe and lose the lines already buffered
    for line in rr.stdout:
        out += line
    rr.wait(timeout=60)
    sat.wait()
    print("\n".join(l for l in out.splitlines() if re.search(r"FAULT|STOP sent|linger|final state", l)))
    if killed_at is None or t_fault is None:
        fail("5) START o FAULT non visti")
    print("fault %.2f s dopo il SIGKILL (sat_timeout 2 s); vivo subito dopo il guasto: %s, ancora vivo 2 s dopo: %s; exit %d"
          % (t_fault - killed_at, alive_after_fault, alive_2s, rr.returncode))
    if not (1.9 <= t_fault - killed_at <= 3.0 and alive_after_fault and alive_2s and rr.returncode == 5 and "STOP sent" in out):
        fail("5) atteso FAULT_NOMOTION ~2 s dopo, processo vivo, STOP inviato, exit 5")
    print("OK")

    print("== 6) stadio 3 (FakeGripper): fino a RELEASE ==")
    rr_ff_dry = os.path.join(os.path.dirname(HW_DIR), "config", "task_robot1_hand_ff_dry.yaml")
    cfg6 = make_config(cfg_src, tmp, grasp=True, rr_dry=rr_ff_dry)
    rc, out, dt = hw_run(cfg6, make_command(weights, tmp, -90.0, -2.0, "c6.yaml"), weights, timeout=240)
    print("\n".join(l for l in out.splitlines() if re.search(r"state (CLOSE|SQUEEZE|HOLD|RELEASE|DONE)|gripper|STOPPED|RELEASE:|GRASP:|final state|STOP", l)))
    order = ["state APPROACH", "gripper: grasp commanded", "state CLOSE_FOLLOW", "gripper: grasp OK", "state SQUEEZE", "state HOLD", "state RELEASE",
             "STOP_RAMP sent", "satellite STOPPED", "gripper: released", "state HOLD_SAFE", "state DONE"]
    pos = [out.find(k) for k in order]
    if rc != 0 or "final state: DONE" not in out or min(pos) < 0 or pos != sorted(pos):
        print(out[-5000:])
        fail("6) atteso DONE exit 0 con la sequenza %s (rc %d, posizioni %s)" % (order, rc, pos))
    print("OK: %.0f s" % dt)

    print("== 7) control loop fermo: heartbeat interrotto, satellite_node si ferma da solo ==")
    c7 = make_command(weights, tmp, -90.0, -2.0, "c7.yaml")
    sat = subprocess.Popen([os.path.join(build, "satellite_node"), "--config", cfg2, "--dry-run"], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)
    time.sleep(1.0)
    rr = subprocess.Popen([os.path.join(build, "real_replay"), "--config", hc["robot1"]["real_replay_config_dry"], "--mode", "task", "--dry-run",
                           "--grasp-command", c7, "--hw-config", cfg2, "--weights", weights, "--yes", "--fault-linger-s", "2",
                           "--hang-loop-after-s", "9"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t_hang = None
    out7 = ""
    t_end = time.monotonic() + 30
    while time.monotonic() < t_end:
        line = rr.stdout.readline()
        if not line:
            break
        out7 += line
        if "control loop hung on purpose" in line:
            t_hang = time.monotonic()
            break
    if t_hang is None:
        sat.kill()
        rr.kill()
        print(out7[-3000:])
        fail("7) il loop non e' stato fermato (o START non inviato)")
    try:
        sat_out = sat.communicate(timeout=20)[0]
    except subprocess.TimeoutExpired:
        sat.kill()
        rr.kill()
        fail("7) satellite_node non si e' fermato da solo dopo che il loop si e' fermato")
    t_sat_exit = time.monotonic()
    rc_sat = sat.returncode
    alive = rr.poll() is None  # real_replay (loop fermo) e' ancora vivo: il processo NON ha mandato nulla di suo
    rr.send_signal(signal.SIGINT)
    try:
        rr.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        rr.kill()
    stop_line = [l for l in sat_out.splitlines() if "stop reason" in l]
    print("loop fermo; satellite_node %.1f s dopo: exit %d; %s; real_replay vivo: %s" % (t_sat_exit - t_hang, rc_sat, stop_line, alive))
    if rc_sat != 3 or not any("heartbeat_timeout" in l for l in stop_line):
        print(sat_out[-2000:])
        fail("7) atteso exit 3 con 'heartbeat_timeout' (rc %d)" % rc_sat)
    # rampa di stop (2 s) + hold (1 s) dopo il timeout T_hb 0.5 s + stall 0.1 s
    if not 0.5 <= t_sat_exit - t_hang <= 8.0:
        fail("7) tempi fuori scala: %.1f s" % (t_sat_exit - t_hang))
    print("OK")

    print("\nRIEPILOGO dry-run: omega -2: scatto %+.4f deg, trigger->fine %.4f s, fase al contatto %+.4f deg | "
          "omega -3: scatto %+.4f deg, trigger->fine %.4f s, fase al contatto %+.4f deg" % (r1 + r2))
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
