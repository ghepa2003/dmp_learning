#!/usr/bin/env python3
"""Test end-to-end del satellite_node in dry-run (nessun robot).

  test_satellite_e2e.py <satellite_node> <hw_config.yaml> <log_dir>

1) --dry-run --autostart, corsa 34 deg (~20 s): un ascoltatore UDP verifica che omega_meas, a regime (finestra di
   regressione tutta dentro la fase a velocita' costante), sia entro +-1 % di scene.omega_deg_s; controlla anche seq,
   frequenza, stato finale DONE ed exit code 0.
2) senza --autostart: START via UDP, un solo heartbeat, poi silenzio -> timeout heartbeat -> rampa -> exit 3 con il
   bit 'heartbeat' nei faults.
3) protocollo v3, STOP a rampa: START, regime, poi (a) STOP_RAMP: STOPPING dura stop_ramp_s, poi HOLD, exit 0;
   (b) STOP duro: dura stop_hard_ramp_s; (c) STOP_RAMP e dopo 0.5 s STOP duro: la rampa viene accorciata (ripartenza
   dalla fase e velocita' correnti, nessun salto di omega_cmd oltre la rampa).
Porte alternative (non quelle della config) per non disturbare un satellite_node vero.
"""

import math
import os
import socket
import subprocess
import sys
import time

HW_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HW_DIR)
import hw  # noqa: E402

PUB, CMD = 25601, 25602


def fail(msg):
    print("FAIL: " + msg)
    sys.exit(1)


def receiver():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", PUB))
    s.settimeout(0.05)
    return s


def collect(sock, proc, packets, until=None):
    while proc.poll() is None:
        try:
            p = hw.decode_state(sock.recv(512))
        except socket.timeout:
            p = None
        if p is not None:
            p["t_rx"] = time.monotonic()
            packets.append(p)
            if until is not None and until(p):
                return
    # coda dopo l'uscita
    t_end = time.monotonic() + 0.3
    while time.monotonic() < t_end:
        try:
            p = hw.decode_state(sock.recv(512))
            if p is not None:
                packets.append(p)
        except socket.timeout:
            pass


def test_autostart(node, cfg_path, log_dir, omega_deg):
    print("== 1) dry-run --autostart, corsa 34 deg ==")
    sock = receiver()
    argv = [node, "--config", cfg_path, "--dry-run", "--autostart", "--max-travel-deg", "34", "--pub-port", str(PUB),
            "--monitor-port", "0", "--cmd-port", str(CMD), "--log-dir", log_dir, "--log-tag", "e2e"]
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    packets = []
    collect(sock, proc, packets)
    out = proc.stdout.read()
    rc = proc.wait()
    sock.close()
    print("\n".join(l for l in out.splitlines() if "omega" in l or "stop reason" in l or "log written" in l or "FAIL" in l))
    if rc != 0:
        print(out)
        fail("exit code %d (atteso 0)" % rc)
    if len(packets) < 1500:
        fail("solo %d pacchetti" % len(packets))
    seqs = [p["seq"] for p in packets]
    if any(b <= a for a, b in zip(seqs, seqs[1:])):
        fail("seq non strettamente crescente")
    lost = seqs[-1] - seqs[0] + 1 - len(seqs)
    span = packets[-1]["t_rx"] - packets[0]["t_rx"]
    rate = (len(packets) - 1) / span
    print("pacchetti %d, persi %d, frequenza %.1f Hz" % (len(packets), lost, rate))
    if not 90.0 <= rate <= 110.0:
        fail("frequenza %.1f Hz fuori da [90, 110]" % rate)
    if packets[-1]["state"] != 5:
        fail("stato finale %s (atteso DONE)" % packets[-1]["state"])
    w = omega_deg * hw.DEG
    const = [p for p in packets if p["state"] == 2 and abs(p["omega_cmd"] - w) < 1e-9]
    if not const:
        fail("nessun pacchetto nella fase a velocita' costante")
    t0 = const[0]["t_mono_ns"] * 1e-9
    t1 = const[-1]["t_mono_ns"] * 1e-9
    # omega_meas si usa SOLO dove il nodo dichiara omega_valid
    steady = [p for p in packets if p["omega_valid"]]
    print("fase costante %.2f s, pacchetti con omega_valid %d" % (t1 - t0, len(steady)))
    if len(steady) < 800:
        fail("troppo pochi pacchetti con omega_valid (attesi ~1000 = 10 s)")
    if any(p["state"] != 2 for p in steady):
        fail("omega_valid vero fuori da RUNNING")
    if any(math.isnan(p["omega_meas"]) for p in steady):
        fail("omega_valid vero con omega_meas NaN")
    if any(abs(p["omega_cmd"] - w) > 1e-9 for p in steady):
        fail("omega_valid vero durante una rampa")
    first_valid = steady[0]["t_mono_ns"] * 1e-9
    run0 = next(p for p in packets if p["state"] == 2)["t_mono_ns"] * 1e-9
    print("omega_valid dopo %.3f s da START (atteso ramp_s + omega_window_s = 7 s)" % (first_valid - run0))
    if not 6.98 <= first_valid - run0 <= 7.1:
        fail("omega_valid diventa vero dopo %.3f s" % (first_valid - run0))
    errs = [(p["omega_meas"] - w) / abs(w) for p in steady]
    print("omega_meas a regime: min %.5f max %.5f deg/s (impostata %.3f) -> errore relativo max %.4f %%"
          % (min(p["omega_meas"] for p in steady) / hw.DEG, max(p["omega_meas"] for p in steady) / hw.DEG, omega_deg,
             100 * max(abs(e) for e in errs)))
    if max(abs(e) for e in errs) > 0.01:
        fail("omega_meas fuori da +-1 %")
    # stima indipendente dell'ascoltatore: pendenza LS di theta vs t_mono (informativa)
    ts = [p["t_mono_ns"] * 1e-9 for p in steady]
    th = [p["theta"] for p in steady]
    tm, xm = sum(ts) / len(ts), sum(th) / len(th)
    slope = sum((a - tm) * (b - xm) for a, b in zip(ts, th)) / sum((a - tm) ** 2 for a in ts)
    print("pendenza theta(t_mono) dell'ascoltatore: %.4f deg/s (%.3f %%; tempo di parete, include la deriva del pacing)"
          % (slope / hw.DEG, 100 * (slope - w) / abs(w)))
    if any(p["omega_valid"] for p in packets if p["state"] in (0, 1, 3, 4, 5)):
        fail("omega_valid vero in WAIT_START/STOPPING/HOLD/DONE")
    print("OK")


def test_heartbeat(node, cfg_path, log_dir):
    print("== 2) START + un heartbeat, poi silenzio: timeout heartbeat ==")
    sock = receiver()
    argv = [node, "--config", cfg_path, "--dry-run", "--max-travel-deg", "20", "--pub-port", str(PUB), "--monitor-port", "0",
            "--cmd-port", str(CMD), "--log-dir", log_dir, "--log-tag", "e2e_hb"]
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    packets = []
    collect(sock, proc, packets, until=lambda p: p["state"] == 1)
    if not packets or packets[-1]["state"] != 1:
        proc.kill()
        fail("WAIT_START non visto")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(hw.encode_command(hw.CMD_START, 1), ("127.0.0.1", CMD))
    time.sleep(0.2)
    tx.sendto(hw.encode_command(hw.CMD_HEARTBEAT, 2), ("127.0.0.1", CMD))
    t_last_hb = time.monotonic()
    collect(sock, proc, packets)
    out = proc.stdout.read()
    rc = proc.wait()
    sock.close()
    stopping = [p for p in packets if p["state"] == 3]
    print("\n".join(l for l in out.splitlines() if "stop reason" in l))
    if rc != 3:
        print(out)
        fail("exit code %d (atteso 3)" % rc)
    if not packets[-1]["faults"] & (1 << 1):
        fail("bit heartbeat non presente nei faults (0x%x)" % packets[-1]["faults"])
    if not stopping:
        fail("nessuna rampa di stop (STOPPING) osservata")
    if any(p["omega_valid"] for p in packets):
        fail("omega_valid vero in un run fermato prima di ramp_s + omega_window_s")
    dt = stopping[0]["t_rx"] - t_last_hb
    print("STOPPING iniziato %.3f s dopo l'ultimo heartbeat (T_hb 0.5 s); faults finali %s" % (dt, hw.faults_str(packets[-1]["faults"])))
    if not 0.45 <= dt <= 0.8:
        fail("timeout heartbeat dopo %.3f s" % dt)
    print("OK")


def test_stop_types(node, cfg_path, log_dir, cfg):
    ramp = float(cfg["robot2"]["control"]["stop_ramp_s"])
    hard = float(cfg["robot2"]["control"].get("stop_hard_ramp_s", 0.5))
    omega = float(cfg["scene"]["omega_deg_s"]) * hw.DEG
    for name, seq in (("a) STOP_RAMP", [(hw.CMD_STOP_RAMP, 0.0)]), ("b) STOP duro", [(hw.CMD_STOP, 0.0)]),
                      ("c) STOP_RAMP poi STOP duro dopo 0.5 s", [(hw.CMD_STOP_RAMP, 0.0), (hw.CMD_STOP, 0.5)])):
        print("== 3%s ==" % name)
        sock = receiver()
        argv = [node, "--config", cfg_path, "--dry-run", "--max-travel-deg", "40", "--pub-port", str(PUB), "--monitor-port", "0",
                "--cmd-port", str(CMD), "--log-dir", log_dir, "--log-tag", "e2e_stop"]
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        packets = []
        collect(sock, proc, packets, until=lambda p: p["state"] == 1)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        n = [1]

        def send(t):
            n[0] += 1
            tx.sendto(hw.encode_command(t, n[0]), ("127.0.0.1", CMD))

        send(hw.CMD_START)
        t_start = time.monotonic()
        last_hb = [0.0]
        pending = list(seq)
        t_cmd = [None]
        t_first = [None]

        def tick(p):
            now = time.monotonic()
            if now - last_hb[0] > 0.1:
                send(hw.CMD_HEARTBEAT)
                last_hb[0] = now
            if now - t_start > 5.0 and pending:  # regime (rampa di partenza 2 s)
                if t_first[0] is None:
                    t_first[0] = now
                typ, delay = pending[0]
                if now - t_first[0] >= delay:
                    send(typ)
                    if t_cmd[0] is None:
                        t_cmd[0] = now
                    pending.pop(0)
            return False

        t_end = time.monotonic() + 40.0
        while proc.poll() is None and time.monotonic() < t_end:
            try:
                p = hw.decode_state(sock.recv(512))
            except socket.timeout:
                p = None
            if p is not None:
                p["t_rx"] = time.monotonic()
                packets.append(p)
            tick(p)
        out = proc.stdout.read()
        rc = proc.wait()
        sock.close()
        stopping = [p for p in packets if p["state"] == 3]
        holds = [p for p in packets if p["state"] in (4, 5)]
        if rc != 0 or not stopping or not holds:
            print(out[-2000:])
            fail("3%s: exit %d, STOPPING %d pacchetti, HOLD %d" % (name, rc, len(stopping), len(holds)))
        dur = holds[0]["t_rx"] - stopping[0]["t_rx"]
        t_stop_from_cmd = holds[0]["t_rx"] - t_cmd[0]
        om = [p["omega_cmd"] for p in stopping]
        mono = all(abs(b) <= abs(a) + 1e-9 for a, b in zip(om, om[1:]))
        print("%s: STOPPING->HOLD %.3f s (dal comando %.3f s), omega_cmd monotona verso 0: %s, ultimo omega_cmd %.2e rad/s, exit %d"
              % (name, dur, t_stop_from_cmd, mono, om[-1], rc))
        if len(seq) == 1:
            want = ramp if seq[0][0] == hw.CMD_STOP_RAMP else hard
            if not want - 0.15 <= dur <= want + 0.25:
                fail("3%s: durata %.3f s, attesa %.2f s" % (name, dur, want))
        else:
            want = 0.5 + hard
            if not want - 0.2 <= t_stop_from_cmd <= want + 0.3:
                fail("3%s: dal primo comando %.3f s, atteso ~%.2f s (rampa accorciata)" % (name, t_stop_from_cmd, want))
        if not mono or abs(om[0]) > abs(omega) + 1e-9:
            fail("3%s: omega_cmd non monotona durante la rampa" % name)
        if not packets[-1]["faults"] & (1 << 5):
            fail("3%s: bit stop_command assente nei faults" % name)
    print("OK")


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    node, cfg_path, log_dir = sys.argv[1:]
    cfg = hw.load_config(cfg_path)
    test_autostart(node, cfg_path, log_dir, float(cfg["scene"]["omega_deg_s"]))
    test_heartbeat(node, cfg_path, log_dir)
    test_stop_types(node, cfg_path, log_dir, cfg)
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
