#!/usr/bin/env python3
"""Summary table of real_replay --mode task runs (stage 2 / stage 3) from their log files. Read only.

usage:
  tools/analyze_run.py RUN [RUN ...] [--log-dir DIR] [--sat SAT.csv] [--out-md F.md] [--out-csv F.csv]

RUN is either the prefix of the task logs (".../20261126_093401_669_task_robot") or a short id that identifies it in
--log-dir ("093401", "20261126_093401"). For every run the script reads:
  <prefix>.csv             robot 1 1 kHz log (q, p_des/p_meas, flags; column t = loop time)
  <prefix>_task.csv        task log, one row per 1 kHz cycle (t_mono_s, task_state, theta_deg, sample_age_ms, e_pos_m, e_rot_rad)
  <prefix>_events.csv      task events (t_mono_s)
  <prefix>.csv.meta.txt    config / git of real_replay
  satellite_node CSV + meta: the latest *_satellite_node_<kind>.csv started within 60 s BEFORE the task (or --sat)
  GraspCommand: path from the "state VALIDATE" event (/root/thesis_ws mapped to this repository)

Clocks:
  - events and task CSV: CLOCK_MONOTONIC of the PC (task thread / 1 kHz callback);
  - robot CSV row i <-> task CSV row i (both written once per 1 kHz cycle; checked: same number of rows);
  - satellite CSV column t = robot-2 control time. It is mapped to CLOCK_MONOTONIC by matching the satellite phase:
    each datagram seen by the task (stamp = t_mono - sample_age, theta) is located in the satellite CSV by theta
    (constant-speed segment, theta strictly monotone) and t_mono = a + b * t_sat is fitted (residual printed).
Phase at contact:
  - "task": theta of the latest datagram when the task thread saw the end of the approach, minus theta* (= SUMMARY);
  - "sat":  theta_flange of the satellite CSV at t_mono = t(TRIGGER event) + tau, minus theta*.
Exit code is derived from the final state (DONE 0, FAULT_NOMOTION 5, FAULT_MOTION 6, FAULT_GRASP 8): a libfranka
exception (exit 3) is not visible in these files.
"""
import argparse
import csv
import glob
import hashlib
import math
import os
import re
import sys
from datetime import datetime

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOG_DIR = os.path.join(REPO, "real_replay", "build", "logs")
CONTAINER_ROOT = "/root/thesis_ws"

STATES = {0: "INIT", 1: "WAIT_CMD", 2: "VALIDATE", 3: "WAIT_TRIGGER", 4: "APPROACH", 5: "HOLD_SAFE", 6: "CLOSE_FOLLOW",
          7: "SQUEEZE", 8: "RELEASE", 9: "FAULT_NOMOTION", 10: "FAULT_MOTION", 11: "FAULT_GRASP", 12: "DONE", 13: "HOLD"}
EXIT = {"DONE": 0, "FAULT_NOMOTION": 5, "FAULT_MOTION": 6, "FAULT_GRASP": 8}
FOLLOW_STATES = {6, 7, 13, 8}  # CLOSE_FOLLOW, SQUEEZE, HOLD, RELEASE
FAULT_STATES = {9, 10, 11}
PHASE_GROUPS = [("wait_trigger", {3}), ("approach", {4}), ("follow", FOLLOW_STATES), ("hold_safe", {5}), ("fault", FAULT_STATES)]
SAT_FAULT_BITS = ["window", "heartbeat", "tracking", "safety", "phase_mismatch", "stop_command", "signal", "non_finite",
                  "wait_timeout", "log_full"]
# Franka Panda nominal joint limits [rad] (datasheet; the same as the URDF limits)
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
NAN = float("nan")


# ---- input -------------------------------------------------------------------------------------------------------
def local_path(p):
    if p and p.startswith(CONTAINER_ROOT):
        return REPO + p[len(CONTAINER_ROOT):]
    return p


def read_csv(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        rows = [row for row in r if row]
    return head, rows


def columns(path, names):
    head, rows = read_csv(path)
    out = {}
    for n in names:
        if n not in head:
            continue
        k = head.index(n)
        out[n] = np.array([float(row[k]) if row[k] not in ("", "nan") else NAN for row in rows])
    return out, head


def read_meta(path):
    d = {}
    if not os.path.exists(path):
        return d
    with open(path) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                d[k.strip()] = v.strip()
    return d


def read_events(path):
    ev = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            if len(row) >= 2:
                ev.append((float(row[0]), row[1], row[2] if len(row) > 2 else ""))
    return ev


def first(ev, name, startswith=False):
    for t, n, d in ev:
        if (n.startswith(name) if startswith else n == name):
            return t, d
    return None, None


def num_after(text, key):
    m = re.search(re.escape(key) + r"\s*(-?[0-9.]+(?:e-?[0-9]+)?)", text or "")
    return float(m.group(1)) if m else NAN


def resolve_prefix(run, log_dir):
    if run.endswith("_task_robot") or run.endswith("_task_dryrun"):
        return run
    cands = sorted(glob.glob(os.path.join(log_dir, "*%s*_task_*_events.csv" % run)))
    cands = [c for c in cands if re.search(r"_task_(robot|dryrun)_events\.csv$", c)]
    if len(cands) != 1:
        raise SystemExit("run '%s': %d matches in %s %s" % (run, len(cands), log_dir, [os.path.basename(c) for c in cands]))
    return cands[0][: -len("_events.csv")]


def stamp_of(name):
    m = re.match(r"(\d{8}_\d{6})_(\d{3})_", os.path.basename(name))
    if not m:
        return None
    return datetime.strptime(m.group(1), "%Y%m%d_%H%M%S").timestamp() + int(m.group(2)) / 1000.0


def find_satellite(prefix):
    kind = "robot" if prefix.endswith("_robot") else "dryrun"
    t_task = stamp_of(prefix)
    best = None
    for c in glob.glob(os.path.join(os.path.dirname(prefix), "*_satellite_node_%s.csv" % kind)):
        ts = stamp_of(c)
        if ts is not None and t_task is not None and 0.0 <= t_task - ts <= 60.0:
            if best is None or ts > stamp_of(best):
                best = c
    return best


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


# ---- analysis ----------------------------------------------------------------------------------------------------
def sat_clock_fit(task, sat):
    """t_mono = a + b * t_sat from the datagrams seen by the task (theta matching on the constant-speed segment)."""
    stamp = task["t_mono_s"] - task["sample_age_ms"] * 1e-3
    th = task["theta_deg"]
    ok = np.isfinite(stamp) & np.isfinite(th)
    stamp, th = stamp[ok], th[ok]
    _, idx = np.unique(np.round(stamp, 6), return_index=True)
    stamp, th = stamp[idx], th[idx]
    w = sat["omega_cmd_deg_s"]
    w_nom = np.nanmedian(w[np.abs(w) > 1e-6]) if np.any(np.abs(w) > 1e-6) else NAN
    if not math.isfinite(w_nom):
        return None
    seg = (sat["state"] == 2) & (np.abs(w - w_nom) < 1e-6)
    if seg.sum() < 1000:
        return None
    ts, thf = sat["t"][seg], sat["theta_flange_deg"][seg]
    s = np.sign(w_nom)
    if not np.all(np.diff(s * thf) >= -1e-3):  # must be monotone (measured phase): else no unique match
        # keep the strictly monotone envelope
        keep = np.r_[True, np.diff(np.maximum.accumulate(s * thf)) > 0]
        ts, thf = ts[keep], thf[keep]
    lo, hi = np.min(thf), np.max(thf)
    m = (th > lo + 0.05) & (th < hi - 0.05)
    if m.sum() < 50:
        return None
    t_sat = np.interp(s * th[m], s * thf, ts)
    b, a = np.polyfit(t_sat, stamp[m], 1)
    res = stamp[m] - (a + b * t_sat)
    return {"a": a, "b": b, "resid_ms": float(np.std(res) * 1e3), "n": int(m.sum()), "omega_nom": float(w_nom)}


# ---- jitter ------------------------------------------------------------------------------------------------------
JITTER_EVENTS = ("START sent", "omega_valid", "TRIGGER", "APPROACH", "gripper", "STOP", "satellite STOPPED", "FOLLOW", "HOLD",
                 "control loop stop", "Ctrl-C", "state ")
PERIOD_THRESH_MS = (1.5, 2.0, 3.0, 4.0)


def max_run(mask):
    best = cur = 0
    for v in mask:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return best


def episodes(mask, t):
    """[(t_first, t_last, n_cycles)] of the contiguous runs of mask."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((t[i], t[j], j - i + 1))
            i = j + 1
        else:
            i += 1
    return out


def jitter(tag, t_mono, period_s, sr, t_start, task_t, task_st, ev, grace_cycles=500):
    """Jitter statistics of one 1 kHz loop. Times are reported in seconds from START (CLOCK_MONOTONIC)."""
    R, detail = {}, []
    per = period_s * 1e3
    ok = np.isfinite(per)
    for th in PERIOD_THRESH_MS:
        R["%s_cycles_period_gt_%gms" % (tag, th)] = int(np.sum(per[ok] > th + 1e-9))
    i = int(np.nanargmax(per))
    rel = lambda tm: tm - t_start if t_start is not None else tm
    state_at = lambda tm: STATES.get(int(task_st[min(np.searchsorted(task_t, tm), len(task_st) - 1)]), "?")
    R["%s_period_max_ms" % tag] = float(per[i])
    R["%s_period_max_at" % tag] = "%+.3f s (%s)" % (rel(t_mono[i]), state_at(t_mono[i]))
    R["%s_max_consecutive_gt_3ms" % tag] = max_run(per > 3.0 + 1e-9)
    srm = np.where(np.arange(len(sr)) >= grace_cycles, sr, np.nan)  # the guard ignores the first grace cycles
    for th in (0.95, 0.9):
        R["%s_cycles_sr_lt_%g" % (tag, th)] = int(np.sum(srm < th))
        ep = episodes(srm < th, t_mono)
        R["%s_sr_lt_%g_episodes" % (tag, th)] = len(ep)
    R["%s_sr_min" % tag] = float(np.nanmin(srm)) if np.isfinite(srm).any() else NAN
    R["%s_max_consecutive_sr_lt_0.9" % tag] = max_run(srm < 0.9)
    # where the peaks fall: nearest notable event (+-0.5 s) and distance to the 1 Hz sat_link printf of the task thread
    notable = [(t, n) for t, n, d in ev if n.startswith(JITTER_EVENTS)]
    links = np.array([t for t, n, d in ev if n == "sat_link"])
    near_ev, near_link = 0, 0
    peaks = np.where(per > 3.0 + 1e-9)[0]
    for k in peaks:
        tm = t_mono[k]
        best = min(notable, key=lambda e: abs(e[0] - tm)) if notable else None
        evs = "%s %+.0f ms" % (best[1], (tm - best[0]) * 1e3) if best and abs(best[0] - tm) <= 0.5 else "-"
        if evs != "-":
            near_ev += 1
        dl = float(np.min(np.abs(links - tm))) * 1e3 if len(links) else NAN
        if math.isfinite(dl) and dl <= 20.0:
            near_link += 1
        detail.append("%s period %.0f ms at %+.3f s (%s); nearest event within 0.5 s: %s; nearest sat_link %.0f ms" %
                      (tag, per[k], rel(tm), state_at(tm), evs, dl))
    for a, b, n in episodes(srm < 0.95, t_mono):
        detail.append("%s success_rate < 0.95 from %+.3f to %+.3f s (%d cycles, %s), min %.2f" %
                      (tag, rel(a), rel(b), n, state_at(a), float(np.nanmin(srm[(t_mono >= a) & (t_mono <= b)]))))
    R["%s_peaks_gt3ms_near_event" % tag] = "%d/%d" % (near_ev, len(peaks))
    R["%s_peaks_gt3ms_within_20ms_of_sat_link" % tag] = "%d/%d" % (near_link, len(peaks)) if len(links) else "-"
    return R, detail


def analyze(prefix, sat_path=None):
    R = {"run": os.path.basename(prefix)}
    ev = read_events(prefix + "_events.csv")
    meta = read_meta(prefix + ".csv.meta.txt")
    task, _ = columns(prefix + "_task.csv", ["t_mono_s", "task_state", "source_cmd", "theta_deg", "sample_age_ms", "e_pos_m", "e_rot_rad"])
    rob, rhead = columns(prefix + ".csv", ["t", "period", "success_rate", "phase", "flags"] + ["q%d" % i for i in range(1, 8)])
    notes = []

    # ---- outcome
    states = [(t, n[len("state "):], d) for t, n, d in ev if n.startswith("state ")]
    final = states[-1][1] if states else "?"
    R["final_state"] = final
    R["exit_derived"] = EXIT.get(final, NAN)
    fault = [d for t, n, d in states if n.startswith("FAULT")]
    R["fault_reason"] = fault[0] if fault else ""
    _, R["loop_stop"] = first(ev, "control loop stop")
    R["loop_stop"] = R["loop_stop"] or ""

    # ---- command
    _, cmd_path = first(ev, "state VALIDATE")
    cmd = {}
    if cmd_path and os.path.exists(local_path(cmd_path)):
        with open(local_path(cmd_path)) as f:
            cmd = yaml.safe_load(f)
        R["command"] = os.path.basename(cmd_path)
    else:
        R["command"] = cmd_path or ""
        notes.append("command file not found")
    theta_star = float(cmd.get("theta_star_deg", NAN))
    tau = float(cmd.get("contact_time_s", NAN))
    omega = float(cmd.get("omega_deg_s", NAN))
    R["theta_star_deg"], R["tau_s"], R["omega_deg_s"] = theta_star, tau, omega
    R["weights_sha256_cmd"] = str(cmd.get("weights_sha256", ""))[:16]
    wfile = NAN
    if cmd_path and os.path.exists(local_path(cmd_path)):
        with open(local_path(cmd_path)) as f:
            m = re.search(r"weights:\s*(\S+)", f.read(2000))
        if m:
            wp = local_path(m.group(1))
            wp = os.path.join(wp, "weights.yaml") if os.path.isdir(wp) else wp
            if os.path.exists(wp):
                wfile = sha256_file(wp) == cmd.get("weights_sha256")
    R["weights_file_matches"] = wfile
    R["git_head"] = meta.get("git_head", "")[:12]
    R["git_dirty"] = meta.get("git_dirty", "")
    R["config"] = os.path.basename(meta.get("config_path", ""))

    # ---- trigger / approach (events)
    t_start, _ = first(ev, "START sent")
    t_trig, d_trig = first(ev, "TRIGGER")
    t_end, d_end = first(ev, "APPROACH ended (t = tau, goal)")
    R["start_to_trigger_s"] = t_trig - t_start if t_trig and t_start else NAN
    if d_trig:
        R["trigger_error_deg"] = wrap180(num_after(d_trig, "theta_trig") - num_after(d_trig, "theta"))
        R["trigger_age_ms"] = num_after(d_trig, "age")
    else:
        R["trigger_error_deg"] = R["trigger_age_ms"] = NAN
    R["approach_duration_s"] = t_end - t_trig if t_end and t_trig else NAN
    if d_end:
        th_end, age_end = num_after(d_end, "theta"), num_after(d_end, "age")
        R["phase_contact_task_deg"] = wrap180(th_end - theta_star)
        R["phase_contact_task_extrap_deg"] = R["phase_contact_task_deg"] + omega * age_end * 1e-3
    else:
        R["phase_contact_task_deg"] = R["phase_contact_task_extrap_deg"] = NAN

    # ---- satellite
    sat_path = sat_path or find_satellite(prefix)
    R["satellite_csv"] = os.path.basename(sat_path) if sat_path else ""
    R["phase_contact_sat_deg"] = R["e_q7_trigger_deg"] = R["e_q7_approach_end_deg"] = NAN
    R["sat_clock_ppm"] = R["sat_clock_resid_ms"] = NAN
    R["sat_faults"] = R["sat_stop_reason"] = ""
    fit = None
    if sat_path:
        smeta = read_meta(sat_path + ".meta.txt")
        R["sat_stop_reason"] = smeta.get("stop_reason", "") + (": " + smeta.get("stop_message", "") if smeta.get("stop_message") else "")
        try:
            fbits = int(smeta.get("faults", "0"), 16)
        except ValueError:
            fbits = 0
        R["sat_faults"] = "0x%x [%s]" % (fbits, ",".join(n for k, n in enumerate(SAT_FAULT_BITS) if fbits >> k & 1))
        R["sat_git_head"] = smeta.get("git_head", "")[:12]
        sat, _ = columns(sat_path, ["t", "state", "theta_flange_deg", "e_q7", "omega_cmd_deg_s", "faults", "period", "success_rate"])
        fit = sat_clock_fit(task, sat)
        if fit:
            R["sat_clock_ppm"] = (fit["b"] - 1.0) * 1e6
            R["sat_clock_resid_ms"] = fit["resid_ms"]
            to_sat = lambda tm: (tm - fit["a"]) / fit["b"]
            e7 = np.degrees(sat["e_q7"])
            if t_trig:
                R["e_q7_trigger_deg"] = float(np.interp(to_sat(t_trig), sat["t"], e7))
                if math.isfinite(tau):
                    ts_c = to_sat(t_trig + tau)
                    if sat["t"][0] <= ts_c <= sat["t"][-1]:
                        R["phase_contact_sat_deg"] = wrap180(float(np.interp(ts_c, sat["t"], sat["theta_flange_deg"])) - theta_star)
            if t_end:
                R["e_q7_approach_end_deg"] = float(np.interp(to_sat(t_end), sat["t"], e7))
        else:
            notes.append("satellite clock fit not possible (no constant-speed segment seen by the task)")

    # ---- jitter of the two 1 kHz loops
    detail = []
    nn = min(len(rob["t"]), len(task["t_mono_s"]))
    Rj, d = jitter("r1", task["t_mono_s"][:nn], rob["period"][:nn], rob["success_rate"][:nn], t_start, task["t_mono_s"],
                   task["task_state"], ev)
    R.update(Rj)
    detail += d
    if sat_path:
        if fit:
            sat_mono = fit["a"] + fit["b"] * sat["t"]
            R["sat_time_base"] = "theta fit"
        else:  # fallback: satellite WAIT_START -> RUNNING at the START sent by the task (UDP on localhost)
            i_run = int(np.argmax(sat["state"] >= 2))
            sat_mono = (t_start if t_start else 0.0) + (sat["t"] - sat["t"][i_run])
            R["sat_time_base"] = "START offset"
        Rj, d = jitter("sat", sat_mono, sat["period"], sat["success_rate"], t_start, task["t_mono_s"], task["task_state"], ev)
        R.update(Rj)
        detail += d
    R["_detail"] = detail

    # ---- errors per phase (task CSV). Only cycles with the loop RUNNING (robot CSV phase 1): in the controlled stop
    # (phase 2) the error is measured against the stop target (pose at the stop) and restarts from 0. "loop_stop" = phase 2.
    st = task["task_state"].astype(int)
    lp = np.full(len(st), 1)
    nn = min(len(st), len(rob["phase"]))
    lp[:nn] = rob["phase"][:nn].astype(int)
    for name, group in PHASE_GROUPS + [("loop_stop", None)]:
        m = (lp == 2) if group is None else (np.isin(st, list(group)) & (lp == 1))
        for key, arr, scale in (("e_pos", task["e_pos_m"], 1e3), ("e_rot", task["e_rot_rad"], 180.0 / math.pi)):
            unit = "mm" if key == "e_pos" else "deg"
            R["%s_max_%s_%s" % (key, name, unit)] = float(np.nanmax(arr[m]) * scale) if m.any() else NAN
            R["%s_final_%s_%s" % (key, name, unit)] = float(arr[m][-1] * scale) if m.any() else NAN

    # ---- joint margins (robot CSV, aligned row by row with the task CSV)
    q = np.vstack([rob["q%d" % i] for i in range(1, 8)]).T
    if len(q) != len(st):
        notes.append("robot CSV %d rows != task CSV %d rows: margins aligned by truncation" % (len(q), len(st)))
    n = min(len(q), len(st))
    q, stq = q[:n], st[:n]
    marg = np.degrees(np.minimum(q - Q_MIN, Q_MAX - q))
    for name, group in (("approach", {4}), ("follow", FOLLOW_STATES)):
        m = np.isin(stq, list(group))
        for j in range(7):
            R["margin_min_%s_q%d_deg" % (name, j + 1)] = float(marg[m, j].min()) if m.any() else NAN
        if m.any():
            j = int(np.argmin(marg[m].min(axis=0)))
            R["margin_min_%s_deg" % name] = "%.2f (q%d)" % (marg[m, j].min(), j + 1)
        else:
            R["margin_min_%s_deg" % name] = ""
    R["q5_final_deg"] = float(np.degrees(rob["q5"][-1]))
    R["q7_final_deg"] = float(np.degrees(rob["q7"][-1]))
    fl = rob["flags"].astype(int)
    R["cycles_rate_limited"] = int(np.sum(fl & 1 != 0))
    R["cycles_torque_clamped"] = int(np.sum(fl & 2 != 0))
    R["robot1_period_max_ms"] = float(np.nanmax(rob["period"]) * 1e3)
    R["robot1_success_rate_min"] = float(np.nanmin(rob["success_rate"][500:])) if len(rob["success_rate"]) > 500 else NAN

    # ---- gripper
    t_gc, _ = first(ev, "gripper: grasp commanded")
    t_go, _ = first(ev, "gripper: grasp OK")
    R["grasp_cmd_to_ok_s"] = t_go - t_gc if t_gc and t_go else NAN
    t_stop = None
    for t, n_, d in ev:
        if n_ in ("STOP_RAMP sent to the satellite", "STOP sent to the satellite"):
            t_stop = t
            break
    t_open = None
    for t, n_, d in ev:
        if n_ in ("gripper: released (open confirmed)", "gripper: opened after the fault"):
            t_open = t
    R["stop_to_open_s"] = t_open - t_stop if t_open and t_stop and t_open > t_stop else NAN
    t_sst, _ = first(ev, "satellite STOPPED: target frozen, gripper open commanded")
    R["sat_stopped_to_open_s"] = t_open - t_sst if t_open and t_sst else NAN

    # ---- task thread health (sat_link lines)
    links_all = [(t, d) for t, n_, d in ev if n_ == "sat_link"]
    links = [d for t, d in links_all]
    R["load_fail"] = num_after(links[-1], "load_fail") if links else NAN
    R["max_task_period_ms"] = max((num_after(d, "task period max") for d in links), default=NAN)
    # rx age only BEFORE the first STOP: afterwards the satellite ends and stops publishing (the age grows by design)
    R["max_rx_age_before_stop_ms"] = max((num_after(d, "rx_age") * 1e3 for t, d in links_all if t_stop is None or t < t_stop), default=NAN)
    _, fs = first(ev, "FOLLOW", startswith=True)
    R["follow_start"] = fs or ""
    R["notes"] = "; ".join(notes)
    return R


# ---- output ------------------------------------------------------------------------------------------------------
def fmt(v):
    if isinstance(v, float):
        if math.isnan(v):
            return "-"
        return "%.4f" % v if abs(v) < 100 else "%.2f" % v
    if isinstance(v, bool):
        return "yes" if v else "NO"
    return str(v)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--log-dir", default=DEFAULT_LOG_DIR)
    ap.add_argument("--sat", help="satellite CSV (only with a single run)")
    ap.add_argument("--out-md")
    ap.add_argument("--out-csv")
    ap.add_argument("--jitter-detail", action="store_true", help="also print every period > 3 ms and every success_rate < 0.95 episode")
    a = ap.parse_args(argv)
    if a.sat and len(a.runs) != 1:
        ap.error("--sat needs exactly one run")
    res = [analyze(resolve_prefix(r, a.log_dir), a.sat) for r in a.runs]
    keys = []
    for r in res:
        keys += [k for k in r if k not in keys and not k.startswith("_")]
    md = ["| metric | " + " | ".join(r["run"].split("_")[1] for r in res) + " |", "|---|" + "---|" * len(res)]
    for k in keys:
        if k == "run":
            continue
        md.append("| %s | %s |" % (k, " | ".join(fmt(r.get(k, "")).replace("|", "/") for r in res)))
    text = "\n".join(md) + "\n"
    if a.out_md:
        with open(a.out_md, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    if a.jitter_detail:
        for r in res:
            print("\n## %s" % r["run"])
            for line in r["_detail"]:
                print("- " + line)
    if a.out_csv:
        with open(a.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(keys)
            for r in res:
                w.writerow([r.get(k, "") for k in keys])
    return 0


if __name__ == "__main__":
    sys.exit(main())
