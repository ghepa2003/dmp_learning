#!/usr/bin/env python3
"""
bag_time_probe.py - legge header.stamp (sim) e timestamp di ricezione dai bag rosbag2 .db3,
SENZA ROS (solo sqlite3 + numpy).

Blob CDR (mensaggi con std_msgs/Header come primo campo: PoseStamped, TwistStamped, JointState;
per rosgraph_msgs/Clock il campo `clock` sta nella stessa posizione):
  byte 0..3 : intestazione di incapsulamento CDR (byte0 = 0x00, byte1 = 0x01 little-endian /
              0x00 big-endian, byte 2..3 opzioni)
  byte 4..7 : stamp.sec  (int32)
  byte 8..11: stamp.nanosec (uint32)
Verifiche: byte0 == 0 e byte1 in {0,1} uguali per tutti i messaggi; nanosec < 1e9; sec >= 0.
Se il primo messaggio non le soddisfa lo stamp del topic e' NON VERIFICATO (nessun dato usato).

Definizioni:
  Topic cercati per ruolo: target_pose_aligned = nome che termina con "/target_pose_aligned";
  actual_pose = termina con "/actual_pose"; joint_states = "/joint_states"; clock = "/clock";
  target_pose = "/target_pose"; target_twist = "/target_twist".
  Ricezione = colonna `timestamp` di sqlite (ns). Ordine dei messaggi = ORDER BY timestamp, id
  (verificato contro le CSV: vedi verify_csv_alignment).
  RTF = delta(stamp) / delta(ricezione). Finestre di 10 s: finestre COMPLETE e consecutive di
  10 s di tempo di ricezione dal primo messaggio del topic; RTF di finestra = (stamp ultimo -
  stamp primo)/(ricezione ultima - ricezione prima) dei messaggi nella finestra (>= 2 messaggi).
  Intervalli: differenze consecutive di header.stamp (ms), mediana e massimo su tutte le
  differenze. "non monotoni" = stamp[i] < stamp[i-1]; "nulli" = sec==0 e nanosec==0;
  "ripetuti" = stamp[i] == stamp[i-1].

Uso:
  python3 tools/bag_time_probe.py [--manifest tools/runs_manifest.yaml] [--out tools/canonical_out]
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

ROLE_MATCH = {
    "target_pose_aligned": ("suffix", "/target_pose_aligned"),
    "actual_pose": ("suffix", "/actual_pose"),
    "joint_states": ("exact", "/joint_states"),
    "clock": ("exact", "/clock"),
    "target_pose": ("exact", "/target_pose"),
    "target_twist": ("exact", "/target_twist"),
}
SIM_LIKE_MAX_SEC = 10 ** 8   # sec < 1e8 => "sim_like"; sec >= 1e8 => "epoch_like" (es. 1.7e9)
WINDOW_S = 10.0


@dataclass
class TopicStamps:
    role: str
    name: str
    recv_ns: np.ndarray      # int64, sqlite `timestamp`
    stamp_ns: np.ndarray     # int64, header.stamp in ns (valido solo se stamp_verified)
    stamp_verified: bool
    stamp_class: str         # sim_like | epoch_like | NON VERIFICATO
    note: str = ""


def db_files(bag_dir) -> list[Path]:
    bag_dir = Path(bag_dir)
    meta = bag_dir / "metadata.yaml"
    if not meta.is_file():
        raise FileNotFoundError(f"metadata.yaml assente: {meta}")
    info = yaml.safe_load(meta.read_text())["rosbag2_bagfile_information"]
    if info.get("storage_identifier") != "sqlite3":
        raise ValueError(f"{bag_dir}: storage_identifier {info.get('storage_identifier')!r} != sqlite3")
    files = [bag_dir / f for f in info["relative_file_paths"]]
    for f in files:
        if not f.is_file():
            raise FileNotFoundError(f"file db3 assente: {f}")
    return files


def _match_role(topic_name: str) -> str | None:
    for role, (kind, pat) in ROLE_MATCH.items():
        if (kind == "exact" and topic_name == pat) or (kind == "suffix" and topic_name.endswith(pat)):
            return role
    return None


def _decode_prefixes(blobs: list[bytes]) -> tuple[bool, np.ndarray, str]:
    """Ritorna (ok, stamp_ns, nota). ok solo se tutte le verifiche CDR/stamp passano sul PRIMO messaggio
    e il layout e' coerente su tutti."""
    n = len(blobs)
    if n == 0:
        return False, np.empty(0, np.int64), "nessun messaggio"
    if any(len(b) < 12 for b in blobs):
        return False, np.empty(0, np.int64), "blob < 12 byte"
    arr = np.frombuffer(b"".join(blobs), dtype=np.uint8).reshape(n, 12)
    if arr[0, 0] != 0 or arr[0, 1] not in (0, 1):
        return False, np.empty(0, np.int64), f"intestazione CDR anomala {arr[0, :4].tolist()}"
    if not (arr[:, 0] == arr[0, 0]).all() or not (arr[:, 1] == arr[0, 1]).all():
        return False, np.empty(0, np.int64), "endianness non uniforme tra i messaggi"
    little = bool(arr[0, 1] == 1)
    sec = arr[:, 4:8].copy().view("<i4" if little else ">i4").ravel().astype(np.int64)
    nsec = arr[:, 8:12].copy().view("<u4" if little else ">u4").ravel().astype(np.int64)
    if not (0 <= nsec[0] < 10 ** 9 and sec[0] >= 0):
        return False, np.empty(0, np.int64), f"primo stamp implausibile sec={sec[0]} nsec={nsec[0]}"
    return True, sec * 10 ** 9 + nsec, ("little-endian" if little else "big-endian")


def read_bag(bag_dir) -> dict[str, TopicStamps]:
    """Legge tutti i topic dei ruoli noti. Restituisce {role: TopicStamps} (solo ruoli presenti)."""
    per_role: dict[str, list] = {}
    for db in db_files(bag_dir):
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            for tid, name in con.execute("SELECT id, name FROM topics"):
                role = _match_role(name)
                if role is None:
                    continue
                rows = con.execute("SELECT timestamp, substr(data,1,12) FROM messages "
                                   "WHERE topic_id=? ORDER BY timestamp, id", (tid,)).fetchall()
                per_role.setdefault(role, []).append((name, rows))
        finally:
            con.close()
    out: dict[str, TopicStamps] = {}
    for role, parts in per_role.items():
        names = {n for n, _ in parts}
        if len(names) != 1:
            raise ValueError(f"{bag_dir}: ruolo {role} associato a piu' topic {sorted(names)}")
        rows = sorted((r for _, rs in parts for r in rs), key=lambda r: r[0])
        recv = np.array([r[0] for r in rows], dtype=np.int64)
        ok, stamp, note = _decode_prefixes([bytes(r[1]) for r in rows])
        if ok:
            cls = "sim_like" if stamp[0] // 10 ** 9 < SIM_LIKE_MAX_SEC else "epoch_like"
        else:
            cls, stamp = "NON VERIFICATO", np.zeros(len(recv), np.int64)
        out[role] = TopicStamps(role, names.pop(), recv, stamp, ok, cls, note)
    return out


def verify_csv_alignment(bag: dict[str, TopicStamps], csv_t: dict[str, np.ndarray],
                         tol_s: float = 1e-6) -> None:
    """Le CSV degli extractor hanno t = (ricezione - t0)*1e-9 con t0 = primo messaggio tra
    target_pose_aligned e actual_pose (condiviso) oppure primo joint_states. Verifica riga per riga;
    ValueError se il conteggio o i tempi non coincidono (bag e CSV non corrispondono)."""
    t0_pose = min(bag["target_pose_aligned"].recv_ns[0], bag["actual_pose"].recv_ns[0])
    t0 = {"target_aligned": t0_pose, "actual_pose": t0_pose, "joint_states": bag["joint_states"].recv_ns[0]}
    role_of = {"target_aligned": "target_pose_aligned", "actual_pose": "actual_pose",
               "joint_states": "joint_states"}
    for csv_role, t in csv_t.items():
        ts = bag[role_of[csv_role]].recv_ns
        if len(ts) != len(t):
            raise ValueError(f"{csv_role}: {len(t)} righe CSV contro {len(ts)} messaggi nel bag")
        d = np.max(np.abs((ts - t0[csv_role]) * 1e-9 - t))
        if d > tol_s:
            raise ValueError(f"{csv_role}: t CSV != (ricezione - t0) (max scarto {d:.3e} s)")


def topic_stats(ts: TopicStamps) -> dict:
    n = len(ts.recv_ns)
    row = {"topic": ts.name, "role": ts.role, "n": n, "stamp_class": ts.stamp_class,
           "cdr_note": ts.note,
           "recv_first_s": ts.recv_ns[0] * 1e-9, "recv_last_s": ts.recv_ns[-1] * 1e-9}
    d_ric = (ts.recv_ns[-1] - ts.recv_ns[0]) * 1e-9
    row["d_recv_s"] = d_ric
    row["msg_per_s_recv"] = (n - 1) / d_ric if d_ric > 0 else float("nan")
    if not ts.stamp_verified:
        for k in ("sim_first_s", "sim_last_s", "d_sim_s", "rtf_overall", "rtf_win_min", "rtf_win_max",
                  "n_windows", "msg_per_s_sim", "dt_sim_median_ms", "dt_sim_max_ms", "n_non_monotonic",
                  "n_null", "n_repeated"):
            row[k] = "NON VERIFICATO"
        return row
    st = ts.stamp_ns
    d_sim = (st[-1] - st[0]) * 1e-9
    diffs = np.diff(st)
    row.update({"sim_first_s": st[0] * 1e-9, "sim_last_s": st[-1] * 1e-9, "d_sim_s": d_sim,
                "rtf_overall": d_sim / d_ric if d_ric > 0 else float("nan"),
                "msg_per_s_sim": (n - 1) / d_sim if d_sim > 0 else float("nan"),
                "dt_sim_median_ms": float(np.median(diffs)) * 1e-6, "dt_sim_max_ms": float(diffs.max()) * 1e-6,
                "n_non_monotonic": int((diffs < 0).sum()), "n_null": int((st == 0).sum()),
                "n_repeated": int((diffs == 0).sum())})
    rel = (ts.recv_ns - ts.recv_ns[0]) * 1e-9
    n_win = int(rel[-1] // WINDOW_S)
    edges = np.searchsorted(rel, np.arange(n_win + 1) * WINDOW_S, side="left")
    rtfs = []
    for b in range(n_win):
        lo, hi = edges[b], edges[b + 1] - 1
        if hi - lo >= 1 and ts.recv_ns[hi] > ts.recv_ns[lo]:
            rtfs.append((st[hi] - st[lo]) / (ts.recv_ns[hi] - ts.recv_ns[lo]))
    row["n_windows"] = len(rtfs)
    row["rtf_win_min"] = min(rtfs) if rtfs else float("nan")
    row["rtf_win_max"] = max(rtfs) if rtfs else float("nan")
    return row


def resolve_extra_run(run_id: str, data_dir: Path, bags_dir: Path) -> dict:
    """Run fuori dal manifest (es. phase0): file per convenzione, ma tutti devono esistere."""
    paths = {r: data_dir / f"{r}_{run_id}.csv" for r in ("target_aligned", "actual_pose", "joint_states")}
    for r, p in paths.items():
        if not p.is_file():
            raise FileNotFoundError(f"{run_id}: file mancante ({r}): {p}")
    bag = bags_dir / run_id
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"{run_id}: bag mancante: {bag}")
    return {"run_id": run_id, "paths": paths, "bag_path": bag}


def settle_report(entry: dict) -> dict:
    """Parte 3: t_settle_1e4 in sim time e in ricezione + distanza js-target in sim time."""
    import pandas as pd
    import canonical_metrics as cm
    bag = read_bag(entry["bag_path"])
    for role in ("target_pose_aligned", "actual_pose", "joint_states"):
        if role not in bag or not bag[role].stamp_verified:
            raise ValueError(f"{entry['run_id']}: stamp di {role} NON VERIFICATO/assente")
    T = pd.read_csv(entry["paths"]["target_aligned"])
    csv_t = {"target_aligned": T["t"].to_numpy(),
             "actual_pose": pd.read_csv(entry["paths"]["actual_pose"])["t"].to_numpy(),
             "joint_states": pd.read_csv(entry["paths"]["joint_states"])["t"].to_numpy()}
    verify_csv_alignment(bag, csv_t)
    idx = cm.settle_index(T[["x", "y", "z"]].to_numpy())
    tg, js = bag["target_pose_aligned"], bag["joint_states"]
    return {
        "run_id": entry["run_id"], "n_target": len(tg.recv_ns), "settle_idx": idx,
        "t_settle_sim_s": (tg.stamp_ns[idx] - tg.stamp_ns[0]) * 1e-9,
        "t_settle_recv_s": (tg.recv_ns[idx] - tg.recv_ns[0]) * 1e-9,
        "t_settle_csv_s": float(csv_t["target_aligned"][idx]),
        "tgt_last_sim_s": (tg.stamp_ns[-1] - tg.stamp_ns[0]) * 1e-9,
        "sim_js_last_minus_tgt_last_s": (js.stamp_ns[-1] - tg.stamp_ns[-1]) * 1e-9,
        "recv_js_last_minus_tgt_last_s": (js.recv_ns[-1] - tg.recv_ns[-1]) * 1e-9,
        "sim_js_first_minus_tgt_first_s": (js.stamp_ns[0] - tg.stamp_ns[0]) * 1e-9,
        "recv_js_first_minus_tgt_first_s": (js.recv_ns[0] - tg.recv_ns[0]) * 1e-9,
        "t_settle_sim_abs_s": tg.stamp_ns[idx] * 1e-9,
    }


PART3_RUNS = ("reach_task_baseline_replay_prodmp_kt200_delay1", "reach_task_satellite_rot_phase90_kt200",
              "reach_task_satellite_rot_phase0")


def main(argv=None) -> int:
    import pandas as pd
    import canonical_metrics as cm
    ap = argparse.ArgumentParser(description="header.stamp e RTF dai bag .db3 (senza ROS)")
    ap.add_argument("--manifest", default=str(cm.DEFAULT_MANIFEST))
    ap.add_argument("--out", default=str(cm.DEFAULT_OUT))
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    entries = cm.load_manifest(args.manifest)
    rows = []
    for e in entries:
        if e.get("bag_path") is None:
            raise ValueError(f"{e['run_id']}: manifest senza 'bag'")
        bag = read_bag(e["bag_path"])
        absent = [r for r in ROLE_MATCH if r not in bag]
        for role in ("target_pose_aligned", "actual_pose", "joint_states"):
            if role not in bag:
                raise ValueError(f"{e['run_id']}: topic {role} assente nel bag")
        for role, ts in bag.items():
            r = topic_stats(ts)
            r["run_id"] = e["run_id"]
            r["roles_absent"] = ",".join(absent)
            rows.append(r)
    topics = pd.DataFrame(rows)
    topics.to_csv(out / "bag_time_probe_topics.csv", index=False)
    cols = ["run_id", "role", "n", "stamp_class", "sim_first_s", "sim_last_s", "d_sim_s", "d_recv_s",
            "rtf_overall", "rtf_win_min", "rtf_win_max", "n_windows", "msg_per_s_sim", "msg_per_s_recv",
            "dt_sim_median_ms", "dt_sim_max_ms", "n_non_monotonic", "n_null", "n_repeated"]
    topics["run_id"] = topics["run_id"].str.replace("reach_task_", "", regex=False)
    with pd.option_context("display.width", 300, "display.max_rows", 500, "display.max_columns", 50,
                           "display.float_format", "{:.3f}".format):
        print(topics[cols].to_string(index=False))
        print("\nruoli assenti in tutti i bag:", sorted(set(",".join(topics['roles_absent']).split(",")) - {''}))

    by_id = {e["run_id"]: e for e in entries}
    s3 = []
    for rid in PART3_RUNS:
        entry = by_id.get(rid) or resolve_extra_run(rid, cm.DEFAULT_DATA_DIR, cm.DEFAULT_BAGS_DIR)
        s3.append(settle_report(entry))
    s3df = pd.DataFrame(s3)
    s3df.to_csv(out / "bag_time_probe_settle.csv", index=False)
    with pd.option_context("display.width", 300, "display.max_columns", 50,
                           "display.float_format", "{:.3f}".format):
        print("\n=== PARTE 3 ===")
        print(s3df.T.to_string(header=False))
    print(f"\nscritti: {out}/bag_time_probe_topics.csv, {out}/bag_time_probe_settle.csv")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(main())
