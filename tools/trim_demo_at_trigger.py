#!/usr/bin/env python3
"""Trim a demo CSV (and its derived joint_states CSV) at the first gripper_trigger row.

Usage: trim_demo_at_trigger.py <demo_raw.csv>

Reads <demo_raw.csv> (header t,x,y,z,qw,qx,qy,qz,gripper_trigger), keeps the rows up to and
including the first one with gripper_trigger != 0 and writes <stem>_trim.csv. Then reads
<stem>_joint_states.csv (header t,fer_joint1..fer_joint7, same time base) and writes
<stem>_trim_joint_states.csv with the rows t <= t_trigger. Existing outputs are never
overwritten. Trigger semantics match demo_csv_io (value != 0 is a trigger).
"""
import sys
from pathlib import Path


class TrimError(Exception):
    pass


def _read_lines(path):
    with open(path, newline="") as f:
        return [ln for ln in f.read().splitlines() if ln.strip()]


def _split(line):
    return [c.strip() for c in line.split(",")]


def _parse_float(text, what, path, row):
    try:
        return float(text)
    except ValueError:
        raise TrimError(f"{path}: row {row}: cannot parse {what} '{text}'")


def trim_demo(demo_path):
    """Returns (out_demo, out_js, rows_read, rows_written, t_trigger, tau, trigger_is_last, js_read, js_written)."""
    demo_path = Path(demo_path)
    if not demo_path.is_file():
        raise TrimError(f"demo CSV not found: {demo_path}")
    stem = demo_path.stem
    js_path = demo_path.with_name(f"{stem}_joint_states.csv")
    out_demo = demo_path.with_name(f"{stem}_trim.csv")
    out_js = demo_path.with_name(f"{stem}_trim_joint_states.csv")

    for out in (out_demo, out_js):
        if out.exists():
            raise TrimError(f"output already exists, refusing to overwrite: {out}")

    lines = _read_lines(demo_path)
    if len(lines) < 2:
        raise TrimError(f"{demo_path}: no data rows")
    header, rows = lines[0], lines[1:]
    cols = _split(header)
    if "t" not in cols or "gripper_trigger" not in cols:
        raise TrimError(f"{demo_path}: header must contain 't' and 'gripper_trigger', got: {header}")
    t_idx, g_idx = cols.index("t"), cols.index("gripper_trigger")

    trig = None
    times = []
    for i, row in enumerate(rows):
        f = _split(row)
        if len(f) <= max(t_idx, g_idx):
            raise TrimError(f"{demo_path}: row {i + 2}: too few fields")
        times.append(_parse_float(f[t_idx], "t", demo_path, i + 2))
        if trig is None and f[g_idx] != "" and _parse_float(f[g_idx], "gripper_trigger", demo_path, i + 2) != 0.0:
            trig = i
    if trig is None:
        raise TrimError(f"{demo_path}: no row with gripper_trigger != 0 (nothing to trim at)")
    if trig == 0:
        raise TrimError(f"{demo_path}: the trigger is on the first data row (trimmed demo would have 1 sample)")

    if not js_path.is_file():
        raise TrimError(f"joint_states CSV not found: {js_path}")
    js_lines = _read_lines(js_path)
    if len(js_lines) < 2:
        raise TrimError(f"{js_path}: no data rows")
    js_header, js_rows = js_lines[0], js_lines[1:]
    js_cols = _split(js_header)
    if "t" not in js_cols:
        raise TrimError(f"{js_path}: header must contain 't', got: {js_header}")
    jt_idx = js_cols.index("t")
    t_trigger = times[trig]
    js_kept = []
    for i, row in enumerate(js_rows):
        f = _split(row)
        if len(f) <= jt_idx:
            raise TrimError(f"{js_path}: row {i + 2}: too few fields")
        if _parse_float(f[jt_idx], "t", js_path, i + 2) <= t_trigger:
            js_kept.append(row)
    if not js_kept:
        raise TrimError(f"{js_path}: no rows with t <= t_trigger={t_trigger}")

    kept = rows[: trig + 1]
    try:
        with open(out_demo, "x", newline="") as f:
            f.write("\n".join([header] + kept) + "\n")
        with open(out_js, "x", newline="") as f:
            f.write("\n".join([js_header] + js_kept) + "\n")
    except FileExistsError as e:
        raise TrimError(f"output already exists, refusing to overwrite: {e.filename}")

    tau = times[trig] - times[0]
    return (out_demo, out_js, len(rows), len(kept), t_trigger, tau, trig == len(rows) - 1,
            len(js_rows), len(js_kept))


def main(argv):
    if len(argv) != 2:
        print(__doc__.strip().split("\n")[2], file=sys.stderr)
        return 2
    try:
        out_demo, out_js, n_read, n_kept, t_trig, tau, is_last, js_read, js_kept = trim_demo(argv[1])
    except TrimError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"demo:         read {n_read} rows, wrote {n_kept} -> {out_demo}")
    print(f"joint_states: read {js_read} rows, wrote {js_kept} -> {out_js}")
    print(f"t_trigger = {t_trig:.6f} s, tau = {tau:.6f} s")
    if not is_last:
        print(f"warning: the trigger row is not the last row of the raw demo ({n_read - n_kept} later rows dropped)",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
