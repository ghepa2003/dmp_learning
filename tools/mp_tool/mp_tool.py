#!/usr/bin/env python3
"""mp_tool - CLI orchestrator for the DMP / ProDMP offline pipeline.

This tool does NOT reimplement any numeric logic. It only invokes the
already-built, already-verified C++ binaries under tools/dmp_offline_test/
(and tools/dmp_offline_test/06_goal_generalization/plots/build/) as
subprocesses, plus a small number of existing Python plot scripts, and
organizes their outputs under runs/<timestamp>_<subcommand>_<label>/.

See README.md in this directory for usage examples and the list of
subcommands that could NOT be cleanly wrapped (and why).
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Paths (fixed relative to this file -> repo root)
# ---------------------------------------------------------------------------
THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[2]  # tools/mp_tool/mp_tool.py -> repo root
OFFLINE_DIR = REPO_ROOT / "tools" / "dmp_offline_test"
BUILD_DIR = OFFLINE_DIR / "build"
GOAL_DIR = OFFLINE_DIR / "06_goal_generalization"
GOAL_BUILD_DIR = GOAL_DIR / "plots" / "build"
COMMON_SCRIPTS = OFFLINE_DIR / "common" / "scripts"

LEARN_TEST_PRODMP = BUILD_DIR / "learn_and_test_prodmp"
LEARN_TEST_DMP = BUILD_DIR / "learn_and_test_dmp"
LEARN_TEST_PRODMP_HOLDOUT = BUILD_DIR / "learn_and_test_prodmp_holdout"
EXTRACT_PRODMP_DIAGNOSTICS = BUILD_DIR / "extract_prodmp_diagnostics"
RUN_GOAL_GENERALIZATION = GOAL_BUILD_DIR / "run_goal_generalization"
RUN_PRODMP_GOAL_GENERALIZATION = GOAL_BUILD_DIR / "run_prodmp_goal_generalization"

PLOT_REAL_DEMO = OFFLINE_DIR / "02_real_data" / "scripts" / "plot_real_demo.py"
PLOT_GOAL_GENERALIZATION = GOAL_DIR / "scripts" / "plot_goal_generalization.py"

# Production-default config sources (READ-ONLY: mp_tool never writes to these).
PRODMP_FEATURES_YAML = REPO_ROOT / "src" / "haptic_dmp_learning" / "config" / "prodmp_features.yaml"
DMP_FEATURES_YAML = REPO_ROOT / "src" / "haptic_dmp_learning" / "config" / "dmp_features.yaml"
PARAMS_YAML = REPO_ROOT / "src" / "haptic_dmp_learning" / "config" / "params.yaml"

RUNS_DIR = REPO_ROOT / "runs"

# Gazebo eval pipeline (tools/gazebo_cartesian_eval/) -- READ-ONLY for
# gazebo-collect. mp_tool must never move/rename/copy/modify anything under
# this tree; it only creates symlinks under runs/ pointing back at the
# originals (see cmd_gazebo_collect).
GAZEBO_DIR = REPO_ROOT / "tools" / "gazebo_cartesian_eval"
GAZEBO_BAGS_DIR = GAZEBO_DIR / "bags"
GAZEBO_DATA_DIR = GAZEBO_DIR / "data"
GAZEBO_PLOTS_DIR = GAZEBO_DIR / "plots"

# data/<prefix>_<run_name>.csv -> (clean symlink name, human description).
# Prefixes match exactly what extract_bag_to_csv.py / extract_force_grasp_to_csv.py
# write (see tools_inventory.md section 3.1).
GAZEBO_DATA_MAP = {
    "target_aligned": ("tracking_target.csv", "target pose aligned for tracking comparison"),
    "actual_pose": ("tracking_actual.csv", "actual/measured robot pose"),
    "force": ("force.csv", "contact force log"),
    "grasp_state": ("grasp_state.csv", "grasp state machine log"),
    "gripper_cmd": ("gripper_cmd.csv", "gripper command log"),
    "target_odom": ("target_odom.csv", "target object odometry"),
}
# These 4 categories are only produced by record_full_test_bag.sh (or the
# all-in-one run_full_test_analysis.sh); record_bag.sh only records tracking
# (target_aligned/actual_pose), so their absence is expected/normal, not an
# error, for tracking-only runs.
GAZEBO_FULL_TEST_ONLY_PREFIXES = {"force", "grasp_state", "gripper_cmd", "target_odom"}

# plots/<subdir>/*_<run_name>.png -> (clean symlink name, human description,
# which script produces it). Subdir names match tools_inventory.md 3.1.
GAZEBO_PLOT_MAP = {
    "02_gazebo_tracking": ("plot_tracking.png", "cartesian tracking plot (evaluate_cartesian_tracking.py)"),
    "03_gain_sweep": ("plot_gain_sweep.png", "headless tracking plot from a gain-sweep run (evaluate_cartesian_tracking_headless.py)"),
    "04_grasp_test": ("plot_grasp.png", "force/grasp-state plot (plot_force_grasp_state.py)"),
}

# Production defaults for classic DMP shape params, taken from
# live_demo_recorder_node in params.yaml (the currently-active recorder node;
# haptic_dmp_wrapper_node is deprecated per project notes but shares the same
# values anyway).
DMP_PROD_N_BASIS = 200
DMP_PROD_ALPHA_X = 4.6
DMP_PROD_ALPHA_Z = 25.0
DMP_PROD_BETA_Z = 6.25

# The 5 hardcoded goal offsets used by run_prodmp_goal_generalization.cpp
# (source: tools/dmp_offline_test/06_goal_generalization/scripts/
#  run_prodmp_goal_generalization.cpp, create5Goals()). Documented here for
# reference only -- they are NOT configurable via CLI on the binary itself.
PRODMP_GOAL_OFFSETS_M = [
    ("Goal 1", (0.04, 0.03, 0.02)),
    ("Goal 2", (-0.05, 0.04, -0.03)),
    ("Goal 3", (0.03, -0.05, 0.04)),
    ("Goal 4", (-0.04, -0.03, 0.05)),
    ("Goal 5", (0.05, -0.04, -0.03)),
]

# Default sweep grids, reused verbatim from the existing shell sweep scripts
# (see tools/dmp_offline_test/04_basis_sweep/scripts/run_*sweep*.sh):
DEFAULT_GRID_NBASIS = [5, 10, 15, 20, 25, 30, 40, 50, 60, 80, 100, 150, 200, 300, 500]
DEFAULT_GRID_LAMBDA = [1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0]
DEFAULT_GRID_WINDOW = [0.01, 0.02, 0.05, 0.10, 0.20]


# ---------------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------------
class ToolError(RuntimeError):
    pass


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def make_run_dir(subcommand: str, label: str) -> Path:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    safe_label = label.replace("/", "_").replace(" ", "_")
    run_dir = RUNS_DIR / f"{timestamp()}_{subcommand}_{safe_label}"
    if run_dir.exists():
        raise ToolError(f"run dir already exists (non-overwriting convention): {run_dir}")
    run_dir.mkdir(parents=True)
    return run_dir


def run_cmd(cmd, cwd=None, env=None, label=""):
    """Runs a subprocess, capturing stdout/stderr. On failure: print full
    original stdout/stderr and abort (hard constraint: never silently
    continue on a wrapped-binary failure)."""
    printable = " ".join(str(c) for c in cmd)
    print(f"[mp_tool] $ {printable}" + (f"   (cwd={cwd})" if cwd else ""))
    result = subprocess.run(
        [str(c) for c in cmd], cwd=str(cwd) if cwd else None, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    if result.returncode != 0:
        print(f"\n[mp_tool] FATAL: subprocess failed (exit {result.returncode}){' for ' + label if label else ''}:")
        print(f"[mp_tool] command: {printable}")
        print("----- stdout -----")
        print(result.stdout)
        print("----- stderr -----")
        print(result.stderr)
        sys.exit(1)
    return result


def require_binary(path: Path):
    if not path.exists():
        raise ToolError(
            f"required binary not found: {path}\n"
            f"Build it first with the project's existing build scripts (mp_tool does not build binaries "
            f"except where explicitly documented, e.g. goal-generalization binaries)."
        )
    return path


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def read_last_summary_row(summary_csv: Path) -> dict:
    with open(summary_csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ToolError(f"summary.csv produced no rows: {summary_csv}")
    return rows[-1]


def flatten_weights(weights_field):
    """Weights in both DMP and ProDMP weights.yaml are stored as a list of
    per-axis weight lists (x, y, z). This just reads that already-computed
    field back out for display -- no recomputation."""
    out = []

    def _walk(x):
        if isinstance(x, (list, tuple)):
            for e in x:
                _walk(e)
        elif isinstance(x, dict):
            for e in x.values():
                _walk(e)
        elif isinstance(x, (int, float)):
            out.append(float(x))

    _walk(weights_field)
    return out


def max_abs_weight_from_yaml(weights_yaml: Path, algo: str) -> float:
    d = load_yaml(weights_yaml)
    if algo == "dmp":
        w = d.get("position_dmp", {}).get("weights")
    else:
        w = d.get("weights")
    vals = flatten_weights(w)
    return max(abs(v) for v in vals) if vals else float("nan")


# ---------------------------------------------------------------------------
# fit
# ---------------------------------------------------------------------------
SATELLITE_OPTIONS = ("satellite_center", "satellite_axis", "satellite_omega_deg_s",
                     "grasp_point", "phase_at_contact_deg")


def _satellite_cli_args(args) -> list:
    """Optional satellite-at-demo description for the <stem>_demo_params.yaml written next to
    weights.yaml by learn_and_test_prodmp. All five options together or none."""
    given = [name for name in SATELLITE_OPTIONS if getattr(args, name, None) is not None]
    if not given:
        return []
    if len(given) != len(SATELLITE_OPTIONS):
        raise ToolError(
            "the satellite options must be given ALL together (--satellite-center, --satellite-axis, "
            "--satellite-omega-deg-s, --grasp-point, --phase-at-contact-deg); "
            f"got only {len(given)} of {len(SATELLITE_OPTIONS)}")
    return [
        "--satellite-center", *[str(v) for v in args.satellite_center],
        "--satellite-axis", *[str(v) for v in args.satellite_axis],
        "--satellite-omega-deg-s", str(args.satellite_omega_deg_s),
        "--grasp-point", args.grasp_point,
        "--phase-at-contact-deg", str(args.phase_at_contact_deg),
    ]


def cmd_fit(args):
    demo_csv = Path(args.demo_csv).resolve()
    if not demo_csv.exists():
        raise ToolError(f"demo CSV not found: {demo_csv}")

    algo = args.algo
    label = args.label or f"{algo}_{demo_csv.stem}"
    _satellite_cli_args(args)  # validate (all-or-none) BEFORE creating the run directory
    run_dir = make_run_dir("fit", label)

    weights_yaml = run_dir / "weights.yaml"
    replay_csv = run_dir / "replay.csv"
    summary_csv = run_dir / "summary.csv"

    if algo == "prodmp":
        require_binary(LEARN_TEST_PRODMP)
        defaults = load_yaml(PRODMP_FEATURES_YAML)
        n_basis = args.n_basis if args.n_basis is not None else defaults.get("num_basis", 80)
        ridge_lambda = args.ridge_lambda if args.ridge_lambda is not None else defaults.get("ridge_lambda", 1e-10)
        window = args.window if args.window is not None else defaults.get("position_filter", {}).get("window_sec", 0.05)
        fix_goal = defaults.get("fix_goal_to_demo_endpoint", True) if args.fix_goal is None else args.fix_goal

        cmd = [
            LEARN_TEST_PRODMP, demo_csv, weights_yaml, replay_csv, summary_csv, label,
            str(n_basis), "-", "-", "-", "",
            "--lambda", str(ridge_lambda), "--window", str(window),
        ]
        if fix_goal:
            cmd.append("--fix-goal")
        # mp_tool has no legacy behaviour to preserve for its own output (unlike
        # learn_and_test_prodmp itself, whose default stays OFF for backward
        # compatibility): every NEW fit made through mp_tool embeds the fitted
        # orientation QuaternionDMP into weights.yaml via the unified format, so
        # weights.yaml alone is enough for ROS2 consumers/inspection - no
        # separate orientation_weights_yaml_path needed. Override with
        # --no-with-orientation to opt back out.
        if args.with_orientation:
            cmd.append("--with-orientation")
        cmd += _satellite_cli_args(args)
        run_cmd(cmd, label=label)
        print(f"[mp_tool] ProDMP fit: n_basis={n_basis} ridge_lambda={ridge_lambda} window={window} "
              f"fix_goal={fix_goal} with_orientation={args.with_orientation}")

    elif algo == "dmp":
        require_binary(LEARN_TEST_DMP)
        n_basis = args.n_basis if args.n_basis is not None else DMP_PROD_N_BASIS
        if args.fix_goal is not None:
            print("[mp_tool] NOTE: --fix-goal/--no-fix-goal has no effect for classic DMP (ProDMP-only concept); ignored.")

        # ridge_lambda / window overrides require an ad-hoc feature-config
        # YAML (learn_and_test_dmp has no --lambda/--window flags), same
        # approach the existing sweep scripts use.
        base_cfg = load_yaml(DMP_FEATURES_YAML)
        if args.ridge_lambda is not None:
            base_cfg.setdefault("regression", {})["ridge_lambda"] = args.ridge_lambda
            base_cfg["regression"]["method"] = base_cfg["regression"].get("method", "ridge")
        if args.window is not None:
            vf = base_cfg.setdefault("velocity_filter", {})
            vf["window_sec_1"] = args.window
            vf["window_sec_2"] = args.window

        if args.ridge_lambda is not None or args.window is not None:
            feature_config_path = run_dir / "dmp_feature_config.yaml"
            with open(feature_config_path, "w") as f:
                yaml.safe_dump(base_cfg, f)
        else:
            feature_config_path = DMP_FEATURES_YAML

        cmd = [
            LEARN_TEST_DMP, demo_csv, weights_yaml, replay_csv, summary_csv, label,
            str(n_basis), "-", "-", "-", str(feature_config_path),
        ]
        if any(getattr(args, n, None) is not None for n in SATELLITE_OPTIONS):
            print("[mp_tool] NOTE: the --satellite-* options apply to --algo prodmp only (demo_params file); ignored.")
        run_cmd(cmd, label=label)
        print(f"[mp_tool] Classic DMP fit: n_basis={n_basis} feature_config={feature_config_path}")
    else:
        raise ToolError(f"unknown algo: {algo}")

    row = read_last_summary_row(summary_csv)
    print(f"[mp_tool] RMSE overall: {row['rmse_overall_mm']} mm | "
          f"endpoint pos error: {row['endpoint_pos_error_mm']} mm | "
          f"endpoint orient error: {row['endpoint_orient_error_deg']} deg")

    plot_path = _plot_real_demo(demo_csv, replay_csv, run_dir)
    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, row, plot_path


def _plot_real_demo(demo_csv: Path, replay_csv: Path, run_dir: Path) -> Path:
    """Wraps 02_real_data/scripts/plot_real_demo.py unmodified. That script
    hardcodes its input paths (data/replay_from_yaml.csv relative to CWD) and
    output dir (plots/02_real_data/ relative to CWD) and calls plt.show(), so
    we run it with cwd=run_dir and MPLBACKEND=Agg (plt.show() is then a
    harmless no-op) and stage the replay CSV at the path it expects."""
    data_dir = run_dir / "data"
    data_dir.mkdir(exist_ok=True)
    staged_replay = data_dir / "replay_from_yaml.csv"
    staged_replay.write_bytes(replay_csv.read_bytes())

    env = dict(os.environ)
    env["MPLBACKEND"] = "Agg"
    run_cmd([sys.executable, PLOT_REAL_DEMO, demo_csv], cwd=run_dir, env=env, label="plot_real_demo.py")

    produced = run_dir / "plots" / "02_real_data" / "real_demo_plot.png"
    final_path = run_dir / "plot.png"
    if produced.exists():
        final_path.write_bytes(produced.read_bytes())
    return final_path


# ---------------------------------------------------------------------------
# sweep
# ---------------------------------------------------------------------------
def _parse_fix_others(s: str) -> dict:
    out = {}
    if not s:
        return out
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = float(v.strip())
    return out


def _run_prodmp_point(demo_csv, run_dir, tag, n_basis, ridge_lambda, window, fix_goal):
    weights = run_dir / f"weights_{tag}.yaml"
    replay = run_dir / f"replay_{tag}.csv"
    summary = run_dir / "summary.csv"
    cmd = [
        LEARN_TEST_PRODMP, demo_csv, weights, replay, summary, tag,
        str(n_basis), "-", "-", "-", "", "--lambda", str(ridge_lambda), "--window", str(window),
    ]
    if fix_goal:
        cmd.append("--fix-goal")
    run_cmd(cmd, label=tag)
    return read_last_summary_row(summary)


def _run_dmp_point(demo_csv, run_dir, tag, n_basis, ridge_lambda, window):
    weights = run_dir / f"weights_{tag}.yaml"
    replay = run_dir / f"replay_{tag}.csv"
    summary = run_dir / "summary.csv"
    base_cfg = load_yaml(DMP_FEATURES_YAML)
    base_cfg.setdefault("regression", {})["ridge_lambda"] = ridge_lambda
    base_cfg["regression"]["method"] = "ridge"
    vf = base_cfg.setdefault("velocity_filter", {})
    vf["window_sec_1"] = window
    vf["window_sec_2"] = window
    cfg_path = run_dir / f"cfg_{tag}.yaml"
    with open(cfg_path, "w") as f:
        yaml.safe_dump(base_cfg, f)
    cmd = [LEARN_TEST_DMP, demo_csv, weights, replay, summary, tag, str(n_basis), "-", "-", "-", str(cfg_path)]
    run_cmd(cmd, label=tag)
    return read_last_summary_row(summary)


def cmd_sweep(args):
    param = args.param
    algo = args.algo
    demo_csvs = [Path(p).resolve() for p in args.demo_csv]
    for p in demo_csvs:
        if not p.exists():
            raise ToolError(f"demo CSV not found: {p}")
    demo_csv = demo_csvs[0]
    if len(demo_csvs) > 1:
        print(f"[mp_tool] NOTE: sweep currently sweeps against the first demo CSV only: {demo_csv}")

    if algo == "prodmp":
        require_binary(LEARN_TEST_PRODMP)
    else:
        require_binary(LEARN_TEST_DMP)

    prod_defaults = load_yaml(PRODMP_FEATURES_YAML)
    fixed = {
        "n_basis": DMP_PROD_N_BASIS if algo == "dmp" else prod_defaults.get("num_basis", 80),
        "lambda": prod_defaults.get("ridge_lambda", 1e-10),
        "window": prod_defaults.get("position_filter", {}).get("window_sec", 0.05),
    }
    fixed.update(_parse_fix_others(args.fix_others))

    if args.grid:
        grid = [float(v) for v in args.grid.split(",")]
    else:
        grid = {"nbasis": DEFAULT_GRID_NBASIS, "lambda": DEFAULT_GRID_LAMBDA, "window": DEFAULT_GRID_WINDOW}[param]

    label = f"{algo}_{param}_{demo_csv.stem}"
    run_dir = make_run_dir("sweep", label)
    (run_dir / "summary.csv").unlink(missing_ok=True)

    rows = []
    for val in grid:
        n_basis = int(val) if param == "nbasis" else int(fixed["n_basis"])
        ridge_lambda = val if param == "lambda" else fixed["lambda"]
        window = val if param == "window" else fixed["window"]
        tag = f"{param}_{val:g}"
        if algo == "prodmp":
            row = _run_prodmp_point(demo_csv, run_dir, tag, n_basis, ridge_lambda, window,
                                     fix_goal=prod_defaults.get("fix_goal_to_demo_endpoint", True))
        else:
            row = _run_dmp_point(demo_csv, run_dir, tag, n_basis, ridge_lambda, window)
        row["_swept_value"] = val
        rows.append(row)

    production_value = {"nbasis": fixed["n_basis"], "lambda": fixed["lambda"], "window": fixed["window"]}[param]
    plot_path = _two_panel_sweep_plot(rows, param, production_value, run_dir, algo, demo_csv.name)

    print(f"[mp_tool] Sweep complete: {len(rows)} points over {param}, grid={grid}")
    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, rows, plot_path


def _two_panel_sweep_plot(rows, param, production_value, run_dir, algo, demo_name):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [r["_swept_value"] for r in rows]
    rmse = [float(r["rmse_overall_mm"]) for r in rows]
    endpoint = [float(r["endpoint_pos_error_mm"]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    log_x = param in ("lambda",)
    plot_fn1 = ax1.semilogx if log_x else ax1.plot
    plot_fn2 = ax2.semilogx if log_x else ax2.plot
    plot_fn1(xs, rmse, "o-", color="tab:blue")
    plot_fn2(xs, endpoint, "o-", color="tab:orange")
    ax1.axvline(production_value, color="k", linestyle="--", label=f"production = {production_value:g}")
    ax2.axvline(production_value, color="k", linestyle="--", label=f"production = {production_value:g}")
    ax1.set_xlabel(param)
    ax1.set_ylabel("RMSE overall [mm]")
    ax1.set_title("RMSE overall vs " + param)
    ax1.legend()
    ax2.set_xlabel(param)
    ax2.set_ylabel("Final (endpoint) position error [mm]")
    ax2.set_title("Endpoint error vs " + param)
    ax2.legend()
    fig.suptitle(f"{algo.upper()} sweep over {param} - demo: {demo_name}")
    fig.tight_layout()
    out_path = run_dir / "plot.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# diagnose
# ---------------------------------------------------------------------------
def cmd_diagnose(args):
    diag_type = args.diag_type
    demo_csv = Path(args.demo_csv).resolve()
    if not demo_csv.exists():
        raise ToolError(f"demo CSV not found: {demo_csv}")
    label = args.label or f"{diag_type}_{demo_csv.stem}"
    run_dir = make_run_dir("diagnose", label)

    if diag_type == "holdout":
        require_binary(LEARN_TEST_PRODMP_HOLDOUT)
        defaults = load_yaml(PRODMP_FEATURES_YAML)
        n_basis = args.n_basis if args.n_basis is not None else defaults.get("num_basis", 80)
        ridge_lambda = args.ridge_lambda if args.ridge_lambda is not None else defaults.get("ridge_lambda", 1e-10)
        window = args.window if args.window is not None else defaults.get("position_filter", {}).get("window_sec", 0.05)
        holdout_fraction = args.holdout_fraction
        weights = run_dir / "weights.yaml"
        summary = run_dir / "summary.csv"
        cmd = [LEARN_TEST_PRODMP_HOLDOUT, demo_csv, weights, summary, label,
               str(n_basis), str(ridge_lambda), str(window), str(holdout_fraction)]
        run_cmd(cmd, label=label)
        row = read_last_summary_row(summary)
        try:
            gap_ratio = float(row["rmse_holdout_mm"]) / float(row["rmse_in_sample_mm"])
        except (KeyError, ZeroDivisionError, ValueError):
            gap_ratio = float("nan")
        print(f"[mp_tool] Holdout: rmse_in_sample_mm={row.get('rmse_in_sample_mm')} "
              f"rmse_holdout_mm={row.get('rmse_holdout_mm')} gap_ratio={gap_ratio:.4f} "
              f"(gap_ratio is also printed directly to stdout above by the binary)")
        print(f"[mp_tool] Run directory: {run_dir}")
        return run_dir, row

    if diag_type == "condition":
        require_binary(EXTRACT_PRODMP_DIAGNOSTICS)
        defaults = load_yaml(PRODMP_FEATURES_YAML)
        n_basis = args.n_basis if args.n_basis is not None else defaults.get("num_basis", 80)
        ridge_lambda = args.ridge_lambda if args.ridge_lambda is not None else defaults.get("ridge_lambda", 1e-10)
        window = args.window if args.window is not None else defaults.get("position_filter", {}).get("window_sec", 0.05)
        cmd = [EXTRACT_PRODMP_DIAGNOSTICS, demo_csv, str(n_basis), str(ridge_lambda), str(window)]
        if defaults.get("fix_goal_to_demo_endpoint", True):
            cmd.append("--fix-goal")
        # The binary ignores argv entirely (see warning below) AND hardcodes a
        # path relative to its own build dir ("../../demo_raw_trajC.csv"), so
        # it must be run with cwd=OFFLINE_DIR (tools/dmp_offline_test) or it
        # aborts with a file-not-found error regardless of arguments passed.
        result = run_cmd(cmd, cwd=OFFLINE_DIR, label=label)
        out_file = run_dir / "condition_diagnostics.txt"
        out_file.write_text(result.stdout)
        print("[mp_tool] WARNING: extract_prodmp_diagnostics ignores ALL CLI arguments -- verified against its "
              "source (tools/dmp_offline_test/common/src/extract_prodmp_diagnostics.cpp: int main() with no argv "
              "handling at all). It always analyzes the hardcoded demo ../../demo_raw_trajC.csv with hardcoded "
              "num_basis=200, alpha_x=4.6, window_sec=0.20 -- regardless of what is passed here. The output below "
              "is NOT specific to your demo CSV or requested hyperparameters.")
        print(result.stdout)
        print(f"[mp_tool] Run directory: {run_dir}")
        return run_dir, result.stdout

    if diag_type == "perturbation":
        return _cmd_diagnose_perturbation(args, demo_csv, run_dir, label)

    raise ToolError(f"unknown diagnose type: {diag_type}")


def _cmd_diagnose_perturbation(args, demo_csv, run_dir, label):
    """Colored-noise perturbation stability study, generalized to an
    arbitrary demo CSV.

    NOTE: this does NOT literally invoke
    04_basis_sweep/scripts/run_perturbation_stability_colored_noise.py --
    that script hardcodes WORKSPACE_DIR, a fixed list of 3 specific demo
    files (Trajectory C/A/Reach Task), and a fixed output directory
    (slides_material/), with no argparse/CLI at all, so it cannot be pointed
    at an arbitrary --demo-csv. mp_tool reimplements only the (non-core)
    noise-generation/orchestration glue from that script -- Butterworth
    low-pass colored noise added to the raw demo positions, N realizations
    refit via the verified learn_and_test_prodmp binary -- while the actual
    DMP fitting/rollout is still done entirely by that unmodified C++
    binary, exactly as in the original script.
    """
    import pandas as pd
    import scipy.signal
    import scipy.ndimage

    require_binary(LEARN_TEST_PRODMP)
    defaults = load_yaml(PRODMP_FEATURES_YAML)
    n_basis = args.n_basis if args.n_basis is not None else defaults.get("num_basis", 80)
    ridge_lambda = args.ridge_lambda if args.ridge_lambda is not None else defaults.get("ridge_lambda", 1e-10)
    window = args.window if args.window is not None else defaults.get("position_filter", {}).get("window_sec", 0.05)
    n_realizations = args.n_realizations
    cutoff_hz = args.cutoff_hz
    seed_base = 42

    df = pd.read_csv(demo_csv)
    t = df["t"].values
    pos = df[["x", "y", "z"]].values
    dt_avg = float(np.mean(np.diff(t)))
    w_size = max(3, int(round(0.025 / dt_avg)))
    pos_smooth = scipy.ndimage.uniform_filter1d(pos, size=w_size, axis=0, mode="nearest")
    sigma_m = float(np.std(pos - pos_smooth))

    fs = 1.0 / dt_avg
    b, a = scipy.signal.butter(2, cutoff_hz, btype="low", fs=fs)

    summary_csv = run_dir / "summary.csv"
    max_weights, endpoint_errors = [], []
    for r in range(n_realizations):
        rng = np.random.RandomState(seed_base + r * 100)
        white = rng.normal(0.0, 1.0, size=pos.shape)
        colored = scipy.signal.filtfilt(b, a, white, axis=0)
        colored = colored - np.mean(colored, axis=0)
        colored = colored / np.std(colored) * sigma_m

        df_pert = df.copy()
        df_pert[["x", "y", "z"]] = pos + colored
        pert_csv = run_dir / f"perturbed_r{r}.csv"
        df_pert.to_csv(pert_csv, index=False)

        tag = f"r{r}"
        row = _run_prodmp_point(pert_csv, run_dir, tag, n_basis, ridge_lambda, window, fix_goal=True)
        max_weights.append(max_abs_weight_from_yaml(run_dir / f"weights_{tag}.yaml", "prodmp"))
        endpoint_errors.append(float(row["endpoint_pos_error_mm"]))

    max_weights = np.array(max_weights)
    endpoint_errors = np.array(endpoint_errors)
    print(f"[mp_tool] Perturbation stability ({n_realizations} realizations, sigma={sigma_m*1000:.4f} mm, "
          f"fc={cutoff_hz} Hz, n_basis={n_basis}, lambda={ridge_lambda}, window={window}):")
    print(f"  max|w|: mean={max_weights.mean():.3f} std={max_weights.std():.3f} "
          f"min={max_weights.min():.3f} max={max_weights.max():.3f}")
    print(f"  endpoint_pos_error_mm: mean={endpoint_errors.mean():.4f} std={endpoint_errors.std():.4f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.hist(max_weights, bins=min(8, n_realizations))
    ax1.set_title("max|w| across realizations")
    ax1.set_xlabel("max|w|")
    ax2.hist(endpoint_errors, bins=min(8, n_realizations))
    ax2.set_title("endpoint position error [mm]")
    ax2.set_xlabel("mm")
    fig.suptitle(f"Colored-noise perturbation stability - {demo_csv.name}")
    fig.tight_layout()
    plot_path = run_dir / "plot.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    with open(run_dir / "perturbation_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["realization", "max_abs_weight", "endpoint_pos_error_mm"])
        for i in range(n_realizations):
            w.writerow([i, max_weights[i], endpoint_errors[i]])

    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, {"max_weights": max_weights, "endpoint_errors": endpoint_errors}


# ---------------------------------------------------------------------------
# compare-dmp-prodmp
# ---------------------------------------------------------------------------
def cmd_compare(args):
    demo_csv = Path(args.demo_csv).resolve()
    if not demo_csv.exists():
        raise ToolError(f"demo CSV not found: {demo_csv}")
    label = args.label or demo_csv.stem
    run_dir = make_run_dir("compare", label)

    require_binary(LEARN_TEST_PRODMP)
    require_binary(LEARN_TEST_DMP)

    prod_defaults = load_yaml(PRODMP_FEATURES_YAML)
    dmp_weights = run_dir / "dmp_weights.yaml"
    dmp_replay = run_dir / "dmp_replay.csv"
    dmp_summary = run_dir / "dmp_summary.csv"
    run_cmd([LEARN_TEST_DMP, demo_csv, dmp_weights, dmp_replay, dmp_summary, "dmp",
              str(DMP_PROD_N_BASIS), "-", "-", "-", str(DMP_FEATURES_YAML)], label="dmp")
    dmp_row = read_last_summary_row(dmp_summary)

    prodmp_weights = run_dir / "prodmp_weights.yaml"
    prodmp_replay = run_dir / "prodmp_replay.csv"
    prodmp_summary = run_dir / "prodmp_summary.csv"
    n_basis = prod_defaults.get("num_basis", 80)
    ridge_lambda = prod_defaults.get("ridge_lambda", 1e-10)
    window = prod_defaults.get("position_filter", {}).get("window_sec", 0.05)
    cmd = [LEARN_TEST_PRODMP, demo_csv, prodmp_weights, prodmp_replay, prodmp_summary, "prodmp",
           str(n_basis), "-", "-", "-", "", "--lambda", str(ridge_lambda), "--window", str(window)]
    if prod_defaults.get("fix_goal_to_demo_endpoint", True):
        cmd.append("--fix-goal")
    # Embed the orientation QuaternionDMP that learn_and_test_prodmp already
    # fits internally into prodmp_weights.yaml (unified format), so the
    # produced file is consumable standalone (e.g. by prodmp_gazebo_executor_node
    # with no separate orientation_weights_yaml_path). Note this does NOT change
    # the endpoint_orient_error_deg/rmse numbers reported below - those were
    # already computed by learn_and_test_prodmp's internal (always-on) fixed
    # QuaternionDMP fit and read back from prodmp_summary.csv regardless of this
    # flag; --with-orientation only controls whether that already-fitted model
    # is also *persisted* into the YAML.
    cmd.append("--with-orientation")
    run_cmd(cmd, label="prodmp")
    prodmp_row = read_last_summary_row(prodmp_summary)

    dmp_max_w = max_abs_weight_from_yaml(dmp_weights, "dmp")
    prodmp_max_w = max_abs_weight_from_yaml(prodmp_weights, "prodmp")

    print(f"[mp_tool] Comparison table (demo: {demo_csv.name}):")
    header = f"{'':>12} {'RMSE overall [mm]':>18} {'endpoint pos err [mm]':>22} {'max|w|':>10}"
    print(header)
    print(f"{'DMP':>12} {dmp_row['rmse_overall_mm']:>18} {dmp_row['endpoint_pos_error_mm']:>22} {dmp_max_w:>10.3f}")
    print(f"{'ProDMP':>12} {prodmp_row['rmse_overall_mm']:>18} {prodmp_row['endpoint_pos_error_mm']:>22} {prodmp_max_w:>10.3f}")

    plot_path = _overlay_plot(demo_csv, dmp_replay, prodmp_replay, run_dir)

    with open(run_dir / "comparison_table.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["algo", "rmse_overall_mm", "endpoint_pos_error_mm", "endpoint_orient_error_deg", "max_abs_weight"])
        w.writerow(["dmp", dmp_row["rmse_overall_mm"], dmp_row["endpoint_pos_error_mm"], dmp_row["endpoint_orient_error_deg"], dmp_max_w])
        w.writerow(["prodmp", prodmp_row["rmse_overall_mm"], prodmp_row["endpoint_pos_error_mm"], prodmp_row["endpoint_orient_error_deg"], prodmp_max_w])

    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, dmp_row, prodmp_row, plot_path


def _load_pose_csv(path: Path):
    t, x, y, z = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); x.append(float(row["x"])); y.append(float(row["y"])); z.append(float(row["z"]))
    return np.array(t), np.array(x), np.array(y), np.array(z)


def _overlay_plot(demo_csv, dmp_replay, prodmp_replay, run_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    dt, dx, dy, dz = _load_pose_csv(demo_csv)
    pt, px, py, pz = _load_pose_csv(dmp_replay)
    qt, qx, qy, qz = _load_pose_csv(prodmp_replay)

    fig = plt.figure(figsize=(16, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.plot(dx, dy, dz, label="Demo", linewidth=2, color="black")
    ax3d.plot(px, py, pz, "--", label="DMP replay", color="tab:blue")
    ax3d.plot(qx, qy, qz, "--", label="ProDMP replay", color="tab:red")
    ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
    ax3d.set_title("3D trajectory: demo vs DMP vs ProDMP")
    ax3d.legend()

    axt = fig.add_subplot(1, 2, 2)
    axt.plot(dt, dx, color="tab:blue", label="demo x")
    axt.plot(dt, dy, color="tab:orange", label="demo y")
    axt.plot(dt, dz, color="tab:green", label="demo z")
    axt.plot(pt, px, "--", color="tab:blue", alpha=0.6, label="DMP x")
    axt.plot(pt, py, "--", color="tab:orange", alpha=0.6, label="DMP y")
    axt.plot(pt, pz, "--", color="tab:green", alpha=0.6, label="DMP z")
    axt.plot(qt, qx, ":", color="tab:blue", alpha=0.9, label="ProDMP x")
    axt.plot(qt, qy, ":", color="tab:orange", alpha=0.9, label="ProDMP y")
    axt.plot(qt, qz, ":", color="tab:green", alpha=0.9, label="ProDMP z")
    axt.set_xlabel("t [s]"); axt.set_ylabel("position [m]")
    axt.set_title("Position vs time")
    axt.legend(fontsize=7, ncol=3)

    fig.tight_layout()
    out_path = run_dir / "plot.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# goal-sweep
# ---------------------------------------------------------------------------
def cmd_goal_sweep(args):
    demo_csv = Path(args.demo_csv).resolve()
    if not demo_csv.exists():
        raise ToolError(f"demo CSV not found: {demo_csv}")
    algo = args.algo
    label = args.label or f"{algo}_{demo_csv.stem}"
    run_dir = make_run_dir("goalsweep", label)

    if args.offsets:
        print("[mp_tool] NOTE: --offsets was given, but the 5 goal offsets are hardcoded inside "
              "run_goal_generalization / run_prodmp_goal_generalization (C++ create5Goals()) and are NOT "
              "configurable via CLI or YAML without editing/rebuilding that binary, which mp_tool must not do. "
              "The offsets file is ignored; the binaries' built-in offsets are documented in mp_tool.py "
              "(PRODMP_GOAL_OFFSETS_M) and printed below for reference.")
        for name, off in PRODMP_GOAL_OFFSETS_M:
            print(f"    {name}: dx={off[0]*100:.0f}cm dy={off[1]*100:.0f}cm dz={off[2]*100:.0f}cm")

    if algo == "prodmp":
        require_binary(RUN_PRODMP_GOAL_GENERALIZATION)
        defaults = load_yaml(PRODMP_FEATURES_YAML)
        n_basis = args.n_basis if args.n_basis is not None else defaults.get("num_basis", 80)
        ridge_lambda = args.ridge_lambda if args.ridge_lambda is not None else defaults.get("ridge_lambda", 1e-10)
        window = args.window if args.window is not None else defaults.get("position_filter", {}).get("window_sec", 0.05)
        run_cmd([RUN_PRODMP_GOAL_GENERALIZATION, demo_csv, run_dir, label,
                  str(n_basis), str(ridge_lambda), str(window)], label=label)
        replay_orig = run_dir / "data" / f"{label}_replay_orig.csv"
        # plot_goal_generalization.py's load_goals_info() always expects an
        # 'err_orient_deg' column (written by the classic-DMP binary's
        # goals_info CSV, which reports orientation error). The ProDMP binary
        # is position-only and its goals_info CSV has no such column (see
        # run_prodmp_goal_generalization.cpp: "goal_id,name,gx,gy,gz,err_pos_mm").
        #
        # NOTE (unified ProDMP+orientation YAML format, added later): this
        # placeholder is NOT fixed by that feature. run_prodmp_goal_generalization
        # is a SEPARATE C++ binary from learn_and_test_prodmp - it never fits or
        # rolls out a QuaternionDMP at all (it only calls prodmp.setGoal() on the
        # POSITION model for each of the 5 new goals), so there is no orientation
        # fit here to embed/read from a unified file, and ProDMP's own orientation
        # goal is untouched by a position-only goal change - "orientation error
        # under a new position goal" isn't a quantity this tool computes. Giving
        # it a real value would require adding QuaternionDMP fitting+rollout to
        # run_prodmp_goal_generalization.cpp itself, which is out of scope here
        # (a modeling change to a binary not covered by this task - see final
        # report "deliberately not implemented").
        #
        # So: pad OUR OWN generated goals_info.csv (in this run's own data/ dir)
        # with a err_orient_deg=0.0 placeholder column so the (unmodified) plot
        # script can consume it -- ProDMP goal generalization here never
        # varies orientation, so 0.0 is honest, not fabricated data.
        goals_info_csv = run_dir / "data" / f"{label}_goals_info.csv"
        with open(goals_info_csv, newline="") as f:
            rows = list(csv.DictReader(f))
        if rows and "err_orient_deg" not in rows[0]:
            fieldnames = list(rows[0].keys()) + ["err_orient_deg"]
            for r in rows:
                r["err_orient_deg"] = "0.0"
            with open(goals_info_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerows(rows)
            print("[mp_tool] NOTE: padded ProDMP goals_info.csv with a placeholder err_orient_deg=0.0 column "
                  "(ProDMP goal generalization is position-only; see comment above) so plot_goal_generalization.py "
                  "can run unmodified.")
    else:
        require_binary(RUN_GOAL_GENERALIZATION)
        n_basis = args.n_basis if args.n_basis is not None else 100
        run_cmd([RUN_GOAL_GENERALIZATION, demo_csv, run_dir, label, str(n_basis)], label=label)
        replay_orig = run_dir / "data" / f"{label}_replay_orig.csv"

    plot_out_dir = run_dir / "plots"
    env = dict(os.environ)
    env["MPLBACKEND"] = "Agg"
    run_cmd([sys.executable, PLOT_GOAL_GENERALIZATION,
              "--demo", demo_csv, "--replay-orig", replay_orig,
              "--out-dir", plot_out_dir, "--label", label], env=env, label="plot_goal_generalization.py")

    print(f"[mp_tool] Goal generalization complete for {algo}. Plot(s) in {plot_out_dir}")
    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, plot_out_dir


# ---------------------------------------------------------------------------
# gazebo-collect
# ---------------------------------------------------------------------------
def cmd_gazebo_collect(args):
    """Collects the scattered outputs of a manual Gazebo eval run (bag dir +
    data/*.csv + plots/*/*.png, all written by the existing, unmodified
    scripts under tools/gazebo_cartesian_eval/scripts/) into a single
    runs/<timestamp>_gazebo_<run_name>/ directory, via symlinks only.

    Hard constraint: this NEVER moves, renames, copies, or modifies anything
    under tools/gazebo_cartesian_eval/ -- every original file/dir stays
    exactly where and how it is; only os.symlink() is used, and only inside
    the freshly created run dir under runs/.
    """
    run_name = args.run_name
    label = args.label or run_name
    run_dir = make_run_dir("gazebo", label)

    found = {}    # clean_name -> resolved original absolute path (str)
    missing = []  # list of (clean_name, reason)

    # 1) bag directory: tools/gazebo_cartesian_eval/bags/<run_name>/
    bag_dir = GAZEBO_BAGS_DIR / run_name
    if bag_dir.is_dir():
        target = bag_dir.resolve()
        os.symlink(target, run_dir / "bag")
        found["bag/"] = str(target)
    else:
        missing.append(("bag/", f"no directory at tools/gazebo_cartesian_eval/bags/{run_name}/"))

    # 2) data CSVs: tools/gazebo_cartesian_eval/data/<prefix>_<run_name>.csv
    #    Anchored suffix match (glob is anchored at both ends implicitly:
    #    "*_<run_name>.csv" only matches filenames whose LAST underscore-
    #    delimited remainder before ".csv" is exactly run_name), so e.g.
    #    run_name="trans_K200_D10" does not false-positive match
    #    "..._trans_K200_D100.csv".
    for csv_path in sorted(GAZEBO_DATA_DIR.glob(f"*_{run_name}.csv")):
        suffix = f"_{run_name}.csv"
        prefix = csv_path.name[: -len(suffix)]
        if prefix not in GAZEBO_DATA_MAP:
            continue  # not one of the 6 recognized categories; ignore silently
        clean_name, _desc = GAZEBO_DATA_MAP[prefix]
        target = csv_path.resolve()
        os.symlink(target, run_dir / clean_name)
        found[clean_name] = str(target)

    for prefix, (clean_name, _desc) in GAZEBO_DATA_MAP.items():
        if clean_name in found:
            continue
        reason = f"no file found matching data/{prefix}_{run_name}.csv"
        if prefix in GAZEBO_FULL_TEST_ONLY_PREFIXES:
            reason += " -- normal if record_bag.sh (tracking-only) was used instead of record_full_test_bag.sh"
        missing.append((clean_name, reason))

    # 3) plots: tools/gazebo_cartesian_eval/plots/<subdir>/*_<run_name>.png
    for subdir, (clean_name, _desc) in GAZEBO_PLOT_MAP.items():
        subdir_path = GAZEBO_PLOTS_DIR / subdir
        matches = sorted(subdir_path.glob(f"*_{run_name}.png")) if subdir_path.is_dir() else []
        if not matches:
            missing.append((clean_name, f"no PNG found matching plots/{subdir}/*_{run_name}.png"))
            continue
        for i, m in enumerate(matches):
            name = clean_name if len(matches) == 1 else clean_name.replace(".png", f"_{i}.png")
            target = m.resolve()
            os.symlink(target, run_dir / name)
            found[name] = str(target)

    # 4) optional cross-navigation link to an offline (fit/sweep/...) run
    offline_run_path = None
    if args.link_offline_run:
        offline_run_path = Path(args.link_offline_run).resolve()
        if not offline_run_path.exists():
            raise ToolError(f"--link-offline-run path does not exist: {offline_run_path}")
        os.symlink(offline_run_path, run_dir / "offline_run")

    index = {
        "run_name": run_name,
        "label": label,
        "type": "gazebo",
        "collected_at": datetime.now().isoformat(),
        "found_files": found,
        "missing": [{"name": n, "reason": r} for n, r in missing],
    }
    if offline_run_path is not None:
        index["offline_run"] = str(offline_run_path)
    with open(run_dir / "index.yaml", "w") as f:
        yaml.safe_dump(index, f, sort_keys=False)

    print(f"[mp_tool] gazebo-collect run_name={run_name}")
    print(f"[mp_tool] Found {len(found)} item(s):")
    for name, orig in found.items():
        print(f"    {name:22s} <- {orig}")
    if missing:
        print(f"[mp_tool] Missing {len(missing)} item(s) (NOT necessarily an error -- see reasons):")
        for name, reason in missing:
            print(f"    {name:22s} : {reason}")
    if offline_run_path is not None:
        print(f"[mp_tool] Linked offline_run -> {offline_run_path}")
    print(f"[mp_tool] Run directory: {run_dir}")
    return run_dir, found, missing


# ---------------------------------------------------------------------------
# list
# ---------------------------------------------------------------------------
def cmd_list(args):
    if not RUNS_DIR.exists():
        print("[mp_tool] No runs/ directory yet.")
        return []
    entries = []
    for run_dir in sorted(RUNS_DIR.iterdir()):
        if not run_dir.is_dir():
            continue
        name = run_dir.name
        parts = name.split("_", 2)
        date_part = parts[0] if len(parts) > 0 else "?"
        subcommand = parts[2].split("_")[0] if len(parts) > 2 else "?"

        # Run-type detection: gazebo-collect runs write index.yaml (and no
        # summary.csv, since they wrap no fitting binary); offline runs
        # (fit/sweep/diagnose/compare/goal-sweep) write *summary*.csv and
        # never index.yaml. Also check the naming convention as a fallback.
        index_yaml = run_dir / "index.yaml"
        is_gazebo = index_yaml.exists() or subcommand == "gazebo"

        if is_gazebo:
            run_type = "gazebo"
            headline = "n/a"
            if index_yaml.exists():
                try:
                    idx = load_yaml(index_yaml)
                    n_found = len(idx.get("found_files", {}))
                    n_missing = len(idx.get("missing", []))
                    rn = idx.get("run_name", "?")
                    headline = f"run_name={rn} found={n_found} missing={n_missing}"
                except Exception:
                    headline = "(index.yaml unreadable)"
            else:
                headline = "(no index.yaml found)"
        else:
            run_type = "offline"
            headline = "n/a"
            summary_candidates = sorted(run_dir.glob("*summary*.csv"))
            if summary_candidates:
                try:
                    row = read_last_summary_row(summary_candidates[0])
                    if "rmse_overall_mm" in row:
                        headline = f"rmse_overall={row['rmse_overall_mm']}mm"
                    elif "rmse_holdout_mm" in row:
                        headline = f"rmse_holdout={row['rmse_holdout_mm']}mm"
                except Exception:
                    headline = "(summary unreadable)"

        print(f"{date_part}  [{run_type:7s}]  {name:55s}  {headline}")
        entries.append((name, run_type, headline))
    return entries


# ---------------------------------------------------------------------------
# argparse wiring
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(prog="mp_tool", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    pf = sub.add_parser("fit", help="Fit a DMP or ProDMP on a demo CSV, replay, and plot.")
    pf.add_argument("demo_csv")
    pf.add_argument("--algo", choices=["dmp", "prodmp"], default="prodmp")
    pf.add_argument("--label", default=None)
    pf.add_argument("--n-basis", type=int, default=None)
    pf.add_argument("--ridge-lambda", type=float, default=None)
    pf.add_argument("--window", type=float, default=None)
    fg = pf.add_mutually_exclusive_group()
    fg.add_argument("--fix-goal", dest="fix_goal", action="store_true", default=None)
    fg.add_argument("--no-fix-goal", dest="fix_goal", action="store_false")
    wo = pf.add_mutually_exclusive_group()
    wo.add_argument("--with-orientation", dest="with_orientation", action="store_true", default=True,
                     help="--algo prodmp only: embed the fitted orientation QuaternionDMP into "
                          "weights.yaml (unified format). Default ON for mp_tool fits (mp_tool has "
                          "no legacy output to preserve, unlike learn_and_test_prodmp's own CLI "
                          "default of OFF).")
    wo.add_argument("--no-with-orientation", dest="with_orientation", action="store_false",
                     help="--algo prodmp only: write position-only weights.yaml (old/legacy shape).")
    pf.add_argument("--satellite-center", type=float, nargs=3, metavar=("X", "Y", "Z"), default=None,
                    help="--algo prodmp only: satellite center at the demo [m], world frame. Written to "
                         "<stem>_demo_params.yaml next to weights.yaml. The five --satellite-*/--grasp-point/"
                         "--phase-at-contact-deg options go ALL together or none.")
    pf.add_argument("--satellite-axis", type=float, nargs=3, metavar=("X", "Y", "Z"), default=None,
                    help="satellite spin axis at the demo (need not be normalized).")
    pf.add_argument("--satellite-omega-deg-s", type=float, default=None,
                    help="signed satellite angular velocity [deg/s] (clockwise = negative), converted to rad/s.")
    pf.add_argument("--grasp-point", choices=["kP0", "kP90", "kP180", "kP270"], default=None,
                    help="grasp point used in the demo.")
    pf.add_argument("--phase-at-contact-deg", type=float, default=None,
                    help="satellite phase theta at the contact [deg], CubeSatelliteModel convention.")
    pf.set_defaults(func=cmd_fit)

    ps = sub.add_parser("sweep", help="Sweep n_basis / lambda / window and plot RMSE + endpoint error vs param.")
    ps.add_argument("param", choices=["nbasis", "lambda", "window"])
    ps.add_argument("demo_csv", nargs="+")
    ps.add_argument("--algo", choices=["dmp", "prodmp"], default="prodmp")
    ps.add_argument("--grid", default=None, help="Comma-separated values, e.g. 10,20,40,80")
    ps.add_argument("--fix-others", default="", help="Comma-separated k=v overrides, e.g. n_basis=80,window=0.05")
    ps.set_defaults(func=cmd_sweep)

    pd_ = sub.add_parser("diagnose", help="Holdout validation, perturbation stability, or condition-number diagnostics.")
    pd_.add_argument("diag_type", choices=["holdout", "perturbation", "condition"])
    pd_.add_argument("demo_csv")
    pd_.add_argument("--label", default=None)
    pd_.add_argument("--n-basis", type=int, default=None)
    pd_.add_argument("--ridge-lambda", type=float, default=None)
    pd_.add_argument("--window", type=float, default=None)
    pd_.add_argument("--holdout-fraction", type=float, default=0.2)
    pd_.add_argument("--n-realizations", type=int, default=8)
    pd_.add_argument("--cutoff-hz", type=float, default=0.5)
    pd_.set_defaults(func=cmd_diagnose)

    pc = sub.add_parser("compare-dmp-prodmp", help="Fit both DMP and ProDMP with production hyperparameters and compare.")
    pc.add_argument("demo_csv")
    pc.add_argument("--label", default=None)
    pc.set_defaults(func=cmd_compare)

    pg = sub.add_parser("goal-sweep", help="Fit once, test 5 new goals (goal generalization).")
    pg.add_argument("demo_csv")
    pg.add_argument("--algo", choices=["dmp", "prodmp"], default="prodmp")
    pg.add_argument("--label", default=None)
    pg.add_argument("--offsets", default=None, help="Not supported: offsets are hardcoded in the C++ binary; see note printed at runtime.")
    pg.add_argument("--n-basis", type=int, default=None)
    pg.add_argument("--ridge-lambda", type=float, default=None)
    pg.add_argument("--window", type=float, default=None)
    pg.set_defaults(func=cmd_goal_sweep)

    pgz = sub.add_parser(
        "gazebo-collect",
        help="Collect an existing manual Gazebo eval run's scattered outputs (bag/CSVs/plots) into "
             "runs/<timestamp>_gazebo_<run_name>/ via symlinks (read-only, nothing under "
             "tools/gazebo_cartesian_eval/ is moved/renamed/modified).",
    )
    pgz.add_argument("run_name", help="The <run_name> originally passed to record_bag.sh / record_full_test_bag.sh.")
    pgz.add_argument("--label", default=None, help="Defaults to run_name.")
    pgz.add_argument("--link-offline-run", default=None, metavar="PATH",
                      help="Path to a runs/<...> offline run dir (fit/sweep/...) to cross-link as offline_run/ "
                           "for navigating between the DMP fit that produced the weights used in this Gazebo run "
                           "and its Gazebo eval outputs.")
    pgz.set_defaults(func=cmd_gazebo_collect)

    pl = sub.add_parser("list", help="List existing runs under runs/ with a one-line summary each.")
    pl.set_defaults(func=cmd_list)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except ToolError as e:
        print(f"[mp_tool] ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
