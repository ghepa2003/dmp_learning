# mp_tool

A thin Python CLI orchestrator for the DMP / ProDMP offline pipeline in
`tools/dmp_offline_test/`. It does **not** reimplement any fitting/rollout
math: every numeric result comes from the existing, already-verified C++
binaries in `tools/dmp_offline_test/build/` and
`tools/dmp_offline_test/06_goal_generalization/plots/build/`, invoked as
subprocesses. mp_tool only handles argument wiring, default hyperparameters
(read from the production YAML configs), output organization under `runs/`,
and plotting.

Run it with `python3 tools/mp_tool/mp_tool.py <subcommand> ...` from
anywhere (paths are resolved relative to the repo root, not your CWD).

Every invocation writes to a fresh, non-overwriting
`runs/<timestamp>_<subcommand>_<label>/` directory at the repo root.

## Subcommands

### `fit` - learn once, replay, plot

```bash
python3 tools/mp_tool/mp_tool.py fit demo_raw_trajA.csv --algo prodmp --label trajA
```

Defaults for `--algo prodmp` come from `src/haptic_dmp_learning/config/prodmp_features.yaml`
(`num_basis=80`, `ridge_lambda=1e-10`, `window=0.05`, `fix_goal=true`);
defaults for `--algo dmp` come from `src/haptic_dmp_learning/config/dmp_features.yaml`
plus `n_basis=200` (the live_demo_recorder_node production value in
`params.yaml`). Any of `--n-basis/--ridge-lambda/--window/--fix-goal` overrides
the YAML default for that run only (production YAML files are never written).
Writes `weights.yaml`, `replay.csv`, `summary.csv`, `plot.png` and prints the
RMSE/endpoint error read back from the freshly produced `summary.csv`.

For `--algo prodmp` the run directory also gets `weights_demo_params.yaml`, written by
`learn_and_test_prodmp` right after the weights (see `core/demo_params.hpp`): tau, contact,
demo displacement, the SHA-256 of `weights.yaml`, the fit configuration used and, optionally,
the satellite state at the demo. It is a separate file (never mistaken for a weights file) and
`weights.yaml` itself is unchanged. If a `<demo>_joint_states.csv` sits next to the demo CSV its
first/last rows fill the start/end joints, otherwise they are `null`. Optional satellite options,
ALL together or none (`--algo prodmp` only):

```bash
python3 tools/mp_tool/mp_tool.py fit demo.csv --algo prodmp \
    --satellite-center 0.75 0 0.35 --satellite-axis 0 0 1 --satellite-omega-deg-s -2 \
    --grasp-point kP270 --phase-at-contact-deg -60
```

For `--algo prodmp`, `weights.yaml` embeds the fitted orientation
QuaternionDMP as an inline `quaternion_dmp:` section by default (unified
ProDMP+orientation format, passed through to `learn_and_test_prodmp` as
`--with-orientation`) - mp_tool has no legacy output shape of its own to
preserve, unlike `learn_and_test_prodmp`'s own CLI, whose default stays OFF
for byte-identical backward compatibility. `weights.yaml` alone is therefore
enough to drive `prodmp_gazebo_executor_node` with no separate
`orientation_weights_yaml_path`. Pass `--no-with-orientation` for the old
position-only shape.

### `sweep` - 1D hyperparameter sweep with a two-panel plot

```bash
python3 tools/mp_tool/mp_tool.py sweep nbasis demo_raw_trajA.csv --algo prodmp
python3 tools/mp_tool/mp_tool.py sweep lambda demo_raw_trajA.csv --algo prodmp --grid 1e-10,1e-8,1e-6
```

Sweeps `nbasis` / `lambda` / `window` over a grid (default grids are the ones
already used in the existing `04_basis_sweep/scripts/run_*sweep*.sh` scripts:
nbasis = `[5,10,...,500]`, lambda = `[1e-9..1e0]`, window = `[0.01..0.20]`),
holding the other hyperparameters at their production value (override with
`--fix-others n_basis=80,window=0.05`). Produces `plot.png` with two panels
(RMSE overall, final/endpoint position error vs the swept parameter), each
marking the production value with a vertical dashed line.

### `diagnose` - holdout / perturbation / condition-number checks

```bash
python3 tools/mp_tool/mp_tool.py diagnose holdout demo_raw_trajA.csv
python3 tools/mp_tool/mp_tool.py diagnose perturbation demo_raw_trajA.csv --n-realizations 8
python3 tools/mp_tool/mp_tool.py diagnose condition demo_raw_trajA.csv
```

- `holdout` wraps `learn_and_test_prodmp_holdout` directly (strictly
  positional CLI, no YAML support - built as documented in the inventory).
- `perturbation` does **not** literally invoke
  `run_perturbation_stability_colored_noise.py` - see "Not cleanly wrapped"
  below. It reimplements only the (non-core) noise-generation/orchestration
  glue from that script, generalized to an arbitrary `<demo.csv>`, and still
  performs every actual fit via the unmodified `learn_and_test_prodmp` binary.
- `condition` wraps `extract_prodmp_diagnostics` - but see the warning it
  prints: this binary's `main()` takes no CLI arguments at all and always
  analyzes a hardcoded `demo_raw_trajC.csv` with hardcoded hyperparameters
  (verified against its source). mp_tool still invokes it (matching the
  inventory's documented syntax) but loudly flags that the output is not
  specific to your demo/hyperparameters.

### `compare-dmp-prodmp` - fit both, side by side

```bash
python3 tools/mp_tool/mp_tool.py compare-dmp-prodmp demo_raw_trajA.csv --label trajA
```

Fits DMP and ProDMP on the same demo with each algorithm's own production
hyperparameters, prints a comparison table (RMSE overall / endpoint error /
max|w|), and produces an overlay `plot.png` (3D trajectory + position-vs-time,
demo vs DMP replay vs ProDMP replay). The `endpoint_orient_error_deg` /
`rmse` numbers in the comparison table and `comparison_table.csv` were always
real (`learn_and_test_prodmp` fits its internal fixed-config QuaternionDMP
and reports real orientation metrics in `summary.csv` regardless of any
flag) - `compare-dmp-prodmp` additionally passes `--with-orientation` so
`prodmp_weights.yaml` also carries that already-fitted QuaternionDMP inline
(unified format), for consistency with `fit` and so the produced file is
consumable standalone.

### `goal-sweep` - fit once, test 5 new goals

```bash
python3 tools/mp_tool/mp_tool.py goal-sweep demo_raw_trajA.csv --algo prodmp --label trajA
```

Wraps `run_prodmp_goal_generalization` / `run_goal_generalization` and then
`plot_goal_generalization.py`. The 5 goal offsets are **hardcoded** in the
C++ binaries (`create5Goals()` in
`06_goal_generalization/scripts/run_prodmp_goal_generalization.cpp`) and are
not configurable; `--offsets` is accepted but ignored with a printed note
(mp_tool prints the 5 hardcoded offsets for reference). For `--algo prodmp`,
the binary's `goals_info.csv` has no orientation-error column (it's
position-only), while the unmodified `plot_goal_generalization.py` always
expects one; mp_tool pads that column with `0.0` in its own run-local copy
of the CSV before plotting (see comment in `mp_tool.py::cmd_goal_sweep`) -
no source script or binary is modified. **Not fixed by the unified
ProDMP+orientation YAML format**: `run_prodmp_goal_generalization` is a
separate C++ binary from `learn_and_test_prodmp` that never fits or rolls
out a QuaternionDMP at all (it only calls `prodmp.setGoal()` on the position
model for each new goal), so there is no orientation fit to embed/read here,
and "orientation error under a new position-only goal" isn't a quantity this
tool computes - giving it a real value would mean adding QuaternionDMP
fitting to `run_prodmp_goal_generalization.cpp` itself, out of scope for
this task (see the I/O-layer task's final report).

### `gazebo-collect` - collect a manual Gazebo eval run's scattered outputs

```bash
python3 tools/mp_tool/mp_tool.py gazebo-collect trajA_ridge_filter_n100
python3 tools/mp_tool/mp_tool.py gazebo-collect nullspace_K3_rep1 \
    --link-offline-run runs/20260911_171108_fit_testA
```

The Gazebo eval pipeline (`tools/gazebo_cartesian_eval/`) is separate, manual,
and out of scope for this tool to drive - it's already covered end-to-end by
`record_bag.sh` / `record_full_test_bag.sh` / `run_full_test_analysis.sh`.
Those scripts write outputs scattered across three fixed locations keyed by
`<run_name>`:

- `tools/gazebo_cartesian_eval/bags/<run_name>/` (the rosbag directory)
- `tools/gazebo_cartesian_eval/data/*_<run_name>.csv` (varying prefixes:
  `target_aligned_`, `actual_pose_`, `force_`, `grasp_state_`, `gripper_cmd_`,
  `target_odom_`)
- `tools/gazebo_cartesian_eval/plots/*/*_<run_name>.png` (subfolders
  `02_gazebo_tracking/`, `03_gain_sweep/`, `04_grasp_test/`, depending on
  which evaluation script produced the plot)

`gazebo-collect` does **not** re-run or wrap any of those scripts - it is a
purely after-the-fact, read-only collection step. Given `<run_name>`, it
globs for the three categories above (suffix-anchored on `<run_name>` so
e.g. `trans_K200_D10` never false-matches `..._trans_K200_D100.csv`) and
creates `runs/<timestamp>_gazebo_<run_name>/` containing only **symlinks**
with clean names pointing at the exact original files/dirs - nothing under
`tools/gazebo_cartesian_eval/` is ever moved, renamed, copied, or modified
(verified: whole-tree md5sum/mtime snapshots before and after are byte-
identical). Clean-name mapping:

| Original | Symlink |
| :--- | :--- |
| `bags/<run_name>/` | `bag/` |
| `data/target_aligned_<run_name>.csv` | `tracking_target.csv` |
| `data/actual_pose_<run_name>.csv` | `tracking_actual.csv` |
| `data/force_<run_name>.csv` | `force.csv` |
| `data/grasp_state_<run_name>.csv` | `grasp_state.csv` |
| `data/gripper_cmd_<run_name>.csv` | `gripper_cmd.csv` |
| `data/target_odom_<run_name>.csv` | `target_odom.csv` |
| `plots/02_gazebo_tracking/*_<run_name>.png` | `plot_tracking.png` |
| `plots/03_gain_sweep/*_<run_name>.png` | `plot_gain_sweep.png` |
| `plots/04_grasp_test/*_<run_name>.png` | `plot_grasp.png` |

Missing categories (e.g. no `force`/`grasp_state`/`gripper_cmd`/
`target_odom` CSVs, which only `record_full_test_bag.sh` produces - plain
`record_bag.sh` only records tracking) are **not** treated as errors: they
are listed in the printed summary and in `index.yaml` under `missing`, each
with a plain-language reason. `index.yaml` also records `run_name`, the
collection timestamp, and the `found_files` clean-name -> original-path
mapping. Pass `--link-offline-run runs/<offline_folder>` to additionally
symlink `offline_run -> <that path>` inside the new run dir, for navigating
between the DMP/ProDMP fit that produced the weights used in a Gazebo run
and that run's evaluation outputs.

### `list`

```bash
python3 tools/mp_tool/mp_tool.py list
```

Lists every run under `runs/` with a one-line summary, distinguishing
`[offline]` runs (fit/sweep/diagnose/compare-dmp-prodmp/goal-sweep - headline
is the RMSE/holdout metric read back from `summary.csv`) from `[gazebo ]`
runs (`gazebo-collect` - headline is `run_name=... found=N missing=M`, since
these have no fitting binary and thus no RMSE to report). Run type is
detected by the presence of `index.yaml` (gazebo) vs `*summary*.csv`
(offline).

## Verification

For every subcommand, the exact same binary/script was hand-invoked outside
mp_tool with identical hyperparameters on the same demo CSV
(`demo_raw_trajA.csv`), and the numeric outputs were diffed against mp_tool's
run. All matched exactly (see the assistant's final report for the actual
numbers). mp_tool adds no numeric transformation between a binary's stdout/
CSV output and what it reports back.

## Not cleanly wrapped (and why)

- **`run_perturbation_stability_colored_noise.py`** - hardcodes
  `WORKSPACE_DIR`, a fixed list of 3 specific demo files (Trajectory
  C/A/"Reach Task"), and a fixed output directory (`slides_material/`), with
  no `argparse`/CLI whatsoever (confirmed by reading its source - no
  `sys.argv` or `argparse` usage at all). It cannot be pointed at an
  arbitrary `<demo.csv>` without editing the script. `mp_tool diagnose
  perturbation` instead reimplements its (non-core) noise-generation loop
  generalized to any demo CSV, while still doing every actual fit through
  the unmodified `learn_and_test_prodmp` binary.
- **`extract_prodmp_diagnostics`** - its `main()` (verified from
  `tools/dmp_offline_test/common/src/extract_prodmp_diagnostics.cpp`) takes
  no arguments at all; the inventory's documented CLI signature does not
  match the actual binary. It always loads a hardcoded
  `../../demo_raw_trajC.csv` (path relative to `tools/dmp_offline_test/`,
  so mp_tool also has to run it with that fixed `cwd`) with hardcoded
  `n_basis=200`, `window_sec=0.20`. mp_tool still wraps it (per the
  inventory's documented syntax and because it is a real, working binary)
  but prints an explicit warning every time that its output ignores whatever
  demo/hyperparameters you passed.
- **`plot_nbasis_study.py` / `plot_ridge_lambda_sweep.py`** - not used by
  `mp_tool sweep`. Their multi-series CSV/CLI shape (built for the shell
  scripts' many-variant, many-seed studies) doesn't match the spec's
  single-sweep "two-panel plot with the production value marked" ask, so
  `mp_tool sweep` renders its own small two-panel plot directly from the
  `summary.csv` rows it already collected (still 100% real binary output,
  just plotted differently).
- **The `06_goal_generalization` 5 goal offsets** - not wrapped as a
  configurable parameter (`--offsets`) because they are hardcoded in C++
  (`create5Goals()`) and mp_tool must not modify or rebuild binary numeric
  logic to parametrize them. Documented in `mp_tool.py`
  (`PRODMP_GOAL_OFFSETS_M`) and printed to the user instead.
