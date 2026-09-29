# Design Notes

Short technical notes on design decisions / non-issues that came up during
investigation but did not warrant (or did not need) a code change. These
supplement, and do not replace, any existing documentation.

## Module map — grasp selection pipeline (as of 2026-09-29)

**Dependency direction** (`package.xml`/`CMakeLists.txt`): `satellite_grasp_planner` depends
on `haptic_dmp_learning` and `franka_cartesian_control`; neither of those depends on
`satellite_grasp_planner` or on each other. Constraint: code inside `haptic_dmp_learning`
(e.g. `prodmp_gazebo_executor_node`) cannot link the planner library without introducing a
dependency cycle.

**Module table** (role per each header's own top comment):

| Package/module | Role |
|---|---|
| `haptic_dmp_learning/core/prodmp` | Integral-form ProDMP for 3D Cartesian position, "level 1" variant |
| `haptic_dmp_learning/core/cube_satellite_model` | Pure geometry of the cube-shaped target satellite (replaces the ring model) |
| `haptic_dmp_learning/core/grasp_roll` | Pure helpers for grasp roll psi about the TCP approach axis |
| `haptic_dmp_learning/core/grasp_cost` | Pure functions for the grasp-candidate cost (SI units); builds surface velocity, combines normalized terms into a scalar to minimize |
| `haptic_dmp_learning/core/math_utils` | Header-only shared math helpers (angle wrap, deg/rad, SLERP) |
| `haptic_dmp_learning/core/demo_params` | Demo-parameters file read by the on-board optimizer next to the ProDMP weights file |
| `haptic_dmp_learning/core/joint_states_csv_io` | Buffering/CSV I/O for `/joint_states` recorded alongside a demo |
| `franka_cartesian_control/core/robot_model` | Rigid-body kinematic/dynamic model of the Panda/FR3 (Pinocchio-backed) |
| `satellite_grasp_planner/core/selection_inputs` | Assembles everything `selectGrasp()` needs from the demo-params file + current satellite measurement |
| `satellite_grasp_planner/core/candidate_scan` | Offline scan of candidates (grasp point k x phase theta): ProDMP EE velocity vs. satellite surface velocity + goal term. No IK, no RobotModel, no collision |
| `satellite_grasp_planner/core/candidate_eval` | Full evaluation of one candidate: kinematic feasibility of the rollout + arm/satellite collision at the final configuration |
| `satellite_grasp_planner/core/grasp_selection` | Selects (k, theta, psi): candidate scan followed by full evaluations in partial-cost order, stopped by a bound |
| `satellite_grasp_planner/core/launch_plan` | Pure functions to plan launch time from a satellite measurement and decide whether reselection is needed |
| `satellite_grasp_planner/core/manipulability` | Translational manipulability index (product of singular values of the 3xN Jacobian block) |
| `satellite_grasp_planner/core/joint_path_simulator` | Simulates a Cartesian task-space trajectory via `RobotModel`/`VelocityIkSolver` |

**Data flow** (per header doc comments): `selection_inputs::buildSelectionInputs` (reads
`demo_params` + URDF + `SatelliteSnapshot`) -> `selectGrasp()` (`scanCandidates`, then
`evaluateCandidate` = `checkKinematicFeasibility` + `checkSatelliteCollision`, best-first
with a stopping bound) -> `planLaunch()`. This is the documented, but not yet wired-up, data
flow: `selectGrasp()` (declared `grasp_selection.hpp`) is called only from `TEST(...)` bodies
in `test/test_grasp_selection.cpp` (`GraspSelectionTest.*`, `SatelliteSelectionProbe.*`,
`SatelliteSelectionFromDemoParamsProbe.*`); `planLaunch()` (declared `launch_plan.hpp`) is
called only from `TEST(...)` bodies in `test/test_launch_plan.cpp` (`LaunchPlanPhase.*`,
`LaunchPlanPlan.*`, `LaunchPlanReselection.*`, `LaunchPlanThrows.*`). No file outside those two
test binaries calls either function, and no single test body calls both.

**Superseded / legacy**: `satellite_grasp_planner/core/phase_selector` is still compiled into
the core library and has its own test (`test/test_phase_selector.cpp`), but is not referenced
by `grasp_selection.cpp`/`selection_inputs.cpp`/`candidate_scan.cpp`/`candidate_eval.cpp` (the
current pipeline). `haptic_dmp_learning/core/satellite_intercept.hpp` (`anchorAndRotate`,
`PhaseTracker`, ...) is still used by `prodmp_gazebo_executor_node`'s "continuous" mode.
Planned for removal together with the ring code (author's plan, not tracked in the repo).

**Selected invariants** (`file` — rule):
- `cube_satellite_model.hpp` (`CubeSatelliteModel`) — `theta_rad` is an explicit argument of
  every query; "no temporal state", queries are pure functions of `(k, theta_rad, params_)`.
- `launch_plan.hpp` (`SatelliteSnapshot`) — `omega_rad_s` is SIGNED (CCW seen from +axis
  positive, CW negative; theta decreases in time when omega < 0); degrees appear only at the
  CLI level (`mp_tool.py --satellite-omega-deg-s`). `needsReselection`/`planLaunch` throw
  `std::invalid_argument` on `omega == 0`. `launchDelay` (`candidate_scan.hpp/.cpp`) =
  `(theta* - theta_now)/omega - tau_contact`, reduced mod `T = 2*pi/|omega|` into
  `[min_delay, min_delay + T)`.
- `prodmp.hpp`/`prodmp.cpp` (`ProDMP::velocityForCandidateGoal`) — `const`; requires `s_ > 0`
  (throws `std::logic_error` otherwise, call after `step()`); takes an ABSOLUTE goal even when
  `relativeGoal()` is true. In `step()`, the goal enters only through `goal_param_` times a
  closed-form response computed from `s = 0`; the `p1_accum_`/`p2_accum_` quadrature terms do
  not depend on the goal. Consequently `setGoal()` called between `step()` calls changes
  position/velocity as if that goal had been in force from the start, while
  `velocityForCandidateGoal` is unaffected by when `setGoal()` was called. An earlier working
  note (2026-09-25, not in the repo) suggested that mixing `setGoal()` with the queries gives
  silently wrong results; the 2026-09-29 probe did not reproduce it. `setInitialConditions()`
  (via `resetIntegrationState()`) resets `s_` and the accumulators to zero.
- `robot_model.hpp` / `kinematic_feasibility.hpp` / `grasp_selection.hpp` (`RobotModel`) —
  PIMPL-cached kinematics, no mutex; `grasp_selection.hpp` states "RobotModel is not
  thread-safe" and recommends one instance per thread; `kinematic_feasibility.hpp` documents
  that `checkKinematicFeasibility` mutates the shared model via `update()` and must not be
  called concurrently on the same instance. `checkSatelliteCollision` requires the caller to
  have called `update()` first (`satellite_collision.hpp`); `evaluateCandidate`
  (`candidate_eval.cpp`) does `update(kin.q_final, ...)` explicitly before calling it.
- `demo_params.hpp`/`.cpp` — binds itself to one weights file by name + SHA-256 of its bytes;
  `verifyWeightsAlignment` throws `std::runtime_error` on missing file or hash mismatch;
  `sha256Hex`/`sha256FileHex` are self-implemented (no libcrypto). All YAML keys required;
  optional entries (`demo.t_contact_s`, `demo.start_joints_rad`, `demo.end_joints_rad`,
  `satellite_at_demo`) must be `null`, not absent; unknown keys tolerated; `axis` is stored as
  given (`norm > 1e-9` required by the reader; consumers such as `CubeSatelliteModel`
  normalize it). Written as `<stem>_demo_params.yaml` by `learn_and_test_prodmp.cpp`. Exit
  code 3 = weights saved, demo_params write failed; `NonIncreasingTimestampsError` only prints
  a warning, writes no file, exits 0.
- `satellite_grasp_planner/test/probe_gate.hpp` — long probes are skipped unless
  `GRASP_RUN_PROBES` is set to something other than `""`/`"0"`; about 25 minutes per full run,
  reported 2026-09-28, not re-measured.
- `grasp_cost.hpp` (`meanSurfaceSpeedSquared`) — mean over the 4 grasp points at theta = 0 of
  `|omega * axis x r|^2`, `r` from `graspPoseAt(k, 0) - cubeCenterWorld()`; independent of
  theta. MEASURED 2026-09-29 (default `CubeSatelliteModel::Params`: side 0.20 m, standoff
  0.08 m, collar radius 0.05 m; omega = -2 deg/s; re-derived here directly from the formulas
  above, not rerun as an external probe): perpendicular distance from the axis 0.186815 m
  (kP0/kP180) and 0.18 m (kP90/kP270); `|v|^2` = 4.25246e-5 and 3.94784e-5 m^2/s^2, mean
  4.10015e-5; hand-still `e_v_hat = t.e_v/ref.e_v_ref` = 1.037 (kP0/kP180) and 0.963
  (kP90/kP270) — geometry only, cancels out `omega`.

**Offline fit tooling**:
- `tools/dmp_offline_test/build/learn_and_test_prodmp` mtime 2026-09-29 09:49, vs.
  `learn_and_test_prodmp.20260911.bak` mtime 2026-09-11 17:32 (`ls -la`). The three
  `run_{nbasis,filter_window,ridge_lambda}_sweep_prodmp.sh` scripts share an identical g++
  line and rebuild only if one of 8 sources is newer (`-nt`) than the binary; `dmp.cpp` and
  `metrics.cpp` are compiled in but excluded from that `-nt` list.
- `demo_params.cpp` calls `joint_states_csv_io::deriveJointStatesCsvPath`/`readJointStatesCsv`;
  consistent with the reported link failure without `joint_states_csv_io.cpp` on the link
  line (not independently rebuilt here — read-only investigation).
- The sweep scripts (`set -euo pipefail`) `rm -f` their summary CSV; `mp_tool.py`'s
  `run_cmd()` calls `sys.exit(1)` on non-zero subprocess exit.
- `mp_tool.py` defaults: `n_basis=80`, `ridge_lambda=1e-10`, `position_filter.window_sec=0.05`,
  `fix_goal_to_demo_endpoint=True`, `--with-orientation` default `True`.
- `sha256sum` reproduced exactly (2026-09-29):
  `runs/20260914_150515_..._w0.05/weights.yaml` =
  `03d4bc158bbaf652f135d6c0dd736db77ef67fab7a2da4c717ab5bbd86c7ac3a`; its
  `weights_demo_params.yaml` =
  `883ffb2ffdaf8d9d4f0581c092cace8ae40037d4663a0cf5bbb4196228d9af6c`;
  `runs/20260915_090358_fit_reach_task_baseline_prodmp/weights.yaml` has the identical hash.

## 2026-09-11 — Quaternion normalization in `learn_and_test_dmp.cpp`'s local CSV loader

**Context**: `demo_csv_io::readDemoCsv` (used by `learn_and_test_prodmp.cpp`)
normalizes the quaternion read from each CSV row
(`Eigen::Quaterniond(...).normalized()`). The local loader inside
`tools/dmp_offline_test/common/src/learn_and_test_dmp.cpp` does **not**
normalize (`s.orientation = Eigen::Quaterniond(vals[4], vals[5], vals[6], vals[7]);`).
This matters in principle because `QuaternionDMP::log_q(q)` computes
`theta = arccos(qw)`, which requires `|qw| <= 1` — only guaranteed for a
truly unit quaternion.

**Diagnosis performed** (read-only investigation, standalone Python scripts,
no production code touched):

1. Measured the deviation of the raw (pre-normalization) quaternion norm
   from 1.0, `|1 - sqrt(qw^2+qx^2+qy^2+qz^2)|`, per row, for the three demo
   CSVs at the repo root:

   | demo | rows | max dev | mean dev | p50 | p95 | p99 |
   |---|---|---|---|---|---|---|
   | demo_raw_trajA.csv | 62633 | 6.932e-7 | 2.357e-7 | 2.097e-7 | 5.382e-7 | 6.401e-7 |
   | demo_raw_trajC.csv | 68245 | 7.675e-7 | 2.356e-7 | 1.952e-7 | 5.592e-7 | 6.635e-7 |
   | reach_task_baseline.csv | 60900 | 6.907e-7 | 2.393e-7 | 2.082e-7 | 5.491e-7 | 6.437e-7 |

   All three CSVs are already unit-norm to ~1e-6, i.e. essentially
   float32-precision-limited text (likely round-tripped through a
   single-precision device/driver before being written as text). There is
   no evidence in these datasets of quaternions meaningfully off unit norm.

2. Refit classic DMP (`tools/dmp_offline_test/build/learn_and_test_dmp`,
   config: n_basis=200, alpha_x=4.6, alpha_z=25, beta_z=6.25, ridge
   regression with lambda=1e-6, velocity filter window 0.20s/0.20s — the
   exact config recorded in the already-verified `real_trajC_nbasis200.yaml`
   / `dmp_weights_trajC.yaml` weight files) on each demo CSV as-is
   (non-normalized loader, unchanged) and again on a pre-normalized copy of
   each CSV (each quaternion row divided by its own norm; a new file, the
   originals untouched).

   | demo | RMSE non-norm (mm) | RMSE pre-normalized (mm) | Δ | final pos err non-norm (mm) | final pos err pre-norm (mm) | Δ | final orient err non-norm (deg) | final orient err pre-norm (deg) | Δ |
   |---|---|---|---|---|---|---|---|---|---|
   | trajC | 0.296986 | 0.296986 | 0.000000 | 0.0798907 | 0.0798907 | 0.000000 | 0.0735663 | 0.0735663 | 0.000000 |
   | trajA | 0.234995 | 0.234995 | 0.000000 | 0.162079 | 0.162079 | 0.000000 | 0.015209 | 0.015209 | 0.000000 |
   | reach_task_baseline | 0.208075 | 0.208075 | 0.000000 | 0.402913 | 0.402913 | 0.000000 | 0.101490 | 0.101490 | 0.000000 |

   The trajC numbers reproduce the previously documented reference values
   (RMSE 0.2970 mm, final position error 0.0799 mm, final orientation
   error 0.0736°) to 4 decimal places. The pre-normalized fit is identical
   to the non-normalized fit at that precision on all three demos; the raw
   per-sample replay CSVs differ only at the ~1e-6–1e-7 text-representation
   level (last printed digit), consistent with the input deviation
   magnitude above.

**Conclusion**: for the demo CSVs actually in use, the missing
normalization in `learn_and_test_dmp.cpp`'s local loader has **no
measurable effect** on RMSE or final position/orientation error at the
precision already used in thesis material (4 decimal places, mm/deg).
`arccos(qw)` never saw `|qw| > 1` for these inputs; no NaNs were produced.

**Recommendation**: do **not** modify `learn_and_test_dmp.cpp`'s loader (or
`demo_csv_io`) at this time — that loader produced all currently-verified
reference numbers, and re-verifying them against a new loader is not
justified by a discrepancy that doesn't exist in practice. This is
documented here as a known asymmetry between the two loaders, worth
revisiting only if a future demo CSV is captured with visibly worse
quaternion precision (e.g. from a noisier source) where `|1 - ||q|||`
could approach a magnitude that risks `arccos(qw)` seeing `|qw| > 1`. If/when
that happens, the fix is a one-line `.normalized()` call in
`learn_and_test_dmp.cpp`'s local loader (mirroring `demo_csv_io::readDemoCsv`),
followed by re-verification of any reference numbers derived from it.

**Reproducibility**: diagnostic scripts (not part of the build, standalone,
read-only w.r.t. all existing files):
- `analyze_quat_deviation.py` — computes the per-row unit-norm deviation
  stats for one or more demo CSVs.
- `normalize_quat_csv.py` — writes a normalized copy of a demo CSV without
  touching the original.

Both were run from a scratch directory outside the repo for this
investigation; equivalent copies are not committed to the repo as part of
this diagnosis-only task.

**Review note (2026-09-29)**: (1) in the second table, the RMSE and final
position error columns cannot discriminate the effect of quaternion
normalization: `DMP::learnFromDemonstration`/`DMP::step`
(`src/core/dmp.cpp`) never reference `orientation` — the position `DMP` in
`learn_and_test_dmp.cpp` is trained and rolled out independently of the
`QuaternionDMP` (`qdmp`), which reads the (non-normalized) quaternion
instead. (2) The evidence for the conclusion is therefore the final
orientation error column only — one number per demo, not a trajectory
statistic. (3) Cross-check: `learn_and_test_prodmp.cpp`'s normalizing
loader (`demo_csv_io::readDemoCsv`) reports `endpoint_orient_error_deg =
0.10149` on `reach_task_baseline.csv`
(`runs/20260914_150515_..._w0.05/summary.csv`), matching the
`0.101490`° in this entry's table for the non-normalizing classic-DMP
loader. This supports the conclusion only if both pipelines fit
orientation with the same `QuaternionDMP` configuration. Parameters used
by each, for the runs of this entry's table (all three demos share this classic-DMP configuration), side by side:

| Parameter | `learn_and_test_dmp.cpp` (`qdmp`) | `learn_and_test_prodmp.cpp` (`qdmp`) |
|---|---|---|
| n_basis | 200 — CLI arg `argv[6]`, `QuaternionDMP qdmp(n_basis, ...)` | 200 — `kQuatDmpNBasis`, `QuaternionDMP qdmp(kQuatDmpNBasis, ...)` |
| alpha_x | 4.6 — CLI arg `argv[7]`, same constructor | 4.6 — `kQuatDmpAlphaX` |
| alpha_z | 25 — CLI arg `argv[8]`, same constructor | 25.0 — `kQuatDmpAlphaZ` |
| beta_z | 6.25 — CLI arg `argv[9]`, same constructor | 6.25 — `kQuatDmpBetaZ` |
| ridge_lambda | 1.0e-6 — `config/dmp_features.yaml` (`regression.ridge_lambda`, `regression.method: "ridge"`) applied via `dmp_io::applyFeatureConfig(path, dmp, qdmp)` -> `qdmp.setRidgeRegression(true, lambda)` | 1e-6 — `kQuatDmpRidgeLambda`, `qdmp.setRidgeRegression(true, kQuatDmpRidgeLambda)` |
| velocity filter window | 0.20 s / 0.20 s — `config/dmp_features.yaml` (`velocity_filter.window_sec_1`/`_2`) via `applyFeatureConfig` -> `qdmp.setVelocityFilter(true, w1, w2)` | 0.20 s / 0.20 s — `kQuatDmpFilterWindowSec`, `qdmp.setVelocityFilter(true, kQuatDmpFilterWindowSec, kQuatDmpFilterWindowSec)` |

Every value is equal: same configuration. (4) Recommendation unchanged.
