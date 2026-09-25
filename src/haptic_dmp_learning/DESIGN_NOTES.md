# Design notes

## `/master_pose_raw`: the raw master device stream

`/master_pose_raw` (`geometry_msgs/PoseStamped`) is the raw, undemonstrated
pose stream from whatever is standing in for the operator's hand right now.

- **Today**: no physical Geomagic Touch is attached, so
  `csv_master_pose_player_node` replays a previously recorded demo CSV
  (`t,x,y,z,qw,qx,qy,qz`, non-uniform timestamps) onto this topic with a
  zero-order hold, at the pace it was originally recorded.
- **Tomorrow**: the Geomagic Touch driver already publishes
  `geometry_msgs/PoseStamped` natively (see `/touch0/pose` in
  `haptic_dmp_wrapper_node`). Switching sources is therefore a launch-time
  topic remap on the driver node (`-r <driver_native_topic>:=/master_pose_raw`),
  never a conversion node - `csv_master_pose_player_node` is simply not
  launched (`use_csv_playback:=false` in `launch/live_demo.launch.py`).

`live_demo_recorder_node` subscribes to `/master_pose_raw` and does not know
or care which of the two is feeding it.

## `/target_pose`: still reserved for DMP rollout

`/target_pose` is read by `CartesianVelocityController`
(`velocity_cartesian_control`) as the Cartesian setpoint to track. It has one
long-standing producer, `dmp_gazebo_executor_node`, which replays an
*already-learned* DMP onto it.

`live_demo_recorder_node` also publishes onto `/target_pose`, but only to make
the raw, not-yet-learned demonstration visible live in Gazebo while it is
being recorded (i.e. only between the start and stop button events) - not as
soon as poses arrive on `/master_pose_raw`. This is deliberate:
`CartesianVelocityController` captures its device/robot alignment offset
once, from the very first `/target_pose` message it receives after
activation (`on_activate`/`update()` in `cartesian_velocity_controller.cpp`).
Publishing before the operator presses start would anchor that alignment to
an arbitrary device pose instead of the one the demo is meant to start from.
The two producers are not meant to run at the same time: the
live-demo pipeline runs before learning, `dmp_gazebo_executor_node` runs after.

## Start/stop detection: reused, not reinvented

Demo start/stop was already solved by `haptic_dmp_wrapper_node`: it
subscribes `sensor_msgs/Joy` on `/touch0/buttons` and detects rising edges on
`buttons[0]` (start recording) and `buttons[1]` (stop + learn). This is the
Geomagic Touch driver's native button topic and message type.

`live_demo_recorder_node` reuses this mechanism verbatim (same topic, same
message type, same rising-edge logic) instead of inventing a new one. With
the real driver this works unmodified. With the CSV stand-in there are no
physical buttons, so `csv_master_pose_player_node` emits the equivalent
synthetic `Joy` sequence on the same topic: an idle baseline `[0,0]` right
after startup (so the downstream node's "first message just seeds prior
state" rule doesn't swallow the real edge), then `[1,0]` (start rising edge)
once playback begins, then `[0,1]` (stop rising edge) at end of file. End of
file is thus made to look, from the downstream node's point of view,
identical to a real stop-button press.

## Where the ridge+filter training lives

The fit is in-process C++, not an external script:

- `core::DMP::learnFromDemonstration()` /
  `core::QuaternionDMP::learnFromDemonstration()`
  (`src/core/dmp.cpp`, `src/core/quaternion_dmp.cpp`) do the actual locally
  weighted / ridge regression.
- `core::dmp_io::applyFeatureConfig()` (`src/core/dmp_io.cpp`) loads the
  regression method and velocity-filter flags from a YAML file - by default
  `config/dmp_features.yaml`, which already sets `method: ridge`,
  `ridge_lambda: 1e-6` and `velocity_filter.enabled: true` (the same values
  used to produce `real_trajA_ridge_filter.yaml` and friends under
  `tools/dmp_offline_test/weights/`).
- `core::dmp_io::saveToYaml()` writes the learned weights out.

Both `haptic_dmp_wrapper_node::stopRecordingAndLearn()` and
`live_demo_recorder_node::stopRecordingAndLearn()` call this same sequence
directly, in-process, on the samples accumulated in a
`core::DemonstrationRecorder`. The offline CLI under `tools/dmp_offline_test`
(`common/src/learn_and_test_dmp.cpp`) links the exact same `core/*.cpp` files
but is only used for batch sweeps/plots - it is not part of the live
pipeline and nothing here shells out to it.

## Misconfiguration must fail loudly, never silently downgrade

A prior refactor (2026-08-24) replaced the fallback/error handling below with
silent early-returns. The result: a broken/unreachable `dmp_features.yaml`
made `applyFeatureConfig()` return without applying anything, and a node
started via bare `ros2 run` (no `--params-file`) silently kept `n_basis=20`
instead of the validated `200`. Neither failure printed anything. On real
teleoperated demo data this combination degrades the learned trajectory from
~0.23mm RMSE to ~72mm RMSE / 60deg mean orientation error - the DMP looks
"completely wrong" with no error message pointing at why. Root-caused by
isolating with synthetic data (core algorithm unaffected, ~1mm RMSE) then
real data with the config deliberately broken (reproduced the 72mm/60deg
failure exactly). The two conventions below exist specifically to prevent
this class of regression from recurring silently:

- **`core::dmp_io::applyFeatureConfig()` never fails silently.** It tries the
  given path, then two hardcoded fallback locations
  (`$HOME/thesis_ws/src/haptic_dmp_learning/config/dmp_features.yaml`,
  `$HOME/thesis_ws/dmp_features.yaml`), and if all three fail it **throws**
  `std::runtime_error` listing every path it tried. Proceeding with the
  default independent-LWR/no-filter behavior after a config load failure is
  not an acceptable fallback: it looks like a working DMP with much worse
  accuracy, not an obvious failure. Both `haptic_dmp_wrapper_node` and
  `live_demo_recorder_node` let this exception propagate out of the
  constructor; `main()` in each catches it, logs via `RCLCPP_FATAL`, and
  exits with status 1 instead of calling `rclcpp::spin()`.
- **`haptic_dmp_wrapper_node` and `live_demo_recorder_node` both require
  `--ros-args --params-file <config/params.yaml>` at launch.** `n_basis`,
  `alpha_x`, `alpha_z`, and `beta_z` are declared as *mandatory* ROS2
  parameters (no default value), so `declare_parameter<T>(name)` itself throws
  if they were not supplied externally - there is deliberately no code path
  left that silently reads `params.yaml` by hand or falls back to the
  ROS-default `n_basis=20`. `haptic_dmp_wrapper_node` has no launch file (it
  targets the real Geomagic Touch device inside the `geomagic_touch`
  container, run directly via `ros2 run`), so this is its only guard against
  forgetting `--params-file`. `live_demo_recorder_node` is always launched via
  `launch/live_demo.launch.py` / `launch/demo_replay_sync.launch.py`, which do
  pass `params_file` explicitly - but the mandatory declaration makes a bare
  `ros2 run haptic_dmp_learning live_demo_recorder_node` fail loudly instead of
  silently training a bad DMP.
- **Output/weights/feature-config paths default to absolute
  (`$HOME/thesis_ws/...`) paths**, not relative ones, in `config/params.yaml`
  and in the three nodes' `declare_parameter` defaults
  (`output_yaml_path`, `output_demo_csv_path`, `weights_yaml_path`,
  `feature_flags_path`). Relative defaults silently depend on the process's
  current working directory at launch, which is not guaranteed to be
  `/root/thesis_ws` for every way these nodes get started - this is what let
  `dmp_gazebo_executor_node` load a stale `dmp_weights.yaml` without
  complaint in one investigation.
- Whenever an editing pass touches `dmp_io.cpp`, the ROS node constructors,
  or `config/params.yaml`, the change needs an actual `colcon build
  --packages-select haptic_dmp_learning --symlink-install` (not just
  `--symlink-install` for YAML edits) before it is considered live: check
  that the resulting binaries under `build/haptic_dmp_learning/` are newer
  than the edited sources (`stat` both, or `find <sources> -newer <binary>`
  should be empty). A rebuilt-but-unverified binary was mistaken for a stale
  one during this investigation - always check the actual mtimes rather than
  assuming.

## Synchronised demo/replay against a moving target, and per-demo force signature

This section covers `demo_replay_sync_orchestrator`, `grasp_force_calibration_node`
and `grasp_state_machine`, and the sim-time conversion of the replay/stand-in
nodes.

### Why the force calibration is per-demo (`run_id`), not global

`~/contact_wrench_estimate` (published by `CartesianImpedanceController`, see
`franka_cartesian_control/DESIGN_NOTES.md`) is **not** a measurement: it is
`F_ext ≈ -(K·δx + D·δẋ)` from the impedance law, and it carries a
**configuration-dependent bias** plus a tracking-lag term that depends on the
reference source. That same DESIGN_NOTES already states the free-space baseline
differs between live haptic streaming and DMP replay and that "any
contact-detection threshold built on top of this signal downstream must be
calibrated separately". `grasp_force_calibration_node` implements exactly that:
on the first rising edge of the geometric grasp signal it accumulates every
force-norm sample that arrives within a **`capture_window_sec` time window**
(default 0.25 s, measured on the node clock — sim time under `use_sim_time:=true`)
and takes their **median**, then stores it in `force_calibration_<run_id>.yaml`.
`verify` mode reloads the file for the *same* `run_id` and gates on
`|median - f_calib| <= max(tolerance_ratio·f_calib, tolerance_floor_n)`. A single
global threshold would either miss contacts (DMP case) or false-trigger on
tracking noise (haptic case).

The window is a **duration**, not a fixed message count: at the 1 kHz control-loop
rate a fixed count of ~8 messages spans only ~8 ms, far below the tens-to-hundreds
of ms timescale of human teleoperation jitter, so those samples are strongly
correlated and the median buys nothing over a single sample. A one-shot timer on
the node clock closes the window even if the force topic goes completely silent;
**zero messages in the window is an explicit error** (`RCLCPP_ERROR`, and `verify`
publishes `False`) — the node never medians an empty buffer or falls back to a
default.

`finishCaptureWindow` then applies a **second, separate reject** after the median
is computed: if the median force norm is below `min_valid_force_n` (default 3 N)
the capture is treated as configuration/tracking bias rather than real contact —
`calibrate` logs `RCLCPP_ERROR` and writes no file, `verify` logs `RCLCPP_WARN`
and publishes `False`. The 3 N default is the same number and rationale as
`tolerance_floor_n` in `verify` mode: a margin above the ~2 N configuration-
dependent tracking bias documented in `franka_cartesian_control/DESIGN_NOTES.md`.
It is a distinct parameter from `tolerance_floor_n` so the calibrate-side gate and
the verify-side tolerance floor can be tuned independently later.

### The implicit assumption of the demo↔replay force comparison

Comparing a demo-time force signature against a replay-time one is only valid
while the **joint-space path** taken during replay stays close to the one taken
during the demonstration: the sensorless estimate is configuration-dependent, so
a different arm posture at the grasp instant changes the bias even for the same
external wrench. This holds for straight DMP replay of the same trajectory; it
**must be re-verified** when Floating Reference Point / Residual SAC are
introduced, since those change the replayed trajectory (and hence the posture)
relative to the recorded demo. Re-record + re-calibrate in that case. The fixed
`note` field written into every calibration file states this.

### `limit_action` is latched until an explicit reset

`grasp_state_machine` enters `limit_action` when `|F|` exceeds the absolute
`hard_force_limit_n` while in `contact_confirmed`. It does **not** leave that state
automatically — not when the geometric signal is lost, not when `|F|` drops back
below the limit. The only exit is `std_msgs/Bool` `true` on
`~/reset_limit_action`.

Rationale: an automatic exit on signal loss conflates two outcomes that a
`~/grasp_state` consumer must be able to tell apart — (a) the target was
**repelled** by excessive contact (exactly the failure this whole component
exists to detect), and (b) the safety action was **handled** correctly.
Auto-clearing would make both look identical (a transition back to `free_space`).
Latching until an operator/supervisor explicitly resets keeps the machine
visibly parked in `limit_action` (state is still published every cycle, nothing
is blocked) so downstream logic sees the stop and its cause.

This is an assumption taken to proceed; revisit it if practical use shows an
automatic recovery path is actually wanted (e.g. reset on a fresh
`contact_confirmed` from a clean re-approach).

### Why the 5 s are anchored to the target's first odometry stamp

`sync_delay_sec` (default 5.0) is measured from the **sim-time timestamp of the
first `/…/odometry` message** of the freshly spawned `free_target_object`
(sim-time is guaranteed on that topic by `free_target_object.launch.py`, which
sets `use_sim_time:=true` on its odometry bridge), not from the wall-clock
instant the orchestrator launched the spawn subprocess. Launching
`ros2 launch free_target_object …` has a variable wall-clock duration (xacro
expansion, world-name service call, entity creation) which is irrelevant as long
as the anchor is a sim-time-relative quantity. Both `demo` and `replay` modes use
the identical anchor, so the target is in the same sim-time state when the demo
start edge is emitted and when the DMP rollout begins.

### Why `startup_delay_sec` is zeroed for orchestrated replay

`dmp_gazebo_executor_node` has its own internal one-shot `startup_delay_sec`
(default 1.0 s) to let Gazebo/controller settle before the rollout. In the
orchestrated replay path that settle time is already covered by the 5 s sync
window, so the orchestrator launches the executor with `startup_delay_sec:=0.0`.
Leaving it at 1.0 would add a delay on the replay side that has no counterpart on
the demo side, producing an asymmetric offset between the two runs.

### sim-time conversion and stale `tau`

`dmp_gazebo_executor_node` and `csv_master_pose_player_node` had their
`create_wall_timer` calls replaced with `rclcpp::create_timer(this,
get_clock(), …)` so the startup delay, the rollout step `dt` and the CSV pacing
advance on `/clock` when the node is launched with `use_sim_time:=true`
(`create_wall_timer` is contractually a steady-clock timer regardless of
`use_sim_time`). `haptic_dmp_wrapper_node` and `live_demo_recorder_node` have no
timers - only `this->now()`, which already honours `use_sim_time` - so they need
the launch parameter, not a code change.

`tau` (DMP rollout duration) is computed as `demo.back().t - demo.front().t`
(`core/dmp.cpp:79-84`, `core/quaternion_dmp.cpp:86-88`) from sample timestamps
that, before this conversion, were wall-clock quantities. **Any `dmp_weights.yaml`
recorded before the sim-time conversion carries a `tau` that is no longer
consistent with sim-time playback and must be re-recorded** (the executor reads
`tau` back verbatim from the weights YAML and never recomputes it).

### `/clock` availability is checked, not assumed

`/clock` is **not** bridged by any file in this workspace - it depends on an
external component (e.g. `gz_ros2_control` in the Franka Gazebo bringup). Before
doing anything else, `demo_replay_sync_orchestrator` subscribes to `/clock` and
waits `clock_wait_timeout_sec` (default 10 s, wall clock) for the first message;
on timeout it logs `FATAL` and exits non-zero rather than proceeding with a
clock that never advances.

## `satellite_rotation_mode="continuous"` (v0): predictive open-loop intercept

Pure logic lives in `include/haptic_dmp_learning/core/satellite_intercept.hpp` (Eigen + STL, no ROS;
tests in `test/test_satellite_intercept.cpp`). The node wiring is in `prodmp_gazebo_executor_node.cpp`.
Nothing of it is active with `satellite_rotation_enabled=false` or `mode="frozen"`; the frozen branch only
routes its (unchanged) anchoring + rotation arithmetic through `anchorAndRotate()`.

- **Contact phase is a parameter** (`satellite_intercept_phase_deg`); there is no PhaseSelector here.
- **Trigger**: `theta_trig = theta_int - omega * T_total (mod 360)`; the rollout starts at the first phase sample
  crossing it (no angular tolerance). If the first consumed sample is already past, the next lap is awaited.
- **T_total** = node-clock time from trigger detection to `at_end` = `tau` (measured `elapsed >= tau`) + one
  timer period `dt_` (first tick of `step_timer_`) + 0 (single non-blocking TF lookup of `ee_start`). The
  `startup_delay_sec` elapses BEFORE the trigger machinery is armed and is not part of it.
  `satellite_contact_time_s` is checked against `tau + dt_` (`contact_time_tolerance_s`,
  `allow_contact_time_mismatch`).
- **Time base**: `elapsed = now - t_roll0` on the node clock (sim time; `use_sim_time=true` is enforced, and
  `ros_time_is_active()` is checked when the plan starts). ProDMP / QuaternionDMP are advanced by the MEASURED
  step between ticks; frozen keeps `elapsed_ += dt_`.
- **Phase**: `measured` (odometry `header.stamp`, rotation checked to be about the configured axis within 0.5 deg,
  unwrapped; omega verified against the phase history before any trigger is accepted) or `model`
  (`theta0 + omega * t_sim`).
- **Report**: `~/continuous_status` (`diagnostic_msgs/DiagnosticArray`, key/value strings, `header.stamp` = node
  clock) from the start of the wait to >= 20 s after `at_end`.
- The detectable omega error of the consistency check is `tol / (omega * window)`: with 1 deg, 2 deg/s and 5 s
  it is 10 %. Tighten `phase_consistency_tol_deg` / widen `phase_consistency_window_s` for a stricter check.

### 2026-09-22 fix: `anchorAndRotate()` goal was off by the demo's own init position (~192 mm)

`demo_grasp_goal_` was captured as `prodmp_.goal()` - the raw absolute grasp coordinate stored in
`weights.yaml` (`relative_goal: false`), i.e. the point in the demo's own local frame, **not** a
displacement from where the demo started. `anchorAndRotate()` (frozen and continuous both call it,
`satellite_intercept.hpp`) then computed `p_demo_world = ee_anchor + demo_grasp_goal_`, treating that
absolute coordinate as if it were an offset from `ee_anchor` - silently double-counting
`demo_init_pos_` and shifting every world-frame goal by ~192 mm (measured on the affected weights file).
The pure baseline branch (`target_odom_required_=false`, no rotation) never had this bug: it hands
`demo_init_pos_` to ProDMP as `init_pos_` with `relativeGoal=true` and lets `ProDMP::setGoal()` compute
`new_goal - init_pos_` internally - the one place this subtraction should happen.

Fix: `ProDMP::demoDisplacement()` (`core/prodmp.hpp`) now exposes `goal() - initPos()`, the physical
displacement the demo covered, independent of `relativeGoal()`/runtime re-anchoring. `demo_grasp_goal_`
is captured via this accessor instead of `prodmp_.goal()` (`prodmp_gazebo_executor_node.cpp`, at load
time, before anything else can call `setGoal()`/`setInitialConditions()`); `anchorAndRotate()` itself is
unchanged, since it was already computing `ee_anchor + goal` correctly - only what was passed as `goal`
was wrong.

**All frozen-mode results computed before this fix (phase 30/90/180/270 runs) and any
`satellite_grasp_planner` output derived from them are on goals translated by ~192 mm from the
correct point and are not reusable for quantitative conclusions without re-running.**
`satellite_grasp_planner`'s `PhaseSelector`/`JointPathSimulator` do not read `weights.yaml`
themselves - they take `grasp_pos_body` as a caller-supplied parameter, so once the caller passes the
corrected displacement the fix propagates without any change needed in that package.

**2026-09-23 addendum**: the same invalidation applies to `tools/compare_proxy_vs_real_phases.py`
(w_trans-along-phase comparison, PhaseSelector/JointPathSimulator vs. real Gazebo manipulability -
§8.4/9.4). `PhaseSelector`/`JointPathSimulator` themselves are pure and unaffected (see above), but
`compare_proxy_vs_real_phases.py` consumes
`target_aligned_reach_task_satellite_rot_phase{30,90,180,270}.csv` and
`joint_states_reach_task_satellite_rot_phase{30,90,180,270}.csv` - Gazebo runs recorded BEFORE the
fix, i.e. the ~192 mm goal error is baked into the joint trajectories themselves, not just into a
downstream computation. **The pre-fix CSVs (no `_displacement_fixed` suffix) are invalid for any
w_trans-vs-phase quantitative comparison.** The post-fix CSVs (`_displacement_fixed` suffix, see
session summary 2026-09-22 §A.8) are valid but only cover 5 phases (0°, 30°, 90°, 180°, 270°) with a
single run each, no repeats - insufficient for the planned repeated-run sweep.

### Checklist item 4 - "Sweep di fasi con ripetizioni: w massimo => errore finale minimo?" (2026-09-23, post-fix, 25/25 run)

The repeated-run sweep called for above is now complete: 5 fasi (0°, 30°, 90°, 180°, 270°) x 5
ripetizioni = 25/25 run, `Kt=200`, goal geometricamente corretti post-fix displacement (suffisso
`_sweep_5rep_23_09`). The result is **duplice** and the two halves must be kept distinct - a previous
report (Gemini) stated this point was "confermato al 100%", but that conflates (a) and (b) below: only
(a) is confirmed, (b) is an open problem.

**(a) w_real -> errore finale: CONFERMATO**, and on clean (post-fix) data the correlation is stronger
than the pre-fix estimate. `w_real` (Gazebo, media 5 ripetizioni, deviazione standard trascurabile,
<0.01%) correla con l'errore di tracking Cartesiano con Pearson r=-0.954 (R²≈0.91), contro il valore
storico del 21/09 di r=-0.867 (R²=0.752, calcolato su dati poi invalidati dal bug di displacement -
vedi addendum sopra). Questo è citabile in tesi come risultato solido.

180° e 270° hanno `w_real` quasi identico (0.1276 vs 0.1274, 0.16% di scarto) ma errore diverso e
ripetibile (2.68 mm vs 2.02 mm, ~33%): questo residuo non spiegato è coerente con il 25% già
documentato il 21/09, non è un'anomalia nuova introdotta dal fix.

**(b) w_proxy -> w_real (uso predittivo, cioè per `PhaseSelector` PRIMA dell'esecuzione): RICONCILIATO,
proxy affidabile.** Il verdetto precedente (ρ=0.40, proxy non affidabile) era basato su un secondo
script Python diagnostico usato per il primo sweep offline - **non** il codice di produzione - ed è
superato dalla riconciliazione seguente contro il proxy C++ di produzione (`PhaseSelector`/
`JointPathSimulator`, che usa il vero `prodmp.step()`), verificato end-to-end nel container.

Tabella finale riconciliata (unico riferimento per questo punto, sostituisce ogni numero precedente):

| Fase | w_real (Gazebo Kt=200, media 5 rep) | w_proxy C++ (scale=10) | Gap  |
|------|--------------------------------------|--------------------------|------|
| 0°   | 0.0924                               | 0.0961                   | +4.0% |
| 30°  | 0.0858                               | 0.0894                   | +4.2% |
| 90°  | 0.0950                               | 0.0983                   | +3.5% |
| 180° | 0.1276                               | 0.1326                   | +3.9% |
| 270° | 0.1274                               | 0.1326                   | +4.1% |

Ranking preservato: proxy e reale concordano su {180°, 270°} come coppia migliore e 30° come peggiore.
Il gap è sistematico e uniforme (+3.5% a +4.2%), non più il -28%/-81% del calcolo iniziale bacato - il
proxy C++ di produzione è affidabile per la selezione di fase.

Causa del falso allarme iniziale (ρ=0.40): il secondo script Python diagnostico (non il codice di
produzione) usato per il primo sweep offline conteneva una formula di generazione traiettoria
algebrica fittizia, incompatibile con la dinamica di ProDMP (pesi calibrati per un sistema del
second'ordine, applicati linearmente) - produceva escursioni fuori scala (fino a -67 m in Z) che
saturavano il DLS su configurazioni quasi-singolari fittizie. Bug isolato allo script diagnostico, MAI
presente nel codice C++ di produzione (`PhaseSelector`/`JointPathSimulator`), verificato
indipendentemente con esecuzione end-to-end nel container.

**Nota su `joint1_nullspace_gain_scale`: CHIUSO** (indagine dedicata). Testate 4 configurazioni di
`k_ns`: Config A (`scale=10`, attuale), Config B (`scale=5`, teoricamente esatto per K0/D0=1.118 del
controllore reale), Config C ("tasso vero" √Ki uniforme su tutti i giunti), Config D (tasso vero +
smorzamento doppio reale su giunto 1). Config A e Config C producono `w_proxy` identici fino alla 5a
cifra decimale su tutte le 5 fasi: **la scelta di `k_ns` non ha alcun effetto misurabile**. Causa
analitica: la costante di tempo del nullspazio (1/k_ns≈2.2-4.5s) è 13-28x più corta della durata del
rollout (τ=60.97s) - il sistema ha ampio margine per convergere alla stessa varietà stazionaria
indipendentemente da quanto velocemente `k_ns` lo porta lì (`k_ns` determina la velocità di
convergenza, non la destinazione, e con τ≫1/k_ns la destinazione è tutto ciò che conta a fine
rollout). Il ranking tra le 5 fasi resta identico in tutte e 4 le configurazioni testate - `k_ns` non
ha mai avuto impatto pratico sulle decisioni del `PhaseSelector`, indipendentemente dalla sua
calibrazione.

Causa reale del residuo ~4% (non `k_ns`): l'errore di inseguimento stazionario intrinseco al
controllore a impedenza cartesiana a rigidezza finita (`Kt=200` N/m) - il controllore non annulla mai
l'errore posizione-target per costruzione fisica (F=Kt·errore≠0 a regime), mentre il proxy cinematico
(DLS ideale) insegue il target esattamente. Coerente in ordine di grandezza: errore di tracking finale
misurato in Gazebo ≈1.98-2.30 mm su tutte le fasi, residuo relativo su `w_trans` 1.4%-6.3% - stesso
ordine di grandezza, compatibile con la spiegazione, **ma non dimostrato formalmente** (resta
un'ipotesi qualitativa ben supportata, non una prova matematica). Tre possibili sorgenti fisiche del
residuo, non isolate né verificate singolarmente (ipotesi aperte, non fatti): (a) proiettore di
nullspazio approssimato (N=I-J^T(JJ^T+λ²I)^-1J, λ=0.05) che lascia "fuggire" una frazione della coppia
di richiamo nullspazio nello spazio del task; (b) compensazione di gravità imperfetta (massa/baricentro
modellati vs reali, incluso carico gripper); (c) attrito statico nei giunti vicino all'equilibrio.
Nessuna delle tre verificata.

Decisione presa: NON perseguire ulteriore correzione - il gap ~4% non altera il ranking (verificato su
4 configurazioni), quindi non cambia alcuna decisione pratica del `PhaseSelector`. Costo/beneficio
sfavorevole rispetto al tempo disponibile per il progetto. Il codice resta con `k_ns scale=10`
(invariato).

La spiegazione "270° beneficia di compliance dinamica e rilassamento del nullspace" (da un report
precedente) **non è verificata da alcuna analisi dedicata** in questo sweep e non va citata in tesi
come fatto accertato finché non isolata con un test specifico.

**Punto 4 della checklist: CHIUSO.** w massimo (proxy) predice correttamente errore minimo (reale)
entro un margine di errore uniforme del ~4%, su tutte e 5 le fasi testate.
