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
- **`haptic_dmp_wrapper_node` requires `--ros-args --params-file
  <config/params.yaml>` at launch.** `n_basis`, `alpha_x`, `alpha_z`, and
  `beta_z` are declared as *mandatory* ROS2 parameters (no default value), so
  `declare_parameter<T>(name)` itself throws if they were not supplied
  externally - there is deliberately no code path left that silently reads
  `params.yaml` by hand or falls back to the ROS-default `n_basis=20`. This
  node has no launch file (it targets the real Geomagic Touch device inside
  the `geomagic_touch` container, run directly via `ros2 run`), so this is
  the only guard against forgetting `--params-file`. `live_demo_recorder_node`
  is not required to do this the same way because `launch/live_demo.launch.py`
  always passes `params_file` explicitly.
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
