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
