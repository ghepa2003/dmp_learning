# Design Notes

## Sensorless contact wrench estimate (`~/contact_wrench_estimate`)

- This is not a real force measurement. It is an estimate derived from the
  Cartesian impedance control law itself: `F_ext ≈ -(K · δx + D · δẋ)`, where
  δx = target − current (this codebase's error convention, see
  `CartesianError`), which is only valid in the quasi-static regime.
  `K · δx + D · δẋ` is the wrench the controller commands toward the target
  (the same term used for `tau_task`); negating it reports the external
  reaction wrench instead. When the commanded reference velocity is
  low/near-zero, this negated term approximates the external wrench resisting
  the spring-damper restoring force. When the reference is moving quickly, the
  same term also contains the tracking-lag contribution (the wrench the
  controller applies purely to catch up to a moving target), so at high
  reference speeds this signal should be interpreted with caution — it is not
  a clean contact signal in that regime.

- **This component is specific to the Gazebo simulation path.** On real
  Franka hardware, `~/contact_wrench_estimate` must be fed from the native
  `O_F_ext_hat_K` estimate exposed by `libfranka` (based on a momentum
  observer over the joint torque sensors), not from this computation. Do not
  port this implementation to the real-hardware path — the topic is kept
  source-agnostic (`geometry_msgs/msg/WrenchStamped` on a generic name,
  independent of `enable_nullspace_leak_diagnostics`'s sim-only diagnostic
  style) specifically so the real-hardware publisher can be swapped in
  without changing downstream consumers.

- The free-space baseline noise on this signal differs depending on whether
  the tracked Cartesian reference comes from live haptic device streaming
  (teleoperation — noise dominated by human tracking jitter) or from DMP
  replay (a smooth pre-recorded trajectory). Any contact-detection threshold
  built on top of this signal downstream must be calibrated separately for
  the two reference sources; a single shared threshold will either miss
  contacts in the DMP case or false-trigger on tracking noise in the haptic
  case.

- **The vendored Gazebo impedance bringup now bridges `/clock` explicitly.**
  The stock Franka Gazebo bringup did **not** bridge `/clock` from Gazebo to
  ROS 2 in this configuration — `gz_sim.launch.py` / `gz_ros2_control` do not
  expose it — so every ROS-side node started with `use_sim_time:=true`
  (`controller_manager`, the controllers, and the downstream
  `haptic_dmp_learning` nodes) ran against a clock that never advanced. This is
  invisible to static inspection of the workspace and was found only at runtime
  (`ros2 topic info /clock --verbose` → `Publisher count: 0` while three nodes
  were already subscribed). A dedicated `clock_bridge` node
  (`ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock`)
  was added to `franka_gazebo_overrides/gazebo_cartesian_impedance_control.launch.py`
  (inherited by the `_headless` wrapper; propagated into `franka_ws` verbatim by
  `scripts/setup_cartesian_control.sh` / `setup_franka_cartesian_control.sh`). It
  is a hard prerequisite for anything depending on sim time:
  `demo_replay_sync_orchestrator_node` (which aborts if `/clock` is silent) and
  the sim-time conversion of `dmp_gazebo_executor_node` /
  `csv_master_pose_player_node`.
