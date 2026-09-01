# Design notes — vendored Franka overrides

These packages hold copies of files that belong to the vendored `franka_ros2`
tree in `$FRANKA_WS`, edited for this project and pushed back in by the
`scripts/setup_*.sh` scripts. A future upstream update of `franka_ros2` would
silently discard every one of these edits until the corresponding setup script
is re-run — an accepted risk.

| Override file | Overwrites (in `$FRANKA_WS`) | Propagated by |
|---|---|---|
| `franka_gazebo_overrides/gazebo_*_control*.launch.py`, `franka_gazebo_controllers.yaml` | `franka_gazebo/franka_gazebo_bringup/{launch,config}/` | `full_reset_franka_container.sh` (add `--no-haptic` for the reduced perimeter) |
| `franka_description_overrides/franka_hand.xacro` | `franka_description/end_effectors/common/franka_hand.xacro` | `setup_franka_description_overrides.sh` |

## Gripper: actuated fingers via **software mimic** in `gz_ros2_control`

The earlier "move the finger `<origin>` to fake an open static pose" trick is
**superseded** — the fingers are now real prismatic joints.

- `${arm_id}_finger_joint1` (**leader**) and `${arm_id}_finger_joint2`
  (**follower**) become `type="prismatic"` in
  `franka_description_overrides/franka_hand.xacro`, with the upstream
  `axis` / `limit` (`lower=0.0 upper=0.04 effort=100 velocity=0.2`) / `dynamics`
  (`damping=0.3`) re-enabled and the `<origin>` restored to `0 0 0.0584`.
- The follower carries `<mimic joint="${arm_id}_finger_joint1"/>` — **no
  `multiplier`**. The two joint axes are already opposite (`+Y` leader, `-Y`
  follower), so the same commanded value on both produces symmetric closing;
  the default multiplier of `1` is correct. (A `multiplier="-1"` would be needed
  only if both axes pointed the same way.)
- **The mimic is resolved in software by `gz_ros2_control`**, not by gz-physics:
  Ignition **Fortress** has no native mimic-joint support, but the
  `gz_ros2_control` plugin (same one the Franka bringup uses) applies the mimic
  constraint itself. Verified experimentally with the official
  `gz_ros2_control_demos` `gripper_mimic_joint_example_position` in this exact
  environment.
- **Confirmed `<ros2_control>` structure** (the official docs are slightly
  misaligned — they show `mimic` / `multiplier` params *inside* the
  `<ros2_control>` joint entry; in the working setup those are **absent**):
  - leader: `command_interface name="position"` + `state_interface`
    position/velocity/effort, with `initial_value` = `0.04` on the position
    `state_interface` (start open);
  - follower: **only** `state_interface` position/velocity/effort — **no
    `command_interface`, no mimic/multiplier param**. The `<mimic>` tag in the
    URDF (above) is the whole mechanism.
  This block lives in `franka_description`'s `*.ros2_control.xacro` (arm model,
  in `$FRANKA_WS`), **not** in `franka_hand.xacro` and not vendored in this
  workspace — it must be edited container-side (or that file vendored too).
- Controller: `gripper_controller`
  (`forward_command_controller/ForwardCommandController`, `interface_name:
  position`) targets **only the leader** `fer_finger_joint1`
  (`franka_gazebo_overrides/franka_gazebo_controllers.yaml`). It is spawned from
  `gazebo_cartesian_impedance_control.launch.py` after `joint_state_broadcaster`,
  gated on `load_gripper:=true`.
- Still **no effect unless the bringup is launched with `load_gripper:=true`**
  (default `false` loads no hand at all).

## `/clock` bridge

See `franka_cartesian_control/DESIGN_NOTES.md` — `clock_bridge` was added to
`gazebo_cartesian_impedance_control.launch.py` because the stock bringup does
not bridge `/clock`.
