# free_target_object

A **single free-floating rigid body** for Gazebo (Ignition **Fortress** / `gz-sim6`),
meant to replace the use of `testing_plane` as a "satellite" target.

Unlike `testing_plane` (a 6-DOF chain of position-servoed joints welded to the
world, i.e. an immovable kinematic obstacle), this model has **no joint to the
world and no controller**. It is a passive link that obeys only its own
rigid-body physics:

```
F = m a            I omega_dot = tau
```

`testing_plane` is left untouched in the repo for reference.

## Files

| Path | Purpose |
|------|---------|
| `models/free_target_object.sdf.xacro` | one `<link>`, `<gravity>0</gravity>`, mass/size/inertia as xacro args, **no `<ros2_control>`**; one observation-only `OdometryPublisher` plugin |
| `launch/free_target_object.launch.py` | expand xacro → spawn with `ros_gz_sim create` at a pose, start the odometry bridge, optionally seed an initial twist |

## Pose + twist on ROS 2

The model carries an **`OdometryPublisher`** plugin (ships with Fortress,
observation-only — it applies no force). The launch **also** starts a
`ros_gz_bridge parameter_bridge` (no separate command) that bridges

```
/model/<name>/odometry     ignition.msgs.Odometry
        →
/<name>/odometry           nav_msgs/msg/Odometry      # e.g. /free_target_object/odometry
```

`nav_msgs/msg/Odometry` gives, per message:

| Field | Content |
|-------|---------|
| `header.stamp` | real **sim time** of the sample (not zero — this is the whole point) |
| `header.frame_id` | `world` |
| `child_frame_id` | `<name>` (the model name) |
| `pose.pose` | the body's **world pose** (position + orientation) |
| `twist.twist.linear` | body **linear velocity** [m/s] |
| `twist.twist.angular` | body **angular velocity** [rad/s] |

The twist is useful beyond pose tracking — e.g. checking the target's
post-contact velocity in later analysis.

Published at **200 Hz** (`<odom_publish_frequency>`), matching this project's
other pose streams (`/target_pose` publishers run at 200 Hz).

### Plugin config (in the xacro)

```xml
<plugin filename="ignition-gazebo-odometry-publisher-system"
        name="ignition::gazebo::systems::OdometryPublisher">
  <dimensions>3</dimensions>          <!-- mandatory: default 2 = planar, wrong here -->
  <odom_frame>world</odom_frame>      <!-- matches the franka rviz fixed frame -->
  <robot_base_frame>free_target_object</robot_base_frame>  <!-- = xacro arg `name` -->
  <odom_publish_frequency>200</odom_publish_frequency>
</plugin>
```

`<odom_topic>` is left at its default, `/model/<name>/odometry`.

### Why not `dynamic_pose/info` → TF, or `PosePublisher`

- **`dynamic_pose/info` → `tf2_msgs/TFMessage`** (previous approach): every
  bridged transform came out with `stamp = 0` and `frame_id = ""`.
  SceneBroadcaster only fills the `Pose_V` **container** header, and
  `ros_gz_bridge`'s `Pose_V → TFMessage` conversion reads only the **per-entity**
  `Pose` headers, which are empty. Structural gap, not fixable downstream.
- **`PosePublisher`**: in gz-sim6 6.18.0 `<publish_model_pose>` is a no-op
  (topic advertised, nothing published) and `<publish_link_pose>` is only
  model-relative (constant for a single link at the origin).

`OdometryPublisher` builds its own message, so `header.stamp` / `frame_id` /
`child_frame_id` are all populated correctly.

### World name

`world` is optional and used **only** for `ros_gz_sim create -world`. If left
empty the launch resolves it from the running server via the `/gazebo/worlds`
service (`ignition.msgs.Empty` → `StringMsg_V`), the same way
`ros_gz_sim create` does. The odometry bridge does not need the world name.

No dependency on `franka_cartesian_control` or `haptic_dmp_learning`.

## Prerequisite

Start your Gazebo world **first** (this launch only spawns into an
already-running simulation, same pattern as `testing_plane`).

## Usage

Body at rest — the configuration for VIM validation:

```bash
ros2 launch free_target_object free_target_object.launch.py \
  x:=0.6 y:=0.0 z:=0.5
```

With a custom mass / size (inertia auto-derived as a solid homogeneous box):

```bash
ros2 launch free_target_object free_target_object.launch.py \
  mass:=5.0 size_x:=0.2 size_y:=0.2 size_z:=0.2 x:=0.6 z:=0.5
```

Rotating target (see limitation below):

```bash
ros2 launch free_target_object free_target_object.launch.py \
  x:=0.6 z:=0.5 wz:=0.5          # 0.5 rad/s about world Z
```

### Arguments

| Arg | Default | Meaning |
|-----|---------|---------|
| `name` | `free_target_object` | model name in Gazebo |
| `world` | `` (empty) | target world name; empty ⇒ resolved from the `/gazebo/worlds` service |
| `x y z` | `0.0` | spawn position [m] |
| `roll pitch yaw` | `0.0` | spawn orientation [rad] |
| `mass` | `5.0` | link mass [kg] |
| `size_x size_y size_z` | `0.2` | box edge lengths [m] |
| `ixx iyy izz` | `0.0` | principal inertia [kg m²]; `0` ⇒ use the homogeneous-box formula from `mass` + `size_*` |
| `wx wy wz` | `0.0` | initial angular velocity [rad/s] (world frame) |
| `vx vy vz` | `0.0` | initial linear velocity [m/s] (world frame) |

Default inertia for the 5 kg / 0.2 m cube: `Ixx = Iyy = Izz = 5·(0.2²+0.2²)/12 ≈ 0.03333 kg m²`.

## How the initial twist is applied — and its limitation

Ignition **Fortress has no service to set an entity's velocity** (only
`/world/<world>/set_pose`), and `EntityFactory` (the spawn message) has no
velocity field. The only velocity mechanism that loads on an
**already-running server with no extra setup** is the built-in
`VelocityControl` system plugin.

* **`wx=wy=wz=vx=vy=vz=0` (default):** the only plugin in the expanded SDF is
  the observation-only `OdometryPublisher`. The body is genuinely free —
  nothing drives it after spawn. This is the correct setup for VIM validation.

* **any component non-zero:** the SDF gains a `VelocityControl` plugin seeded
  with `<initial_linear>` / `<initial_angular>`. **`VelocityControl` re-asserts
  the commanded twist every simulation step**, so `omega` is *held constant*:
  * For a **homogeneous cube** (isotropic inertia — the default) torque-free
    rotation is already constant-`omega`, so this is physically exact.
  * For a deliberately **anisotropic** inertia it suppresses nutation
    (`I·omega_dot = tau` is not integrated).

  A true one-shot "kick then let it tumble freely" for the anisotropic case
  would require a small custom Ignition system plugin (setting
  `AngularVelocityCmd` once, which `Physics` consumes and clears). Not
  included here to keep the package dependency-free and build-free; add it if
  the anisotropic free-tumble case becomes relevant.
