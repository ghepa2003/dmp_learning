# Testing Plane

This repository provides a **ROS2 + Ignition Fortress Gazebo** code to:

- Spawn a **Xacro testing plane model** into an **already running Ignition simulation**
- Use **gz_ros2_control**
- Load **ROS2 controllers**
- Command joints from a **Python node**
- Generate **sinusoidal motion**

The testing plane model is implemented as a **6-DOF virtual plane** composed of prismatic and revolute joints allowing full pose control.

---

# Table of Contents

- [Build](#build)
- [Robot Model](#robot-model)
- [Controllers](#controllers)
- [Launch File](#launch-file)
- [Sinusoidal Command Node](#sinusoidal-command-node)
- [Running the Simulation](#running-the-simulation)
- [Topics](#topics)
- [Authors and Maintainers](#authors-and-maintainers)
---

# Build

Clone the repository inside your ROS2 workspace and build it with **colcon build**.

Source the workspace:

```
source install/setup.bash
```

---
# Robot Model

The testing plane is defined in:

```
xacro/testing_plane.xacro
```

The model implements a **6-DOF kinematic chain**:

| Joint | Type | Motion |
|------|------|------|
| testing_plane_trasl_x_joint | prismatic | X translation |
| testing_plane_trasl_y_joint | prismatic | Y translation |
| testing_plane_trasl_z_joint | prismatic | Z translation |
| testing_plane_rot_x_joint | revolute | Roll |
| testing_plane_rot_y_joint | revolute | Pitch |
| testing_plane_rot_z_joint | revolute | Yaw |

Kinematic structure:

```
base
 └── trasl_x
      └── trasl_y
           └── trasl_z
                └── rot_x
                     └── rot_y
                          └── rot_z
                               └── testing_plane_link
```

# Controllers

Defined in:

```
config/testing_plane_controllers.yaml
```

## Controller Manager

```
update_rate: 500
use_sim_time: true
```

## Controllers

### Joint State Broadcaster

Publishes:

```
/testing_plane/joint_states
```

### Position Controller

```
type: position_controllers/JointGroupPositionController
```

Command topic:

```
/testing_plane/testing_plane_position_controller/commands
```

Message type:

```
std_msgs/msg/Float64MultiArray
```

Joint order:

```
[
trasl_x,
trasl_y,
trasl_z,
rot_x,
rot_y,
rot_z
]
```

---

# Launch File

File:

```
launch/testing_plane.launch.py
```

Responsibilities:

1. Convert **Xacro → URDF**
2. Start **robot_state_publisher**
3. Spawn the robot into an **already running Ignition simulation**
4. Load controllers using `ros2 control`
5. Delay controller loading to avoid initialization issues

The delay is implemented with:

```python
TimerAction(period=1.0)
```

---

# Sinusoidal Command Node

File:

```
scripts/position_command.py
```

This ROS2 node publishes commands to:

```
/testing_plane/testing_plane_position_controller/commands
```

It generates a **sinusoidal motion on the Y rotation joint**.

Motion equation:

```
rot_y = A * sin(2π f t)
```

Parameters:

| Parameter | Value |
|-----------|------|
| Amplitude | 0.5 rad |
| Frequency | 0.2 Hz |
| Publish rate | 100 Hz |

---

# Running the Simulation

## 1 Start Ignition Gazebo


The launch file assumes the **simulation is already running**.

---

## 2 Launch the testing plane

```
ros2 launch testing_plane testing_plane.launch.py
```

---

## 3 Run the position controller


```
cd scripts

chmod +x position_command.py

./position_command.py
```

The plane will start **oscillating around the Y axis**.

---

# Topics

## Controller Commands

```
/testing_plane/testing_plane_position_controller/commands
```

Type:

```
std_msgs/msg/Float64MultiArray
```

Example manual command:

```
ros2 topic pub /testing_plane/testing_plane_position_controller/commands \
std_msgs/msg/Float64MultiArray \
"{data: [0,0,0,0,0.5,0]}"
```

---

## Joint States

```
/testing_plane/joint_states
```

Type:

```
sensor_msgs/msg/JointState
```

## Authors and Maintainers:
- Giuseppe Alfonso
