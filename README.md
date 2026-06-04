# SO101 ROS 2 — Plug-and-Play Workspace

ROS 2 Jazzy workspace for the **LeRobot SO-ARM101** (SO101) robotic arm: description, simulation (Gazebo Harmonic), ROS 2 Control, and MoveIt 2 with a **single-launch** setup (Gazebo + MoveIt + RViz).

---

**Feel Free create a branch with 'ros_distro_name' to add support for other ROS distros.**

---


## Features
- **Hardware Interface** integrated with so_arm_100_hardware.
- **ROS 2** (Jazzy recommended) with **Gazebo Harmonic**
- **Unified launch**: one command starts Gazebo, MoveIt `move_group`, and RViz — plan and execute from MoveIt and see the robot move in Gazebo
- **URDF/Xacro** with updated calibration (`so101_new_calib`)
- **ROS 2 Control**  for real and fake(gazebo) hardware.
- **MoveIt 2** motion planning for arm (OMPL)
- **RViz** visualization

---

## Prerequisites

- **ROS 2** (tested on **Jazzy**; Rolling/Kilted may work)
- **Gazebo Harmonic** (via `ros_gz_*` packages)
- Standard build tools: `colcon`, `rosdep`

---

## Clone (with hardware interface submodule)

This repo includes the **LeRobot SO101 hardware interface** as a Git submodule ([so_arm_100_hardware](https://github.com/brukg/so_arm_100_hardware)). Clone with submodules so it is included:

```bash
git clone --recurse-submodules git@github.com:dhruvilmahidhariya/so101_ros2.git
cd so101_ros2
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

---

## One-Command Setup (Plug and Play)

From the workspace root (after cloning):

```bash
# 1. Source your ROS 2 distro (e.g. Jazzy)
source /opt/ros/jazzy/setup.bash

# 2. Install dependencies and build
./setup.sh
```

Then source the workspace in every new terminal:

```bash
source install/setup.bash
```

**What `setup.sh` does:** runs `rosdep update`, installs all dependencies from `src` package manifests, and builds with `colcon build --symlink-install`. No manual dependency installation needed.

---

## Usage

### Single launch: sim or real

One launch file with a `robot_mode` argument (think: **fake_hardware**, `true` = sim, `false` = real, or use **both**):

```bash
# Simulation only (Gazebo + MoveIt + RViz) — default
ros2 launch lerobot_moveit so101.launch.py

# Real robot only (no Gazebo)
ros2 launch lerobot_moveit so101.launch.py robot_mode:=real

```

- **`robot_mode:=sim`** (default): Gazebo + MoveIt + RViz. Plan and Execute drives the simulated robot.
- **`robot_mode:=real`**: Real hardware + MoveIt + RViz. Connect the arm via USB (e.g. `/dev/ttyACM1`); override with `serial_port:=/dev/ttyUSB0` if needed (passed to the real controller).


In RViz use **Motion Planning** and **Execute**; planning library **OMPL** for arm and gripper.

**Real robot: no movement?**
- **Arm**: In the Motion Planning panel, set **Planning Group** to **kienmatics** (not "gripper"). Then move the interactive marker to a new pose and use Plan & Execute.
- **Gripper**: Set Planning Group to **gripper**, change the gripper target (drag the marker or set a new joint goal), then Plan & Execute.
- If the target pose is the same as the current one, the plan has zero motion and the controller still reports "Goal reached" with no visible movement.
- Check USB: `ls -l /dev/ttyACM1`, user in `dialout` group; override port with `serial_port:=/dev/ttyUSB0` if the arm is on a different port.

### Other launch files

- **RViz only** (no Gazebo):  
  `ros2 launch lerobot_description so101_display.launch.py`
- **Gazebo only** (no MoveIt):  
  `ros2 launch lerobot_description so101_gazebo.launch.py`  
  Then controllers:  
  `ros2 launch lerobot_controller so101_controller.launch.py`
- **MoveIt only** (existing move_group + RViz, no sim):  
  `ros2 launch lerobot_moveit so101_moveit.launch.py`  
  (Requires robot state from elsewhere or demo mode.)

---

## Workspace layout

| Package               | Description |
|-----------------------|-------------|
| `lerobot_description` | URDF/Xacro, meshes, Gazebo and RSP launch files |
| `lerobot_controller`  | ROS 2 Control controller config and launch |
| `lerobot_moveit`      | MoveIt 2 config (so101_new_calib), unified Gazebo+MoveIt+Rviz launch |
| `so_arm_100_hardware` | **Submodule** — ROS 2 Control hardware interface and ST3215 servo API (same protocol as SO101 servos). Used for real SO101; joint names and calibration come from this repo. |


## Credit and origin

This repository is based on and extends **[lerobot_ws](https://github.com/Pavankv92/lerobot_ws)** by **Pavankv92**.

- **Original work**: [Pavankv92/lerobot_ws](https://github.com/Pavankv92/lerobot_ws) — ROS 2 package for LeRobot SO-ARM101 (RViz, Gazebo, ROS 2 Control, MoveIt 2).
- **Adaptations in this repo**:
  - Cloned and modified the workspace structure and packages.
  - Updated URDF/calibration (`so101_new_calib`).
  - Added a **unified launch** (Gazebo + MoveIt + RViz in one command).
  - Generated a new MoveIt package and config; overall structure and many packages remain adapted from the original.

Thanks to **Pavankv92** for the original LeRobot SO-ARM101 ROS 2 integration and structure.

---

## License

Apache-2.0 (see [LICENSE](LICENSE)). This project is based on RobotStudio SO-ARM100 and adheres to their license terms.