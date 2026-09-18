# SO101 ROS 2 — 6-DoF Workspace

[▶ Watch the real 6-DoF SO101 robot demo](Docs/Real_video.mp4?raw=true)

ROS 2 Jazzy workspace for a **6-DoF LeRobot SO-ARM101**: URDF, Gazebo Harmonic, ROS 2 Control, AND MoveIt 2.

The stock SO101 is 5-DoF. This repo adds **`elbow_rotate`** between the forearm (`lower_arm_link` / LINK4) and **`link5_link`**, so MoveIt can plan full 6-DoF Cartesian poses.

![6-DoF SO101 assembly](Docs/SO101_dof6_assembled_mod.png)

![MoveIt / RViz planning the 6-DoF arm](Docs/rviz.png)

Feel free to create a branch named after a ROS distro to add support for other releases.

---

## What this repo is

This is the **description, planning, and launch stack** for the modified arm:

- 6-DoF URDF/Xacro (`so101_new_calib`) with LINK4 + link5 meshes
- Unified launch: Gazebo + MoveIt `move_group` + RViz in one command
- Real-robot launch with the same MoveIt config
- Calibration and controller YAML for the extra joint (servo ID 7)
- Wrist camera frame (`camera_optical_frame`) from hand-eye calibration

The Feetech STS3215 driver used on the real arm lives in `so_arm_100_hardware`. Joint names, limits, and calibration for **this** 6-DoF robot are defined here, not in that package.

---

## Features

- **6-DoF kinematics**: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `elbow_rotate`, `wrist_flex`, `wrist_roll` (+ gripper)
- **MoveIt 2** (OMPL) on the `kinematics` planning group, with **Home** and **Extended** named states
- **Unified launch** for sim (`robot_mode:=sim`) or real (`robot_mode:=real`)
- **ROS 2 Control** for Gazebo and the real bus
- **RViz** Motion Planning with the interactive marker on `eef_frame_link`
- **Wrist camera**: Innomaker U20CAM optical frame on `wrist_link` (`so101_camera.xacro`)

---

## Prerequisites

- **ROS 2** (tested on **Jazzy**; Rolling/Kilted may work)
- **Gazebo Harmonic** (`ros_gz_*`)
- `colcon`, `rosdep`

---

## Clone and build

```bash
git clone --recurse-submodules git@github.com:dhruvilmahidhariya/so101_ros2.git
cd so101_ros2
source /opt/ros/jazzy/setup.bash
./setup.sh
source install/setup.bash
```

`setup.sh` runs `rosdep` and `colcon build --symlink-install`. Source `install/setup.bash` in every new terminal.

If you cloned without submodules, the real-robot driver is pulled with:

```bash
git submodule update --init --recursive
```

---

## 6-DoF kinematics

| Joint | Role | Motor ID |
|-------|------|----------|
| `shoulder_pan` | Base yaw | 1 |
| `shoulder_lift` | Shoulder pitch | 2 |
| `elbow_flex` | Elbow pitch | 3 |
| `elbow_rotate` | Forearm roll (added DoF) | 7 |
| `wrist_flex` | Wrist pitch | 4 |
| `wrist_roll` | Wrist roll | 5 |
| `gripper` | Jaw (own MoveIt group) | 6 |

Chain: `base_link` → `shoulder_link` → `upper_arm_link` → `lower_arm_link` → `link5_link` → `wrist_link` → `gripper_link` → `eef_frame_link`.

**Home** is all zeros and matches calibration `center.ticks: 2048` (URDF 0). That pose is a wrist singularity (`elbow_rotate` lines up with `wrist_roll` when `wrist_flex ≈ 0`). For Cartesian planning, start from the **Extended** named state.

Calibration: `src/lerobot_controller/config/calibration_so101_real.yaml`. Non-gripper joints map as `2048 + direction * radians * 4096 / 2π`.

---

## Usage

### Sim or real

```bash
# Gazebo + MoveIt + RViz (default)
ros2 launch lerobot_moveit so101.launch.py

# Real arm + MoveIt + RViz
ros2 launch lerobot_moveit so101.launch.py robot_mode:=real
```

Default serial port is `/dev/ttyACM0`. Override with `serial_port:=/dev/ttyUSB0` if needed.

In RViz use **Motion Planning** → **Execute** (OMPL). Planning group **kinematics** for the arm, **gripper** for the jaw. Named states: **Home**, **Extended**, **Gripper Open**, **Gripper Closed**.

**Real robot: no movement?**
- Set Planning Group to **kinematics**, move the marker, then Plan & Execute.
- A goal identical to the current pose reports “Goal reached” with no motion.
- Check `ls -l /dev/ttyACM0` and that your user is in `dialout`.

Start from **Extended**, not Home, to avoid the wrist singularity.

### Other launches

- RViz only: `ros2 launch lerobot_description so101_display.launch.py`
- Gazebo only: `ros2 launch lerobot_description so101_gazebo.launch.py` then `ros2 launch lerobot_controller so101_controller.launch.py`
- MoveIt only: `ros2 launch lerobot_moveit so101_moveit.launch.py`

---

## Wrist camera calibration

Patterns and helpers live under [`Docs/calibration_patterns/`](Docs/calibration_patterns/). The live URDF frame is [`src/lerobot_description/urdf/so101_camera.xacro`](src/lerobot_description/urdf/so101_camera.xacro), included from `so101.urdf.xacro`.

**Current workspace result** (Innomaker U20CAM on `wrist_link`, Tsai, 18 samples, board_rms ≈ 5 mm):

| | value |
|--|--|
| Intrinsics | `Docs/calibration_patterns/intrinsics/innomaker_640x480.yaml` (640×480) |
| Extrinsics YAML | `Docs/calibration_patterns/extrinsics/handeye_wrist_camera.yaml` |
| `wrist_link` → `camera_optical_frame` xyz (m) | `-0.050360 -0.045513 0.018273` |
| rpy (rad) | `1.090172 1.532300 -0.331997` |
| \|t\| | ≈ 0.070 m |
| Board square size used | `0.0125` m (remeasure for your display) |

In Isaac Lab the same extrinsics are applied at **runtime** via `So101WristCameraCalibCfg` in `isaaclab_assets/robots/so101.py` (camera is **not** baked into the robot USD). Replace that cfg when you recalibrate.

### 1. Pattern setup

Use `Docs/calibration_patterns/chessboard_9x6.png` (or `chessboard_9x6_CLEAN.png`):

1. Open the PNG fullscreen on a phone/tablet; max brightness; auto-brightness **off**.
2. Measure **one black square** edge with a ruler (mm → meters, e.g. 12.5 mm → `0.0125`).
3. For hand-eye, prop the board **fixed on the table** (do not hold it). Move the **arm**, not the board.

OpenCV / ROS size: **inner corners** `9x6` (a 10×7 square board).

### 2. Intrinsics

Calibrate the Innomaker (or other USB cam) once, then save the YAML under `Docs/calibration_patterns/intrinsics/` (example: `innomaker_640x480.yaml`).

```bash
# Prefer yuyv on this camera (mjpeg2rgb can yield flat gray frames)
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video2 \
  -p pixel_format:=yuyv \
  -p image_width:=640 -p image_height:=480 \
  -p framerate:=30.0

# Optional: lock exposure (adjust device node as needed)
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=500

# other terminal
ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.0125 \
  image:=/image_raw camera:=/camera
```

Save / copy the resulting camera matrix and distortion into the intrinsics YAML used below.

### 3. Hand-eye (eye-in-hand)

Prereqs: real robot TF up (`so101.launch.py robot_mode:=real`), camera on `/image_raw`, board fixed in the world. Keep torque on and **jog with MoveIt / RViz** between samples (no limp / torque toggles).

Use **system** Python for ROS Jazzy (`/usr/bin/python3`). Do **not** run the script inside an Isaac Lab env (rclpy breaks).

```bash
# terminal 1
ros2 launch lerobot_moveit so101.launch.py robot_mode:=real

# terminal 2 — USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video2 \
  -p pixel_format:=yuyv \
  -p image_width:=640 -p image_height:=480 \
  -p framerate:=30.0

# terminal 3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 Docs/calibration_patterns/handeye_calibrate.py \
  --intrinsics Docs/calibration_patterns/intrinsics/innomaker_640x480.yaml \
  --size 9x6 \
  --square 0.0125 \
  --image-topic /image_raw \
  --base-frame base_link \
  --wrist-frame wrist_link \
  --out Docs/calibration_patterns/extrinsics/handeye_wrist_camera.yaml
```

In the OpenCV window (image is undistorted for detection):

- **Enter / Space** — record a sample when status shows **DETECTED** (vary **wrist orientation** a lot; keep the full board in view; ≥ 12–18 samples)
- **c** — solve with OpenCV methods (Tsai / Park / Horaud / …); pick lowest board residual; write YAML + URDF snippet
- **o** — optional **one-shot** if you know the board center pose in `base_link` (`--board-xyz` / `--board-rpy`)
- **q** — quit

Trust a solve when **board_rms** is low (a few mm) and methods like Park/Horaud/Tsai agree on xyz within ~1 cm. Preview without restarting the launch:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x -0.050360 --y -0.045513 --z 0.018273 \
  --roll 1.090172 --pitch 1.532300 --yaw -0.331997 \
  --frame-id wrist_link \
  --child-frame-id camera_optical_guess
```

Output parent/child: `wrist_link` → `camera_optical_frame` (OpenCV optical: x-right, y-down, z-forward).

### 4. Apply to URDF (ROS) and Isaac Lab (sim)

**ROS:** copy the printed `<joint>` origin into `src/lerobot_description/urdf/so101_camera.xacro`, then:

```bash
colcon build --packages-select lerobot_description
source install/setup.bash
# restart so101.launch.py, then:
ros2 run tf2_ros tf2_echo wrist_link camera_optical_frame
```

**Isaac Lab:** update `So101WristCameraCalibCfg` in the Isaac Lab repo (`source/isaaclab_assets/.../robots/so101.py`) with the same xyz + quaternion (or rpy→wxyz). The lift task spawns the camera under `wrist_link` at runtime — no USD edit.

```bash
cd <IsaacLab>
source ~/ilabs/bin/activate   # or your Isaac env
TERM=xterm ./isaaclab.sh -p scripts/environments/zero_agent.py \
  --task Isaac-Lift-Cube-SO101-v0 --enable_cameras
```

In the viewport: select the `wrist_cam` prim (under `Robot/so101_new_calib/wrist_link`) or use the Camera sensor view to see the feed.

Re-run hand-eye and update **both** the xacro and `So101WristCameraCalibCfg` whenever the camera mount moves.

---

## Packages

| Package | Role in this workspace |
|---------|------------------------|
| `lerobot_description` | URDF/Xacro, LINK4/link5 meshes, wrist camera frame, display and Gazebo launches |
| `lerobot_controller` | Controller YAML, 6-DoF calibration, real/sim controller launches |
| `lerobot_moveit` | MoveIt config, unified `so101.launch.py` |
| `so_arm_100_hardware` | STS3215 bus driver (dependency for `robot_mode:=real` only) |

---

## Credits

- **Modified 6-DoF link STLs** (LINK4 / link5, elbow-roll upgrade): [rabhishek100/so101-6dof-and-extended-versions](https://github.com/rabhishek100/so101-6dof-and-extended-versions).
- **ROS 2 workspace structure** (RViz, Gazebo, ros2_control, MoveIt): based on [Pavankv92/lerobot_ws](https://github.com/Pavankv92/lerobot_ws).
- **Real-robot STS3215 interface**: [brukg/so_arm_100_hardware](https://github.com/brukg/so_arm_100_hardware).

This repo adds the 6-DoF URDF, MoveIt group, calibration, and  unified launch integration on top of those.

---

## License

Apache-2.0 (see [LICENSE](LICENSE)). This project is based on RobotStudio SO-ARM100 and adheres to their license terms.
