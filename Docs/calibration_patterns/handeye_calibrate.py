#!/usr/bin/env python3
"""Eye-in-hand hand-eye calibration for SO101 (camera on wrist_link).

Prereqs:
  - so101.launch.py robot_mode:=real  (TF + /joint_states)
  - usb_cam publishing /image_raw
  - Chessboard FIXED in the world (do not move it)
  - Move the ARM between samples (MoveIt), keep board fully visible

Controls (terminal):
  Enter / space  - record sample when board is detected
  c              - solve calibration (need >= 8 samples)
  q              - quit
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def load_intrinsics(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    k = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return k, d


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    if msg.encoding in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img
    if msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    raise RuntimeError(f"Unsupported encoding: {msg.encoding}")


def mat_to_xyz_rpy(T: np.ndarray):
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    sy = math.sqrt(T[0, 0] ** 2 + T[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(T[2, 1], T[2, 2])
        pitch = math.atan2(-T[2, 0], sy)
        yaw = math.atan2(T[1, 0], T[0, 0])
    else:
        roll = math.atan2(-T[1, 2], T[1, 1])
        pitch = math.atan2(-T[2, 0], sy)
        yaw = 0.0
    return x, y, z, roll, pitch, yaw


def rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


class HandEyeNode(Node):
    def __init__(self, args):
        super().__init__("so101_handeye")
        self.args = args
        self.K, self.D = load_intrinsics(Path(args.intrinsics))
        cols, rows = (int(x) for x in args.size.split("x"))
        self.pattern_size = (cols, rows)  # inner corners
        self.square = float(args.square)
        self.base = args.base_frame
        self.wrist = args.wrist_frame

        obj = np.zeros((rows * cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj *= self.square
        self.obj_points = obj

        self.bridge_img = None
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # usb_cam typically uses sensor-data QoS (best-effort)
        self.create_subscription(
            Image, args.image_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Hand-eye ready | board {args.size} square={self.square}m | "
            f"TF {self.base} -> {self.wrist} | Enter=sample c=solve q=quit"
        )

    def _on_image(self, msg: Image):
        self.bridge_img = imgmsg_to_cv2(msg)

    def _lookup_base_wrist(self):
        tf = self.tf_buffer.lookup_transform(self.base, self.wrist, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        # quaternion (x,y,z,w) -> R
        r = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        R = self._quat_to_R(r)
        tvec = np.array([t.x, t.y, t.z], dtype=np.float64)
        return R, tvec

    @staticmethod
    def _quat_to_R(q):
        x, y, z, w = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def try_sample(self) -> bool:
        if self.bridge_img is None:
            self.get_logger().warn("No image yet")
            return False
        gray = (
            self.bridge_img
            if self.bridge_img.ndim == 2
            else cv2.cvtColor(self.bridge_img, cv2.COLOR_BGR2GRAY)
        )
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not ok:
            self.get_logger().warn("Chessboard NOT detected")
            return False
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        ok, rvec, tvec = cv2.solvePnP(self.obj_points, corners, self.K, self.D)
        if not ok:
            self.get_logger().warn("solvePnP failed")
            return False
        try:
            R_g2b, t_g2b = self._lookup_base_wrist()
        except Exception as e:
            self.get_logger().error(f"TF {self.base}->{self.wrist} failed: {e}")
            return False

        R_t2c, _ = cv2.Rodrigues(rvec)
        self.R_gripper2base.append(R_g2b)
        self.t_gripper2base.append(t_g2b)
        self.R_target2cam.append(R_t2c)
        self.t_target2cam.append(tvec.reshape(3))
        n = len(self.R_gripper2base)
        self.get_logger().info(f"Sample {n} recorded")
        return True

    def solve(self):
        n = len(self.R_gripper2base)
        if n < 8:
            self.get_logger().error(f"Need >= 8 samples, have {n}")
            return
        R_c2g, t_c2g = cv2.calibrateHandEye(
            self.R_gripper2base,
            self.t_gripper2base,
            self.R_target2cam,
            self.t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        T = rt_to_T(R_c2g, t_c2g)
        x, y, z, rr, pp, yy = mat_to_xyz_rpy(T)
        out = Path(self.args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        text = (
            f"# T_wrist_camera (OpenCV optical frame: x-right y-down z-forward)\n"
            f"# parent: {self.wrist}  child: camera_optical_frame\n"
            f"samples: {n}\n"
            f"xyz_m: [{x:.6f}, {y:.6f}, {z:.6f}]\n"
            f"rpy_rad: [{rr:.6f}, {pp:.6f}, {yy:.6f}]\n"
            f"rpy_deg: [{math.degrees(rr):.3f}, {math.degrees(pp):.3f}, {math.degrees(yy):.3f}]\n"
            f"\n# URDF snippet (optical frame directly on wrist):\n"
            f'<joint name="wrist_to_camera_optical" type="fixed">\n'
            f'  <parent link="{self.wrist}"/>\n'
            f'  <child link="camera_optical_frame"/>\n'
            f'  <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{rr:.6f} {pp:.6f} {yy:.6f}"/>\n'
            f"</joint>\n"
        )
        out.write_text(text, encoding="utf-8")
        print("\n" + text)
        self.get_logger().info(f"Wrote {out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--intrinsics",
        default=str(
            Path.home()
            / "work/so101_ros2/Docs/calibration_patterns/intrinsics/innomaker_640x480.yaml"
        ),
    )
    parser.add_argument("--size", default="9x6", help="inner corners NxM")
    parser.add_argument("--square", type=float, default=0.014)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--wrist-frame", default="wrist_link")
    parser.add_argument(
        "--out",
        default=str(
            Path.home()
            / "work/so101_ros2/Docs/calibration_patterns/extrinsics/handeye_wrist_camera.yaml"
        ),
    )
    args = parser.parse_args()

    rclpy.init()
    node = HandEyeNode(args)
    print("\nKeys: [Enter]=sample  [c]=calibrate  [q]=quit\n")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            # non-blocking key via OpenCV window
            frame = node.bridge_img
            if frame is None:
                vis = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    vis,
                    "Waiting for /image_raw ...",
                    (40, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow("so101_handeye", vis)
            else:
                vis = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                gray = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
                mean = float(gray.mean())
                ok, corners = cv2.findChessboardCorners(gray, node.pattern_size)
                if ok:
                    cv2.drawChessboardCorners(vis, node.pattern_size, corners, ok)
                status = (
                    f"samples={len(node.R_gripper2base)} mean={mean:.0f}  "
                    f"Enter=sample c=solve q=quit"
                )
                if mean < 25:
                    status += "  | IMAGE TOO DARK - raise exposure / light"
                cv2.putText(
                    vis,
                    status,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0) if ok else (0, 0, 255),
                    2,
                )
                cv2.imshow("so101_handeye", vis)
            key = cv2.waitKey(10) & 0xFF
            if key in (13, 32):  # Enter / space
                node.try_sample()
            elif key in (ord("c"), ord("C")):
                node.solve()
            elif key in (ord("q"), ord("Q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
