#!/usr/bin/env python3
"""Eye-in-hand calibration for SO101 (camera on wrist).

Modes:
  A) Multi-pose hand-eye: Enter=sample, c=solve
  B) One-shot with known board pose in base: press o
       T_wrist_cam = inv(T_base_wrist) @ T_base_board @ inv(T_cam_board)

--board-xyz is the CENTER of the inner-corner grid in base_link [m].
OpenCV pattern origin is corner (0,0); the script converts center → origin.
Default orientation: board axes = base axes (X same direction, Z up).
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
from rclpy.time import Time
from sensor_msgs.msg import Image


METHODS = [
    ("Tsai", cv2.CALIB_HAND_EYE_TSAI),
    ("Park", cv2.CALIB_HAND_EYE_PARK),
    ("Horaud", cv2.CALIB_HAND_EYE_HORAUD),
    ("Andreff", cv2.CALIB_HAND_EYE_ANDREFF),
    ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
]


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
    if msg.encoding in ("yuv422_yuy2", "yuyv", "yuv422"):
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(buf, cv2.COLOR_YUV2BGR_YUY2)
    raise RuntimeError(f"Unsupported encoding: {msg.encoding}")


def quat_to_R(x, y, z, w) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rpy_to_R(roll, pitch, yaw) -> np.ndarray:
    """URDF fixed-axis RPY: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


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


def parse_vec3(s: str, name: str) -> np.ndarray:
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must be x,y,z got {s!r}")
    v = np.array([float(p) for p in parts], dtype=np.float64)
    if np.max(np.abs(v)) > 10.0:
        v = v * 0.001
        print(f"[note] interpreted {name} as millimeters → {v} m")
    return v


def write_result(path: Path, effector: str, T: np.ndarray, header_lines: list[str]):
    x, y, z, rr, pp, yy = mat_to_xyz_rpy(T)
    tnorm = float(np.linalg.norm(T[:3, 3]))
    text = (
        "\n".join(header_lines)
        + "\n"
        + f"xyz_m: [{x:.6f}, {y:.6f}, {z:.6f}]\n"
        + f"rpy_rad: [{rr:.6f}, {pp:.6f}, {yy:.6f}]\n"
        + f"rpy_deg: [{math.degrees(rr):.3f}, {math.degrees(pp):.3f}, {math.degrees(yy):.3f}]\n"
        + f"|t|_m: {tnorm:.6f}\n"
        + f"\n# URDF:\n"
        + f'<joint name="wrist_to_camera_optical" type="fixed">\n'
        + f'  <parent link="{effector}"/>\n'
        + f'  <child link="camera_optical_frame"/>\n'
        + f'  <origin xyz="{x:.6f} {y:.6f} {z:.6f}" rpy="{rr:.6f} {pp:.6f} {yy:.6f}"/>\n'
        + f"</joint>\n"
        + f"\n# Preview:\n"
        + f"# ros2 run tf2_ros static_transform_publisher \\\n"
        + f"#   --x {x:.6f} --y {y:.6f} --z {z:.6f} \\\n"
        + f"#   --roll {rr:.6f} --pitch {pp:.6f} --yaw {yy:.6f} \\\n"
        + f"#   --frame-id {effector} --child-frame-id camera_optical_guess\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(text)
    return tnorm


class HandEyeNode(Node):
    def __init__(self, args):
        super().__init__("so101_handeye")
        self.args = args
        self.K, self.D = load_intrinsics(Path(args.intrinsics))
        cols, rows = (int(x) for x in args.size.split("x"))
        self.pattern_size = (cols, rows)
        self.square = float(args.square)
        self.base = args.base_frame
        self.effector = args.wrist_frame

        obj = np.zeros((rows * cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj *= self.square
        self.obj_points = obj
        self.board_center_in_obj = np.array(
            [(cols - 1) * self.square * 0.5, (rows - 1) * self.square * 0.5, 0.0],
            dtype=np.float64,
        )

        # Known board CENTER in base; axes = base (X same, Z up)
        self.mid_base = parse_vec3(args.board_xyz, "board_xyz")
        rpy = parse_vec3(args.board_rpy, "board_rpy")
        R_b = rpy_to_R(rpy[0], rpy[1], rpy[2])
        origin_base = self.mid_base - R_b @ self.board_center_in_obj
        self.T_base_board = rt_to_T(R_b, origin_base)
        self.get_logger().info(
            f"Known board CENTER in {self.base}: {self.mid_base} m  rpy={rpy} "
            f"(pattern origin → {origin_base})"
        )

        self._map1 = self._map2 = self._new_K = None
        self.bridge_img = None
        self._img_stamp: Time | None = None

        self.R_gripper2base: list[np.ndarray] = []
        self.t_gripper2base: list[np.ndarray] = []
        self.R_target2cam: list[np.ndarray] = []
        self.t_target2cam: list[np.ndarray] = []

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Image, args.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"corners {args.size} square={self.square}m | {self.base}->{self.effector} | "
            f"Enter=sample c=multi o=ONE-SHOT q=quit"
        )

    def _on_image(self, msg: Image):
        self.bridge_img = imgmsg_to_cv2(msg)
        self._img_stamp = Time.from_msg(msg.header.stamp)

    def _undistort(self, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        if self._map1 is None:
            self._new_K, _ = cv2.getOptimalNewCameraMatrix(
                self.K, self.D, (w, h), alpha=0.0, newImgSize=(w, h)
            )
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self.K, self.D, None, self._new_K, (w, h), cv2.CV_16SC2
            )
        return cv2.remap(bgr, self._map1, self._map2, cv2.INTER_LINEAR), self._new_K

    def _lookup_base_effector(self, stamp: Time | None):
        when = stamp if stamp is not None else Time()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base, self.effector, when, timeout=Duration(seconds=0.5)
            )
        except tf2_ros.TransformException:
            tf = self.tf_buffer.lookup_transform(
                self.base, self.effector, Time(), timeout=Duration(seconds=1.0)
            )
        t = tf.transform.translation
        q = tf.transform.rotation
        return quat_to_R(q.x, q.y, q.z, q.w), np.array(
            [[t.x], [t.y], [t.z]], dtype=np.float64
        )

    def _detect_board_in_camera(self):
        if self.bridge_img is None:
            self.get_logger().warn("No image yet")
            return None
        bgr = (
            self.bridge_img
            if self.bridge_img.ndim == 3
            else cv2.cvtColor(self.bridge_img, cv2.COLOR_GRAY2BGR)
        )
        und, new_K = self._undistort(bgr)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not ok:
            self.get_logger().warn("Chessboard NOT detected")
            return None
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        ok, rvec, tvec = cv2.solvePnP(self.obj_points, corners, new_K, None)
        if not ok:
            self.get_logger().warn("solvePnP failed")
            return None
        R_t2c, _ = cv2.Rodrigues(rvec)
        return R_t2c, tvec.reshape(3, 1)

    def try_sample(self) -> bool:
        det = self._detect_board_in_camera()
        if det is None:
            return False
        R_t2c, t_t2c = det
        try:
            R_g2b, t_g2b = self._lookup_base_effector(self._img_stamp)
        except Exception as e:
            self.get_logger().error(f"TF {self.base}->{self.effector} failed: {e}")
            return False
        self.R_gripper2base.append(R_g2b)
        self.t_gripper2base.append(t_g2b)
        self.R_target2cam.append(R_t2c)
        self.t_target2cam.append(t_t2c)
        n = len(self.R_gripper2base)
        self.get_logger().info(
            f"Sample {n} | board distance ≈ {float(np.linalg.norm(t_t2c)):.3f} m"
        )
        return True

    def one_shot(self) -> bool:
        """One view + known board in base → camera in wrist."""
        det = self._detect_board_in_camera()
        if det is None:
            return False
        R_t2c, t_t2c = det
        try:
            R_g2b, t_g2b = self._lookup_base_effector(self._img_stamp)
        except Exception as e:
            self.get_logger().error(f"TF {self.base}->{self.effector} failed: {e}")
            return False

        T_base_wrist = rt_to_T(R_g2b, t_g2b)
        T_cam_board = rt_to_T(R_t2c, t_t2c)
        T_wrist_cam = (
            np.linalg.inv(T_base_wrist)
            @ self.T_base_board
            @ np.linalg.inv(T_cam_board)
        )

        T_base_board_est = T_base_wrist @ T_wrist_cam @ T_cam_board
        mid_est = (
            T_base_board_est[:3, :3] @ self.board_center_in_obj + T_base_board_est[:3, 3]
        )
        err = float(np.linalg.norm(mid_est - self.mid_base))
        self.get_logger().info(
            f"One-shot | board-center reproj err ≈ {err*1000:.1f} mm "
            f"(~0 means detection matches your measure)"
        )

        out = Path(self.args.out)
        tnorm = write_result(
            out,
            self.effector,
            T_wrist_cam,
            [
                f"# One-shot ^{self.effector}T_camera",
                f"# parent: {self.effector}  child: camera_optical_frame",
                f"# board_center_base_m: {list(self.mid_base)}",
                f"# board_rpy_rad: {self.args.board_rpy}",
                f"# board_center_reproj_err_m: {err:.6f}",
                "method: one_shot_known_board",
            ],
        )
        self.get_logger().info(f"Wrote {out}  |t|={tnorm*1000:.1f} mm")
        return True

    def _board_rms_in_base(self, T_effector_cam: np.ndarray) -> float:
        origins = []
        for R_g2b, t_g2b, R_t2c, t_t2c in zip(
            self.R_gripper2base,
            self.t_gripper2base,
            self.R_target2cam,
            self.t_target2cam,
        ):
            T_b_t = (
                rt_to_T(R_g2b, t_g2b) @ T_effector_cam @ rt_to_T(R_t2c, t_t2c)
            )
            origins.append(T_b_t[:3, 3])
        origins = np.stack(origins, axis=0)
        mean = origins.mean(axis=0)
        return float(np.sqrt(((origins - mean) ** 2).sum(axis=1).mean()))

    def solve(self):
        n = len(self.R_gripper2base)
        if n < 8:
            self.get_logger().error(f"Need >= 8 samples, have {n}")
            return
        print(f"\n=== Hand-eye methods (n={n})  effector={self.effector} ===")
        results = []
        for name, method in METHODS:
            try:
                R_c2g, t_c2g = cv2.calibrateHandEye(
                    self.R_gripper2base,
                    self.t_gripper2base,
                    self.R_target2cam,
                    self.t_target2cam,
                    method=method,
                )
            except cv2.error as e:
                self.get_logger().warn(f"{name} failed: {e}")
                continue
            T = rt_to_T(R_c2g, t_c2g)
            rms = self._board_rms_in_base(T)
            tnorm = float(np.linalg.norm(t_c2g))
            x, y, z, rr, pp, yy = mat_to_xyz_rpy(T)
            results.append((rms, name, T, tnorm))
            print(
                f"  {name:10s}  xyz=({x:+.4f}, {y:+.4f}, {z:+.4f})  "
                f"|t|={tnorm*1000:.1f} mm  board_rms={rms*1000:.1f} mm"
            )
        if not results:
            self.get_logger().error("All methods failed")
            return
        results.sort(key=lambda r: r[0])
        best_rms, best_name, T, tnorm = results[0]
        write_result(
            Path(self.args.out),
            self.effector,
            T,
            [
                f"# ^{self.effector}T_camera",
                f"# selected: {best_name}  board_rms_m: {best_rms:.6f}",
                f"samples: {n}",
            ],
        )
        self.get_logger().info(f"Selected {best_name} | wrote {self.args.out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--intrinsics",
        default=str(
            Path.home()
            / "work/so101_ros2/Docs/calibration_patterns/intrinsics/innomaker_640x480.yaml"
        ),
    )
    parser.add_argument("--size", default="9x6")
    parser.add_argument("--square", type=float, default=0.014)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--wrist-frame", default="wrist_link")
    parser.add_argument(
        "--board-xyz",
        default="0.24,0,0",
        help="Board CENTER in base [m] (240 mm on +X). Pass 240,0,0 for mm.",
    )
    parser.add_argument(
        "--board-rpy",
        default="0,0,0",
        help="Board rpy in base [rad]: X||base X, Z up",
    )
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
    print(
        "\nKeys: [Enter]=sample  [c]=multi-solve  [o]=ONE-SHOT  [q]=quit\n"
        f"One-shot board CENTER = {args.board_xyz} m, rpy={args.board_rpy} "
        f"in {args.base_frame} (X||base, Z up).\n"
        "Wait for DETECTED, press o.\n"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
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
            else:
                bgr = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                und, _ = node._undistort(bgr)
                vis = und
                gray = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
                ok, corners = cv2.findChessboardCorners(gray, node.pattern_size)
                if ok:
                    cv2.drawChessboardCorners(vis, node.pattern_size, corners, True)
                status = (
                    f"{'DETECTED' if ok else 'NO DETECT'} | "
                    f"samples={len(node.R_gripper2base)} | o=one-shot"
                )
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
            if key in (13, 32):
                node.try_sample()
            elif key in (ord("c"), ord("C")):
                node.solve()
            elif key in (ord("o"), ord("O")):
                node.one_shot()
            elif key in (ord("q"), ord("Q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
