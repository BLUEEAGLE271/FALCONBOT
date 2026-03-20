#!/usr/bin/env python3
"""
VPI Rectified Camera Node — AprilTag edition
=============================================
Simplified pipeline vs ArUco version:
  - Removed: gray conversion, contrast boost
  - Kept:    VPI CUDA fisheye remap only
  - Output:  RGB8 (required by isaac_ros_apriltag)
  - Topics:  /image + /camera_info (isaac_ros_apriltag defaults)
  - D=[0]*5 in CameraInfo — image already rectified, do NOT undistort again
"""

import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import vpi


class VPIRectifiedNode(Node):
    def __init__(self):
        super().__init__('vpi_rectified_node')

        self.frame_id = "camera_optical_frame"
        self.bridge   = CvBridge()

        # --- CONFIGURATION ---
        self.scale_factor = 0.25
        self.DIM = (820, 616)  # After 0.25x scaling: 820x1232

        self.K = np.array([
            [1813.000558 * self.scale_factor, 0., 1634.163912 * self.scale_factor],
            [0., 1826.621610 * self.scale_factor, 1258.598589 * self.scale_factor],
            [0., 0., 1.]
        ])

        # Fisheye distortion coefficients — used for VPI warp map ONLY
        # NOT published in CameraInfo (D=0 there because image is already rectified)
        self.D = [-0.022401, -0.022708, 0.014203, -0.009313]

        # --- VPI WARP MAP (CUDA fisheye correction) ---
        self.get_logger().info("Initializing VPI fisheye correction (CUDA)...")
        self.grid         = vpi.WarpGrid(self.DIM)
        self.X            = np.eye(3, 4)
        K_2x3             = self.K[0:2, :]
        self.vpi_warp_map = vpi.WarpMap.fisheye_correction(
            self.grid,
            K=K_2x3,
            X=self.X,
            mapping=vpi.FisheyeMapping.EQUIDISTANT,
            coeffs=self.D
        )

        # Single GPU image buffer — RGB8 output for isaac_ros_apriltag
        self.vpi_rectified = vpi.Image(self.DIM, vpi.Format.RGB8)

        # --- FPS TRACKING ---
        self._fps_count = 0
        self._fps_last  = time.time()

        # --- ROS PUBLISHERS ---
        # /image and /camera_info are isaac_ros_apriltag's default input topics
        self.image_pub = self.create_publisher(Image,      '/image',       20)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera_info', 20)

        # --- CAMERA ---
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
            return

        # --- CAMERA THREAD ---
        self._running    = True
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

        self.get_logger().info(
            f"Camera ready: {self.DIM[0]}x{self.DIM[1]} | "
            f"Pipeline: VPI CUDA fisheye remap → RGB8 | "
            f"Publishing to /image + /camera_info"
        )

    def gstreamer_pipeline(self, sensor_id=0):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} "
            f"exposuretimerange='13000 60000' gainrange='1 2' ! "
            f"video/x-raw(memory:NVMM), width=3280, height=2464, framerate=21/1 ! "
            f"nvvidconv compute-hw=1 ! "
            f"video/x-raw(memory:NVMM), "
            f"width={self.DIM[0]}, height={self.DIM[1]}, format=NV12 ! "
            f"nvvidconv compute-hw=1 ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink max-buffers=1 drop=true"
        )

    def _camera_loop(self):
        while self._running and rclpy.ok():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            self._process_frame(frame)

    def _process_frame(self, frame):
        try:
            with vpi.Backend.CUDA:
                # Only step: fisheye rectification → RGB8
                # AprilTag GPU detector handles everything else internally
                vpi.asimage(frame).remap(
                    self.vpi_warp_map,
                    interp=vpi.Interp.LINEAR,
                    out=self.vpi_rectified
                )

            # Single download — Jetson unified memory makes this a cache flush
            rgb_frame = self.vpi_rectified.cpu()

            # FPS at 1Hz
            self._fps_count += 1
            now = time.time()
            if now - self._fps_last >= 1.0:
                self.get_logger().info(
                    f"📷 fps={self._fps_count} | "
                    f"{self.DIM[0]}x{self.DIM[1]} RGB8"
                )
                self._fps_count = 0
                self._fps_last  = now

            now_msg = self.get_clock().now().to_msg()

            # Publish RGB8
            img_msg                 = self.bridge.cv2_to_imgmsg(rgb_frame, "rgb8")
            img_msg.header.stamp    = now_msg
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # CameraInfo — D must be all zeros
            # Image is already rectified by VPI — AprilTag must NOT undistort again
            info_msg                  = CameraInfo()
            info_msg.header           = img_msg.header
            info_msg.height           = self.DIM[1]
            info_msg.width            = self.DIM[0]
            info_msg.distortion_model = "plumb_bob"
            info_msg.d                = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k                = self.K.flatten().tolist()
            info_msg.r                = np.eye(3).flatten().tolist()
            P                         = np.zeros((3, 4))
            P[:3, :3]                 = self.K
            info_msg.p                = P.flatten().tolist()
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f"Pipeline Error: {e}")

    def destroy_node(self):
        self._running = False
        if self._cam_thread.is_alive():
            self._cam_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VPIRectifiedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()