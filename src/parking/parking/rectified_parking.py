#!/usr/bin/env python3
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
        self.bridge = CvBridge()
        
        # --- 1. CONFIGURATION ---
        # 0.25 = 820x616 — hits 15Hz on Jetson Orin Nano
        # 0.5  = 1640x1232 — too large, drops to ~3fps
        self.scale_factor = 0.25
        self.DIM = (820, 616)
        
        self.K = np.array([
            [1813.000558 * self.scale_factor,    0.      ,  1634.163912 * self.scale_factor],
            [   0.      , 1826.621610 * self.scale_factor,  1258.598589 * self.scale_factor],
            [   0.      ,    0.      ,     1.      ]
        ])
        
        self.D = [-0.022401, -0.022708, 0.014203, -0.009313]

        # --- 2. CONTRAST TUNING ---
        # output = input * scale + offset
        # scale=2.5, offset=-100 → aggressive contrast for paper markers
        # Tune:
        #   Still grey/washed out  → raise scale to 3.0, offset to -120
        #   Whites blown out       → lower offset to -80
        self.contrast_scale  = 2.5
        self.contrast_offset = -100.0

        # --- 3. VPI WARP MAP (CUDA) ---
        self.get_logger().info("Initializing VPI Hardware Maps...")
        self.grid = vpi.WarpGrid(self.DIM)
        self.X = np.eye(3, 4)
        K_2x3 = self.K[0:2, :]
        self.vpi_warp_map = vpi.WarpMap.fisheye_correction(
            self.grid,
            K=K_2x3,
            X=self.X,
            mapping=vpi.FisheyeMapping.EQUIDISTANT,
            coeffs=self.D
        )

        # GPU images — stay on GPU until final .cpu() call
        self.vpi_rectified  = vpi.Image(self.DIM, vpi.Format.RGB8)
        self.vpi_gray       = vpi.Image(self.DIM, vpi.Format.U8)
        self.vpi_contrasted = vpi.Image(self.DIM, vpi.Format.U8)

        # --- 4. FPS TRACKING ---
        self._fps_count = 0
        self._fps_last  = time.time()

        # --- 5. ROS SETUP ---
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 20)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info', 20)

        # --- 6. CAMERA ---
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
            return

        # --- 7. CAMERA THREAD ---
        # Dedicated thread reads frames exactly when they arrive
        # Much better than a ROS timer which fires independently of frame availability
        self._running = True
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

        self.get_logger().info(
            f"Camera ready: {self.DIM[0]}x{self.DIM[1]} | "
            f"VPI: remap → gray → contrast (all CUDA) | "
            f"contrast_scale={self.contrast_scale} offset={self.contrast_offset}"
        )

    def gstreamer_pipeline(self, sensor_id=0):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode=0 ! "
            f"video/x-raw(memory:NVMM), width=3280, height=2464, framerate=15/1 ! "
            f"nvvidconv ! "
            f"video/x-raw, width={self.DIM[0]}, height={self.DIM[1]}, format=BGRx ! "
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
            # All three steps stay on GPU inside a single CUDA context
            with vpi.Backend.CUDA:
                # STEP 1: Fisheye rectification
                vpi.asimage(frame).remap(
                    self.vpi_warp_map,
                    interp=vpi.Interp.LINEAR,
                    out=self.vpi_rectified
                )
                # STEP 2: BGR → Grayscale
                # ArUco only needs mono — drop colour now to halve data size
                self.vpi_rectified.convert(self.vpi_gray)

                # STEP 3: Contrast boost
                # output = input * scale + offset
                self.vpi_gray.convert(
                    self.vpi_contrasted,
                    scale=self.contrast_scale,
                    offset=self.contrast_offset
                )

            # STEP 4: Single download (Jetson unified mem — cache flush not real copy)
            final_frame = self.vpi_contrasted.cpu()

            # STEP 5: FPS + pixel stats at 1Hz
            self._fps_count += 1
            now = time.time()
            if now - self._fps_last >= 1.0:
                # self.get_logger().info(
                #     f"📷 fps={self._fps_count} | "
                #     f"pixel min={int(final_frame.min())} "
                #     f"max={int(final_frame.max())} "
                #     f"mean={final_frame.mean():.1f} "
                #     f"(target: min=0 max=255 mean=80-120)"
                # )
                self._fps_count = 0
                self._fps_last  = now

            # STEP 6: Publish mono8
            # ArUco calls toCvCopy(msg, MONO8) — already mono8, zero conversion cost
            now_msg = self.get_clock().now().to_msg()
            img_msg = self.bridge.cv2_to_imgmsg(final_frame, "mono8")
            img_msg.header.stamp = now_msg
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            info_msg = CameraInfo()
            info_msg.header = img_msg.header
            info_msg.height = self.DIM[1]
            info_msg.width  = self.DIM[0]
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0] * 5
            info_msg.k = self.K.flatten().tolist()
            info_msg.r = np.eye(3).flatten().tolist()
            P = np.zeros((3, 4))
            P[:3, :3] = self.K
            info_msg.p = P.flatten().tolist()
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
