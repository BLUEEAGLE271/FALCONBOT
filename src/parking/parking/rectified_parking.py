#!/usr/bin/env python3
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
        self.scale_factor = 0.5
        self.DIM = (1640, 1232)
        
        self.K = np.array([
            [1813.000558 * self.scale_factor,    0.      ,  1634.163912 * self.scale_factor],
            [   0.      , 1826.621610 * self.scale_factor,  1258.598589 * self.scale_factor],
            [   0.      ,    0.      ,     1.      ]
        ])
        
        self.D = [-0.022401, -0.022708, 0.014203, -0.009313]

        # --- 2. CONTRAST TUNING ---
        # output = input * scale + offset
        # scale=2.0  → doubles contrast range (blacks get blacker, whites whiter)
        # offset=-80 → darkens midtones so marker blacks become truly black
        # Tune if needed:
        #   Dark environment / dim marker  → scale=2.5, offset=-100
        #   Bright environment             → scale=1.8, offset=-60
        self.contrast_scale  = 3.0
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

        # Pipeline VPI images — all stay on GPU until final .cpu() call
        self.vpi_rectified  = vpi.Image(self.DIM, vpi.Format.RGB8)  # remap output
        self.vpi_gray       = vpi.Image(self.DIM, vpi.Format.U8)    # grayscale
        self.vpi_contrasted = vpi.Image(self.DIM, vpi.Format.U8)    # final output

        # --- 4. ROS SETUP ---
        # Published as mono8 — ArUco converts to MONO8 anyway, this eliminates
        # the wasted BGR→MONO8 conversion and cuts image transport size by 3x
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 20)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info', 20)

        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)
        self.get_logger().info(
            f"Camera ready: {self.DIM[0]}x{self.DIM[1]} | "
            f"VPI pipeline: remap → grayscale → contrast (all CUDA). "
            f"Publishing mono8 directly to ArUco."
        )

    def gstreamer_pipeline(self, sensor_id=0):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=3280, height=2464, framerate=15/1 ! "
            f"nvvidconv ! "
            f"video/x-raw, width={self.DIM[0]}, height={self.DIM[1]}, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! appsink drop=1"
        )

    def timer_callback(self):
        # Drain stale GStreamer buffer frames before grabbing fresh one
        
        ret, frame = self.cap.read()
        if not ret:
            return

        try:
            with vpi.Backend.CUDA:
                # STEP 1: Fisheye rectification (BGR → BGR, GPU)
                vpi.asimage(frame).remap(
                    self.vpi_warp_map,
                    interp=vpi.Interp.LINEAR,
                    out=self.vpi_rectified
                )

                # STEP 2: BGR → Grayscale (GPU)
                # ArUco only uses grayscale — no point carrying colour any further
                self.vpi_rectified.convert(self.vpi_gray)

                # STEP 3: Contrast boost (GPU)
                # output = input * scale + offset
                # Pushes blacks to 0 and whites to 255 — marker edges become razor sharp
                self.vpi_gray.convert(
                    self.vpi_contrasted,
                    scale=self.contrast_scale,
                    offset=self.contrast_offset
                )

            # STEP 4: Single download — Jetson unified mem cache flush, not a real copy
            final_frame = self.vpi_contrasted.cpu()

            # STEP 5: Publish as mono8
            # ArUco node does cv_bridge::toCvCopy(msg, MONO8) — this is already MONO8,
            # so zero conversion happens on the ArUco side
            now = self.get_clock().now().to_msg()
            img_msg = self.bridge.cv2_to_imgmsg(final_frame, "mono8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # Camera Info — K matrix is still valid, image is just grayscale now
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
