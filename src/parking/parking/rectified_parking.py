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

        # --- 2. VERIFY CUDA IS AVAILABLE ---
        if not cv2.cuda.getCudaEnabledDeviceCount():
            self.get_logger().error("No CUDA device found! cv2.cuda will not work.")
            raise RuntimeError("CUDA not available")
        self.get_logger().info(f"CUDA device count: {cv2.cuda.getCudaEnabledDeviceCount()}")

        # --- 3. VPI WARP MAP (runs on VIC/CUDA engine) ---
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
        self.output_vpi = vpi.Image(self.DIM, vpi.Format.RGB8)

        # --- 4. CUDA CLAHE (stays on GPU) ---
        # Applied to L channel of LAB — sharpens marker edges without blowing highlights
        # clipLimit: 2.0=subtle, 3.0=good for paper, 4.0+=dark environments
        self.clahe = cv2.cuda.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        # --- 5. CUDA SHARPENING FILTER (stays on GPU) ---
        # Counters blur introduced by VPI bilinear remap
        # Centre value 9 = moderate. Increase to 12 for more aggressive sharpening.
        sharpen_kernel = np.array([
            [ 0, -1,  0],
            [-1,  9, -1],
            [ 0, -1,  0]
        ], dtype=np.float32) / 5.0

        # Create filter for 3-channel BGR image
        self.sharpen_filter = cv2.cuda.createLinearFilter(
            cv2.CV_8UC3,   # input type
            cv2.CV_8UC3,   # output type
            sharpen_kernel
        )

        # Pre-allocate persistent GPU mats to avoid per-frame allocation
        self.gpu_bgr       = cv2.cuda_GpuMat()
        self.gpu_lab       = cv2.cuda_GpuMat()
        self.gpu_sharpened = cv2.cuda_GpuMat()

        # --- 6. ROS SETUP ---
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 20)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info', 20)

        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)
        self.get_logger().info(
            f"Camera ready: {self.DIM[0]}x{self.DIM[1]} | "
            f"VPI remap + CUDA CLAHE + CUDA sharpen. Full GPU pipeline."
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
        # Drain stale GStreamer buffer frames
        for _ in range(3):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            return

        try:
            # ----------------------------------------------------------------
            # STEP 1: VPI fisheye rectification (VIC/CUDA engine)
            # ----------------------------------------------------------------
            with vpi.Backend.CUDA:
                vpi.asimage(frame).remap(
                    self.vpi_warp_map,
                    interp=vpi.Interp.LINEAR,
                    out=self.output_vpi
                )

            # ----------------------------------------------------------------
            # STEP 2: VPI -> GPU Mat
            # On Jetson unified memory, .cpu() is a cache flush not a real copy.
            # We immediately upload to cv2.cuda GpuMat for further processing.
            # ----------------------------------------------------------------
            rect_cpu = self.output_vpi.cpu()
            self.gpu_bgr.upload(rect_cpu)

            # ----------------------------------------------------------------
            # STEP 3: CLAHE on GPU (LAB colour space, L channel only)
            # ----------------------------------------------------------------
            self.gpu_lab = cv2.cuda.cvtColor(self.gpu_bgr, cv2.COLOR_BGR2Lab)

            # Split LAB channels on GPU
            lab_channels = cv2.cuda.split(self.gpu_lab)
            # [0]=L  [1]=A  [2]=B

            # Apply CLAHE to L channel only — stays on GPU
            l_enhanced = self.clahe.apply(lab_channels[0], cv2.cuda.Stream_Null())

            # Merge back on GPU
            cv2.cuda.merge([l_enhanced, lab_channels[1], lab_channels[2]], self.gpu_lab)

            # LAB -> BGR on GPU
            self.gpu_bgr = cv2.cuda.cvtColor(self.gpu_lab, cv2.COLOR_Lab2BGR)

            # ----------------------------------------------------------------
            # STEP 4: Sharpening on GPU
            # ----------------------------------------------------------------
            self.sharpen_filter.apply(self.gpu_bgr, self.gpu_sharpened)

            # ----------------------------------------------------------------
            # STEP 5: Single download — only CPU touch per frame
            # ----------------------------------------------------------------
            final_frame = self.gpu_sharpened.download()

            # ----------------------------------------------------------------
            # STEP 6: Publish
            # ----------------------------------------------------------------
            now = self.get_clock().now().to_msg()
            img_msg = self.bridge.cv2_to_imgmsg(final_frame, "bgr8")
            img_msg.header.stamp = now
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