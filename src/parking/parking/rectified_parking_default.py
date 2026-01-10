#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CUDARectifiedNode(Node):
    def __init__(self):
        super().__init__('rectified_camera_node')
        
        self.frame_id = "camera_optical_frame"
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0/21.0, self.timer_callback)

        # --- 1. RESOLUTION SETUP ---
        # Fixed to match IMX219 native sensor output (3280x2464)
        self.DIM = (3280, 2464)
        
        # Original Intrinsics (K)
        self.K = np.array([
            [1813.000558,    0.      ,  1634.163912],
            [   0.      , 1826.621610,  1258.598589],
            [   0.      ,    0.      ,     1.      ]
        ])
        self.D = np.array([-0.022401, -0.022708, 0.014203, -0.009313])

        # --- 2. GPU MAP PREPARATION ---
        self.get_logger().info("Generating CUDA Maps...")
        
        # Calc New Camera Matrix
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, self.DIM, np.eye(3), balance=0, new_size=self.DIM, fov_scale=1.0
        )
        
        # Generate Maps on CPU
        map1_cpu, map2_cpu = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.new_K, self.DIM, cv2.CV_32FC1
        )

        # Upload Maps to GPU
        self.map1_gpu = cv2.cuda_GpuMat()
        self.map2_gpu = cv2.cuda_GpuMat()
        self.map1_gpu.upload(map1_cpu)
        self.map2_gpu.upload(map2_cpu)

        # --- CRITICAL FIX: Pre-allocate GPU Buffers ---
        self.gpu_frame = cv2.cuda_GpuMat()
        # Allocate output buffer with explicit size (Height, Width, Type)
        self.gpu_output = cv2.cuda_GpuMat(self.DIM[1], self.DIM[0], cv2.CV_8UC3)

        # --- 3. START CAMERA ---
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream")

    def gstreamer_pipeline(self, sensor_id=0, capture_width=3280, capture_height=2464, framerate=21, flip_method=0):
        # Updated to 3280 width
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=1"
            % (sensor_id, capture_width, capture_height, framerate, flip_method, capture_width, capture_height)
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            # Check if camera disconnected
            self.get_logger().warn("Failed to read frame")
            return

        # 1. Upload to GPU
        self.gpu_frame.upload(frame)

        # 2. Remap (Result goes into self.gpu_output)
        cv2.cuda.remap(self.gpu_frame, self.map1_gpu, self.map2_gpu, cv2.INTER_LINEAR, self.gpu_output)

        # 3. Download
        rect_frame = self.gpu_output.download()
        
        # --- Safety Check ---
        if rect_frame is None:
            self.get_logger().error("GPU Download returned None!")
            return

        # 4. Publish
        now = self.get_clock().now().to_msg()
        
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rect_frame, "bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # Publish Camera Info
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = self.frame_id
            info_msg.header = img_msg.header
            info_msg.height = self.DIM[1]
            info_msg.width = self.DIM[0]
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0]*5
            info_msg.k = self.new_K.flatten().tolist()
            info_msg.r = np.eye(3).flatten().tolist()
            P_matrix = np.zeros((3, 4))
            P_matrix[:3, :3] = self.new_K
            info_msg.p = P_matrix.flatten().tolist()
            
            self.info_pub.publish(info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CUDARectifiedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()







