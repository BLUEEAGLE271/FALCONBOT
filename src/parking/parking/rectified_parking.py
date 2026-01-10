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
        self.scale_factor = 0.25
        self.DIM = (820, 616)
        
        # Input K (Standard 3x3)
        self.K = np.array([
            [1813.000558 * self.scale_factor,    0.      ,  1634.163912 * self.scale_factor],
            [   0.      , 1826.621610 * self.scale_factor,  1258.598589 * self.scale_factor],
            [   0.      ,    0.      ,     1.      ]
        ])
        
        self.D = [-0.022401, -0.022708, 0.014203, -0.009313]

        self.get_logger().info("Initializing VPI Hardware Maps...")

        # --- 2. GENERATE WARP MAP ---
        self.grid = vpi.WarpGrid(self.DIM)
        self.X = np.eye(3, 4)
        K_2x3 = self.K[0:2, :]  
        
        # Generate the Map on VIC
        self.vpi_warp_map = vpi.WarpMap.fisheye_correction(
            self.grid,
            K=K_2x3,
            X=self.X,
            mapping=vpi.FisheyeMapping.EQUIDISTANT,
            coeffs=self.D
        )

        # --- 3. ROS SETUP ---
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Start Camera
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
             self.get_logger().error("Could not open camera!")

        self.timer = self.create_timer(1.0/21.0, self.timer_callback)

    def gstreamer_pipeline(self, sensor_id=0):
        # Capture 8MP -> Hardware Scale to 0.5MP
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=3280, height=2464, framerate=21/1 ! "
            f"nvvidconv ! "
            f"video/x-raw, width={self.DIM[0]}, height={self.DIM[1]}, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! appsink drop=1"
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        try:
            # 1. Wrap OpenCV frame (BGR) into VPI
            

            with vpi.Backend.CUDA:
                #rectified_bgr = vpi.asimage(frame).convert(vpi.Format.NV12_ER).remap(self.vpi_warp_map, interp=vpi.Interp.LINEAR).convert(vpi.Format.RGB8)
                rectified_bgr = vpi.asimage(frame).convert(vpi.Format.NV12_ER).remap(self.vpi_warp_map, interp=vpi.Interp.LINEAR).convert(vpi.Format.RGB8)
            # 5. Download to CPU
            rect_frame = rectified_bgr.cpu()

            # 6. Publish
            now = self.get_clock().now().to_msg()
            img_msg = self.bridge.cv2_to_imgmsg(rect_frame, "bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
            
            # Info
            info_msg = CameraInfo()
            info_msg.header = img_msg.header
            info_msg.height = self.DIM[1]
            info_msg.width = self.DIM[0]
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0]*5 
            info_msg.k = self.K.flatten().tolist()
            info_msg.r = np.eye(3).flatten().tolist()
            P = np.zeros((3, 4))
            P[:3, :3] = self.K
            info_msg.p = P.flatten().tolist()
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f"VPI Error: {e}")

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