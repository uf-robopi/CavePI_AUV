# =======================================================
# Author : Alakrit Gupta, Adnan Abdullah
# Email: gupta.alankrit@ufl.edu, adnanabdullah@ufl.edu
# =======================================================

import os
import cv2
import time
import numpy as np
from datetime import datetime
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
from .perception_util import PrepareEngine, InferenceOnFrame
from .comm_util import initialize_camera, reinitialize_camera

class Detector(Node):
    def __init__(self):
        super().__init__('detector')

        # Package directory
        package_share = get_package_share_directory('detector')

        # Read config params
        config_file = os.path.join(package_share, 'config', 'config.yaml')
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        # Create publishers for data topics
        self.qr_pub = self.create_publisher(Int32, 'qr_info', 10)
        self.map_pub = self.create_publisher(Image, 'map', 10)
        self.lines_pub = self.create_publisher(Float32MultiArray, 'detected_lines', 10)

        self.bridge = CvBridge()


        # Directories for saving frames (relative to the current working directory)
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.join("data", f"session_{run_id}")
        self.FRONT_CAM_DIR = os.path.join(save_dir, "front_cam_frames_auv")
        self.PROCESSED_FRAMES_DIR = os.path.join(save_dir, "down_processed_frames_auv")
        self.RAW_FRAMES_DIR = os.path.join(save_dir, "down_raw_frames_auv")
        os.makedirs(self.FRONT_CAM_DIR, exist_ok=True)
        os.makedirs(self.PROCESSED_FRAMES_DIR, exist_ok=True)
        os.makedirs(self.RAW_FRAMES_DIR, exist_ok=True)

        # Read engine file and prepare bindings
        engine_filename = 'deeplabmbnet_3.engine'
        engine_file = os.path.join(package_share, 'weights', engine_filename)
        self.get_logger().info("Loading TensorRT engine from: " + engine_file)
        (self.engine, self.bindings, self.host_inputs, self.cuda_inputs, 
         self.host_outputs, self.cuda_outputs, self.stream, self.trt_context) = PrepareEngine(engine_file)

        # Camera indices as defined in config file
        self.FRONT_CAMERA_INDEX = config['FRONT_CAMERA_INDEX']
        self.DOWN_CAMERA_INDEX = config['DOWN_CAMERA_INDEX']
        self.front_camera = initialize_camera(self.FRONT_CAMERA_INDEX, "front_camera")
        self.downward_camera = initialize_camera(self.DOWN_CAMERA_INDEX, "downward_camera")

        # Initialize QR code detector
        self.qr_detector = cv2.QRCodeDetector()


        # Set processing rate (10 Hz) using a timer
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self):
        try:
            # --- Process front camera for QR detection ---
            if not self.front_camera.isOpened():
                self.get_logger().error("Front camera is not opened; reinitializing...")
                self.front_camera = reinitialize_camera(self.FRONT_CAMERA_INDEX, "front_camera")
            
            ret_front, frame_front = self.front_camera.read()
            if not ret_front or frame_front is None:
                self.get_logger().warn("Failed to read frame from front camera.")
                qr_value = -1
            else:
                try:
                    data, points, _ = self.qr_detector.detectAndDecode(frame_front)
                    if data:
                        try:
                            qr_value = int(data)
                        except ValueError:
                            self.get_logger().warn("Non-numeric QR code: {}. Using -1.".format(data))
                            qr_value = -1
                    else:
                        qr_value = -1
                except cv2.error as e:
                    self.get_logger().error("OpenCV QR detection failed: {}".format(e))
                    qr_value = -1

                if not self.front_camera.isOpened():
                    self.get_logger().error("Front camera lost connection; reinitializing...")
                    self.front_camera = reinitialize_camera(self.FRONT_CAMERA_INDEX, "front_camera")

            # Save the front camera frame (for debugging/recording)
            cv2.imwrite(f"{self.FRONT_CAM_DIR}/frame_{time.time():.6f}.jpg", frame_front)

            # Publish the QR code info
            qr_msg = Int32()
            qr_msg.data = qr_value
            self.qr_pub.publish(qr_msg)

            # --- Process downward camera for line detection ---
            if not self.downward_camera.isOpened():
                self.get_logger().error("Downward camera is not opened; reinitializing...")
                self.downward_camera = reinitialize_camera(self.DOWN_CAMERA_INDEX, "downward_camera")
            
            ret_down, frame_down = self.downward_camera.read()
            if not ret_down or frame_down is None:
                self.get_logger().warn("Failed to read frame from downward camera.")
            else:
                frame_height, frame_width = frame_down.shape[:2]
                screen_center_x = frame_width // 2
                screen_center_y = frame_height // 2

                self.get_logger().info("Inferencing on downward frame...")
                lines, line_overlayed_map = InferenceOnFrame(
                    self.bindings, self.host_inputs, self.cuda_inputs, 
                    self.host_outputs, self.cuda_outputs, self.stream, 
                    self.trt_context, frame_down)
                
                # Publish the processed frame
                map_msg = self.bridge.cv2_to_imgmsg(line_overlayed_map, encoding='bgr8')
                self.map_pub.publish(map_msg)

                # Publish detected lines
                flat_lines = [float(coord) for line in lines for coord in line]  # flatten the list of lines
                lines_msg = Float32MultiArray()
                lines_msg.data = flat_lines
                self.lines_pub.publish(lines_msg)             
                
                # Save processed and raw downward camera frames
                cv2.imwrite(f"{self.PROCESSED_FRAMES_DIR}/frame_{time.time():.6f}.jpg", line_overlayed_map)
                cv2.imwrite(f"{self.RAW_FRAMES_DIR}/frame_{time.time():.6f}.jpg", frame_down)

                # Optionally, show the processed frame
                # cv2.imshow("Downward Camera - Detected Lines", line_overlayed_map)
                # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error("Exception in timer callback: {}".format(e))

    def destroy_node(self):
        # Clean up: release cameras and destroy OpenCV windows
        if self.front_camera and self.front_camera.isOpened():
            self.front_camera.release()
        if self.downward_camera and self.downward_camera.isOpened():
            self.downward_camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
