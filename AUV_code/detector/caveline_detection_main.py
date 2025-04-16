import os
import cv2
import time
import numpy as np
from datetime import datetime
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Point

from ament_index_python.packages import get_package_share_directory

# Use relative imports if all modules are in the same package directory.
from .perception_util import PrepareEngine, InferenceOnFrame
from .comm_util import initialize_camera, reinitialize_camera

class MainScriptNode(Node):
    def __init__(self):
        super().__init__('main_script_node')

        # Package directory
        package_share = get_package_share_directory('detector')

        # Read config params
        config_file = os.path.join(package_share, 'config', 'config.yaml')
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        # Create publishers for data topics
        self.qr_pub = self.create_publisher(Int32, 'qr_info', 10)
        self.pose_pub = self.create_publisher(String, 'pose', 10)
        self.waypoints_pub = self.create_publisher(Point, 'waypoints', 10)
        self.closest_point_pub = self.create_publisher(Point, 'closest_point', 10)
        self.angle_pub = self.create_publisher(Float32, 'angle', 10)

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

        # Variables for storing previous points for smoothing
        self.prev_point = np.array([0.0, 0.0])
        self.prev_closest_point = np.array([0.0, 0.0])
        self.DIRECTION_HISTORY_SIZE = 3

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
                qr_value = 100
            else:
                try:
                    data, points, _ = self.qr_detector.detectAndDecode(frame_front)
                    if data:
                        try:
                            qr_value = int(data)
                        except ValueError:
                            self.get_logger().warn("Non-numeric QR code: {}. Using 100.".format(data))
                            qr_value = 100
                    else:
                        qr_value = 100
                except cv2.error as e:
                    self.get_logger().error("OpenCV QR detection failed: {}".format(e))
                    qr_value = 100

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

                # Default values
                direction = "lost"
                next_point = np.array([0.0, 0.0])
                closest_point = np.array([0.0, 0.0])
                next_heading_deg = 0.0
                detected_caveline = False

                if lines:
                    countours = len(lines)
                    self.get_logger().info("{} lines detected".format(countours))
                    if countours == 1:
                        x1, y1, x2, y2 = lines[0]
                        next_heading_deg = np.rad2deg(np.arctan2(y2 - y1, x2 - x1))
                        next_point[0] = ((x1 + x2) / 2.0) - screen_center_x
                        next_point[1] = ((y1 + y2) / 2.0) - screen_center_y
                        if np.sqrt(x1**2 + y1**2) < np.sqrt(x2**2 + y2**2):
                            closest_point = np.array([x1, y1])
                        else:
                            closest_point = np.array([x2, y2])
                    elif countours > 1:
                        caveline_points = np.zeros((countours, 2))
                        waypoints = np.zeros((countours, 2))
                        distances = np.zeros((countours, 1))
                        for i, line in enumerate(lines):
                            x1, y1, x2, y2 = line
                            caveline_points[i, 0] = (x1 + x2) / 2.0
                            caveline_points[i, 1] =  (y1 + y2) / 2.0
                            waypoints[i, 0] = caveline_points[i, 0] - screen_center_x
                            waypoints[i, 1] = caveline_points[i, 1] - screen_center_y
                            distances[i] = np.sqrt(waypoints[i, 0]**2 + waypoints[i, 1]**2)
                        closest_point_index = np.argmin(distances)
                        closest_point = waypoints[closest_point_index]
                        if closest_point_index + 1 < countours:
                            next_point_index = closest_point_index + 1
                        else:
                            next_point_index = countours - 1
                        next_point_index = countours - 1  # using the last line as in your script
                        next_point = waypoints[next_point_index, :]
                        next_heading_deg = np.rad2deg(np.arctan2(next_point[1], next_point[0]))
                    
                    if 25 < abs(next_heading_deg) < 155:
                        direction = "turn"
                    else:
                        direction = "straight"
                    self.prev_point = next_point
                    self.prev_closest_point = closest_point
                    detected_caveline = True
                else:
                    self.get_logger().info("No lines detected")
                    next_point = self.prev_point
                    closest_point = self.prev_closest_point
                    # next_point = np.array([0.0, 0.0])
                    # next_heading_deg = 0.0
                    # direction = "lost"
                    # detected_caveline = False

                if detected_caveline:
                    img_closest_x = int(closest_point[0] + screen_center_x)
                    img_closest_y = int(closest_point[1] + screen_center_y)

                    img_next_x = int(next_point[0] + screen_center_x)
                    img_next_y = int(next_point[1] + screen_center_y)

                    # Draw circles (red for next_point, cyan for closest_point)
                    cv2.circle(line_overlayed_map, (img_closest_x, img_closest_y), 5, (0, 255, 125), -1)  
                    cv2.circle(line_overlayed_map, (img_next_x, img_next_y), 5, (0, 0, 255), -1)         # red
                    cv2.circle(line_overlayed_map, (screen_center_x, screen_center_y), 5, (255, 255, 0), -1) # cyan

                    # Draw a green arrowed line between them                    
                    cv2.arrowedLine(line_overlayed_map, (screen_center_x, screen_center_y),
                                    (img_next_x, img_next_y),
                                    (255, 0, 0), 2, tipLength=0.1)
                
                # Save processed frame even if no processing is performed i.e. detected_caveline = False
                # Save processed and raw downward camera frames
                cv2.imwrite(f"{self.PROCESSED_FRAMES_DIR}/frame_{time.time():.6f}.jpg", line_overlayed_map)
                cv2.imwrite(f"{self.RAW_FRAMES_DIR}/frame_{time.time():.6f}.jpg", frame_down)

                # Optionally, show the processed frame
                cv2.imshow("Downward Camera - Detected Lines", line_overlayed_map)
                cv2.waitKey(1)

                # --- Publish processed data ---
                # Publish pose (direction)
                pose_msg = String()
                pose_msg.data = direction
                self.pose_pub.publish(pose_msg)

                # Publish next_point as waypoints (using Point, with z = 0)
                waypoints_msg = Point()
                waypoints_msg.x = float(next_point[0])
                waypoints_msg.y = float(next_point[1])
                waypoints_msg.z = 0.0
                self.waypoints_pub.publish(waypoints_msg)

                # Publish closest_point (using Point, with z = 0)
                closest_point_msg = Point()
                closest_point_msg.x = float(closest_point[0])
                closest_point_msg.y = float(closest_point[1])
                closest_point_msg.z = 0.0
                self.closest_point_pub.publish(closest_point_msg)

                # Publish angle (next_heading_deg)
                angle_msg = Float32()
                angle_msg.data = float(next_heading_deg)
                self.angle_pub.publish(angle_msg)
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
    node = MainScriptNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
