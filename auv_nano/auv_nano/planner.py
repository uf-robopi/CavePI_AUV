# =======================================================
# Author : Alakrit Gupta, Adnan Abdullah
# Email: gupta.alankrit@ufl.edu, adnanabdullah@ufl.edu
# =======================================================

import os
import cv2
import numpy as np
import rclpy
import yaml
import time
import socket
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from .perception_util import PrepareEngine, InferenceOnFrame
from .comm_util import initialize_camera, reinitialize_camera, send_udp_data


class Planner(Node):
    """
    This ROS2 node processes detected lines from the down camera, plans the next waypoint and update robot's state information.
    The node subscribes to the detected lines and map image topics.
    The node publishes the new state, next waypoint, closest point, and heading (yaw) angle.
    """
    def __init__(self):
        super().__init__('planner')

        # Package directory
        package_share = get_package_share_directory('auv_nano')

        # Read config params
        config_file = os.path.join(package_share, 'config', 'config.yaml')
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.UDP_IP = config['UDP_IP']
        self.UDP_PORT = config['UDP_PORT']
        self.PUBLISH_RATE = config['PUBLISH_RATE']  # 5 Hz
        self.SLEEP_TIME = 1.0 / self.PUBLISH_RATE

        # Subscribed message containers
        self.detected_lines = None
        self.map_image = None

        # Subscribe to map and detected lines
        self.lines_sub = self.create_subscription(Float32MultiArray, '/detected_lines', self.lines_callback, 10)
        self.map_sub = self.create_subscription(Image, '/map', self.map_callback, 10)


        # Create publishers for data topics
        self.state_pub = self.create_publisher(String, '/state', 10)
        self.waypoints_pub = self.create_publisher(Point, '/waypoints', 10)
        self.closest_point_pub = self.create_publisher(Point, '/closest_point', 10)
        self.angle_pub = self.create_publisher(Float32, '/angle', 10)
        self.processed_image_pub = self.create_publisher(Image, '/processed_image', 10)

        # CVBridge for converting ROS Image to OpenCV format
        self.bridge = CvBridge()

        # Store previous points for smoothing
        self.prev_point = np.array([0.0, 0.0])
        self.prev_closest_point = np.array([0.0, 0.0])
        self.DIRECTION_HISTORY_SIZE = 3

        # Create UDP socket (alternate comm channel for ROS2)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


        # Set processing rate (10 Hz) using a timer
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def lines_callback(self, msg):
        """
        Store the coordinates (pixel locations) of detected lines.
        """
        try:
            self.detected_lines = np.array(msg.data).reshape(-1, 4)
        except Exception as e:
            self.detected_lines = None
    
    def map_callback(self, msg):
        """
        Store the segmentation map and 
        convert it from ROS Image to OpenCV format
        """
        try:
            self.map_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert map image: {e}")
            self.map_image = None


    def timer_callback(self):
        """
        Callback  function to keep the node alive.
        """

        # Check if segmentation map is published
        if self.map_image is None:
            self.get_logger().info("Waiting for map image...")
            return
        
        try:
            # --- Process the detected lines to calculate next waypoint ---
            lines = self.detected_lines
            line_overlayed_map = self.map_image.copy()
            frame_height, frame_width = line_overlayed_map.shape[:2]
            screen_center_x = frame_width // 2
            screen_center_y = frame_height // 2

            # Initialize variables
            direction = "lost"
            next_point = np.array([0.0, 0.0])
            closest_point = np.array([0.0, 0.0])
            next_heading_deg = 0.0
            detected_caveline = False

            if lines:
                countours = len(lines)
                self.get_logger().info("{} lines detected".format(countours))

                # If only one line is detected, use its coordinates for waypoint
                if countours == 1:
                    x1, y1, x2, y2 = lines[0]
                    next_heading_deg = np.rad2deg(np.arctan2(y2 - y1, x2 - x1))
                    next_point[0] = ((x1 + x2) / 2.0) - screen_center_x
                    next_point[1] = ((y1 + y2) / 2.0) - screen_center_y
                    closest_point = np.array([x1, y1]) if np.hypot(x1, y1) < np.hypot(x2, y2) else np.array([x2, y2])
                
                # If multiple lines are detected, get the farthest line
                else:
                    caveline_points = np.zeros((countours, 2))
                    waypoints = np.zeros((countours, 2))
                    distances = np.zeros((countours, 1))
                    for i, line in enumerate(lines):
                        x1, y1, x2, y2 = line
                        caveline_points[i] = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]
                        waypoints[i] = caveline_points[i] - [screen_center_x, screen_center_y]
                        distances[i] = np.linalg.norm(waypoints[i])
                    closest_point_index = np.argmin(distances)
                    closest_point = waypoints[closest_point_index]
                    next_point_index = countours - 1
                    next_point = waypoints[next_point_index]
                    next_heading_deg = np.rad2deg(np.arctan2(next_point[1], next_point[0]))

                direction = "turn" if 25 < abs(next_heading_deg) < 155 else "straight"
                self.prev_point = next_point
                self.prev_closest_point = closest_point
                detected_caveline = True
            else:
                # No lines detected, reuse previous values
                self.get_logger().info("No lines detected, reusing previous values.")
                next_point = self.prev_point
                closest_point = self.prev_closest_point

            if detected_caveline:
                # Calculate the closest point to the robot and the farthest contour center
                img_closest_x = int(closest_point[0] + screen_center_x)
                img_closest_y = int(closest_point[1] + screen_center_y)
                img_next_x = int(next_point[0] + screen_center_x)
                img_next_y = int(next_point[1] + screen_center_y)

                # Draw the points and an arrow toward the waypoint on the overlayed map
                cv2.circle(line_overlayed_map, (img_closest_x, img_closest_y), 5, (0, 255, 125), -1)
                cv2.circle(line_overlayed_map, (img_next_x, img_next_y), 5, (0, 0, 255), -1)
                cv2.circle(line_overlayed_map, (screen_center_x, screen_center_y), 5, (255, 255, 0), -1)
                cv2.arrowedLine(line_overlayed_map, (screen_center_x, screen_center_y),
                                (img_next_x, img_next_y), (255, 0, 0), 2, tipLength=0.1)

            ## Optional: show the processed image with planned waypoints
            # cv2.imshow("Planner Processed View", line_overlayed_map)
            # cv2.waitKey(1)
            
            # Publish the line overlayed map
            ros_line_overlayed_map= self.bridge.cv2_to_imgmsg(line_overlayed_map, encoding="bgr8")
            self.processed_image_pub.publish(ros_line_overlayed_map)

            # Publish state
            state_msg = String()
            state_msg.data = direction
            self.state_pub.publish(state_msg)

            # Publish next waypoint
            waypoint_msg = Point()
            waypoint_msg.x = float(next_point[0])
            waypoint_msg.y = float(next_point[1])
            waypoint_msg.z = 0.0
            self.waypoints_pub.publish(waypoint_msg)

            # Publish heading
            angle_msg = Float32()
            angle_msg.data = float(next_heading_deg)
            self.angle_pub.publish(angle_msg)

            # Send control commands to RPi over UDP in case ROS2 connection is lost
            print("[INFO] Sending control commands")
            send_udp_data(self.sock, "pose", {"direction": direction}, self.UDP_IP, self.UDP_PORT)
            send_udp_data(self.sock, "waypoints", {"x": float(next_point[0]), "y": float(next_point[1])}, self.UDP_IP, self.UDP_PORT)
            send_udp_data(self.sock, "angle", {"angle": float(next_heading_deg)}, self.UDP_IP, self.UDP_PORT)

            # Sleep to maintain ~5Hz
            time.sleep(self.SLEEP_TIME)

        except Exception as e:
            self.get_logger().error("Exception in timer callback: {}".format(e))

    def destroy_node(self):
        # Clean up:destroy OpenCV windows
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """
    Main function to run the Planner node.
    """
    # Initialize the ROS2 node
    rclpy.init(args=args)
    planner = Planner()
    try:
        # Spin the node to keep it alive and processing
        rclpy.spin(planner)
    except KeyboardInterrupt:
        # Handle keyboard interrupt gracefully
        planner.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        # Clean up and shutdown
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
