#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray, Float32, Int32
import cv2
from cv_bridge import CvBridge
import numpy as np


class CavelineDetectorNode:
    def __init__(self):
        rospy.init_node('caveline_detector')

        # Subscriber
        self.down_cam_sub = rospy.Subscriber(
            name="/downward_camera/image_raw", data_class=Image, callback=self.down_cam_callback)

        # Publishers
        self.state_pub = rospy.Publisher(
            name="/state_data", data_class=String, queue_size=10)
        self.waypoints_pub = rospy.Publisher(
            name="/waypoints_data", data_class=Float32MultiArray, queue_size=10)
        self.angle_pub = rospy.Publisher(
            name="/angle", data_class=Float32, queue_size=10)
        self.contours = rospy.Publisher("/number", Int32, queue_size=1)
        
        self.bridge = CvBridge()
        self.prev_offset_x = 0
        self.prev_offset_y = 0  
        self.direction_history = []
        self.DIRECTION_HISTORY_SIZE = 3


    def down_cam_callback(self, msg):
        try:
            frame_down = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return
        

        frame_height, frame_width = frame_down.shape[:2]
        screen_center_x = frame_width // 2
        screen_center_y = frame_height // 2
        lines = []

        # Convert to grayscale, blur, edge detection
        gray = cv2.cvtColor(frame_down, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.logwarn("No contours detected in the current frame.")
            return

        # Sort the contours from right to left (robot's direction)
        try:
            sorted_contours, _ = self.sort_contours(contours, method='right-to-left')
        except ValueError as e:
            rospy.logerr(f"Error sorting contours: {e}")
            return

        # Fit a line to the contours (using polyfit to fit a line that goes through the contour)
        for contour in sorted_contours:
            if len(contour) >= 5:  # Fit a line only if the contour has enough points
                # Extract x and y coordinates of the contour points
                points = contour.reshape(-1, 2)  # Convert to 2D array of points (x, y)

                # Use np.polyfit to fit a line (degree 1 = linear fit)
                # The polyfit function returns the slope and intercept of the line
                [slope, intercept] = np.polyfit(points[:, 0], points[:, 1], 1)

                # Compute the line's start and end points for visualization
                # We can extend the line by using the image's x-range
                x1, x2 = min(points[:, 0]), max(points[:, 0])
                y1, y2 = slope * x1 + intercept, slope * x2 + intercept

                # Draw the fitted line on the original image (in color for visibility)
                cv2.line(frame_down, (int(x1), int(y1)), (int(x2), int(y2)), (100, 255, 100), 5)
                # print(f"line slope: {slope}, intercept: {intercept}")
                lines.append([x1, y1, x2, y2])

        direction = "lost"
        next_point = np.array([0.0, 0.0])
        next_heading = 0.0
        

        if lines is None:
            # lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=20)
            return
        
        count = len(lines)
        if count == 1:
            x1, y1, x2, y2 = lines[0]
            next_point[0] = x1
            next_point[1] = y1

        elif count > 1:
            first_end_point_x = []
            first_end_point_y = []
            second_end_point_x = []
            second_end_point_y = []

            for line in lines:
                x1, y1, x2, y2 = line
                cv2.line(frame_down, (int(x1), int(y1)), (int(x2), int(y2)), color=(0, 255, 0), thickness=2)  # Green color

                first_end_point_x.append(x1)
                first_end_point_y.append(y1)
                second_end_point_x.append(x2)
                second_end_point_y.append(y2)

            next_point[0] = sum(first_end_point_x)/len(first_end_point_x)
            next_point[1] = sum(first_end_point_y)/len(first_end_point_y)
            
        next_heading = np.rad2deg(np.arctan2(next_point[1] - screen_center_y, next_point[0] - screen_center_x))
        if next_heading >= 0:
            next_heading = next_heading - 180
        elif next_heading < 0:
            next_heading = next_heading + 180

        if abs(next_heading) <= 20 or abs(next_heading) >= 160:
            direction = "straight"
        else:
            direction = "turn"
            


        state = String()
        state.data = direction
        self.state_pub.publish(state)

        waypoint = Float32MultiArray()
        waypoint.data = next_point
        self.waypoints_pub.publish(waypoint)

        angle = Float32()
        angle.data = next_heading
        self.angle_pub.publish(angle)

        out = Int32()
        out.data = count
        self.contours.publish(out)
        
        # Draw the origin point (0, 0) as a red circle
        cv2.circle(frame_down, (0, 0), radius=5, color=(0, 0, 255), thickness=-1)  # Red color

        # Draw the x-axis (horizontal line through the center)
        cv2.line(frame_down, (0, 0), (frame_width, 0), color=(255, 0, 0), thickness=2)  # Blue color

        # Draw the y-axis (vertical line through the center)
        cv2.line(frame_down, (0, 0), (0, frame_height), color=(0, 255, 0), thickness=2)  # Green color

        cv2.circle(frame_down, (screen_center_x, screen_center_y), radius=5, color=(0, 255, 0), thickness=-1)
        cv2.circle(frame_down, (int(next_point[0]), int(next_point[1])), radius=5, color=(255, 255, 255), thickness=-1)
        cv2.arrowedLine(frame_down, (screen_center_x, screen_center_y),
                        (int(next_point[0]), int(next_point[1])),
                        (255, 0, 0), 2, tipLength=0.05)

        cv2.imshow("Downward Camera - Detected Lines", frame_down)
        cv2.waitKey(1)

    
    def sort_contours(self, cnts, method="right-to-left"):
        # initialize the reverse flag and sort index
        reverse = False
        i = 0
        # handle if we need to sort in reverse
        if method == "right-to-left" or method == "bottom-to-top":
            reverse = True
        # handle if we are sorting against the y-coordinate rather than
        # the x-coordinate of the bounding box
        if method == "top-to-bottom" or method == "bottom-to-top":
            i = 1
        # construct the list of bounding boxes and sort them from top to
        # bottom
        boundingBoxes = [cv2.boundingRect(c) for c in cnts]
        (cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
            key=lambda b:b[1][i], reverse=reverse))
        # return the list of sorted contours and bounding boxes
        return (cnts, boundingBoxes)


if __name__ == '__main__':
    try:
        node = CavelineDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass