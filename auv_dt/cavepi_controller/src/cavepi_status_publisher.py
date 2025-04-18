#!/usr/bin/env python3

# =====================================
# Author : Alakrit Gupta
# Email: gupta.alankrit@ufl.edu
# =====================================

import rospy
from std_msgs.msg import String, Float32MultiArray, Float32, Int32, Header
from cavepi_interfaces.msg import CavepiStatus, CavepiInput

class CavepiStatusNode:
    def __init__(self):
        rospy.init_node('caveline_status_publisher')

        # Subscribers
        self.state_sub = rospy.Subscriber(
            name="/state_data", data_class=String, callback=self.state_callback)
        self.num_sub = rospy.Subscriber(
            name="/number", data_class=Int32, callback=self.number_callback)
        self.angle_sub = rospy.Subscriber(
            name="/angle", data_class=Float32, callback=self.angle_callback)
        self.control_sub = rospy.Subscriber(
            name="/cavepi/user_input", data_class=CavepiInput, callback=self.control_callback)
        self.waypoint_sub = rospy.Subscriber(
            name="/waypoints_data", data_class=Float32MultiArray, callback=self.waypoint_callback)

        # Publisher
        self.status_pub = rospy.Publisher(
            name="/cavepi_status", data_class=CavepiStatus, queue_size=10)

        self.state = "lost"
        self.number_of_contours = 0
        self.angle = 0.0
        self.surge = 0.0
        self.yaw = 0.0


    def state_callback(self, msg):
        self.state = msg.data

        
    def number_callback(self, msg):
        self.number_of_contours = msg.data


    def angle_callback(self, msg):
        self.angle = msg.data


    def control_callback(self, msg):
        self.surge = msg.surge
        self.yaw = msg.yaw


    def waypoint_callback(self, msg):
        offset = msg.data

        out = CavepiStatus()
        out.header = Header()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "cavepi_status"
        out.state = self.state
        out.detected_contours = self.number_of_contours
        out.angle = self.angle

        out.control_cmd.layout.dim = []  # No dimensions for a simple 1D array
        out.control_cmd.layout.data_offset = 0
        out.control_cmd.data = [self.surge, self.yaw]  # Example 1D array for control commands

        out.waypoint.layout.dim = []  # No dimensions for a simple 1D array
        out.waypoint.layout.data_offset = 0
        out.waypoint.data = offset

        self.status_pub.publish(out)
            

if __name__ == '__main__':
    try:
        node = CavepiStatusNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
