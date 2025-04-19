#!/usr/bin/env python3

# =====================================
# Author : Alakrit Gupta
# Email: gupta.alankrit@ufl.edu
# =====================================

#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray, Float32
from cavepi_interfaces.msg import CavepiInput
import time
import numpy as np


class CavelineFollowerNode:
    def __init__(self):
        rospy.init_node('caveline_follower')

        # Subscribers
        self.state_sub = rospy.Subscriber(
            name="/state_data", data_class=String, callback=self.state_callback)
        self.waypoints_sub = rospy.Subscriber(
            name="/waypoints_data", data_class=Float32MultiArray, callback=self.waypoints_callback)
        self.angle_sub = rospy.Subscriber(
            name="/angle", data_class=Float32, callback=self.angle_callback)
        self.depth_control_sub = rospy.Subscriber(
            name="/heave_control_input", data_class=Float32, callback=self.depth_control_callback)
        
        # Publishers
        self.control_input_pub = rospy.Publisher(
            name="/cavepi/user_input", data_class=CavepiInput, queue_size=1)
        self.depth_pub = rospy.Publisher(
            name="/depth_data", data_class=Float32, queue_size=1)
        

        self.state = "lost"
        self.angle = 0.0 # degrees

        # Tracking parameters
        self.is_turning = False
        self.straight_confirm_count = 0
        self.STRAIGHT_THRESHOLD = 5  # Threshold for confirming straight direction

        # PID Controller Parameters
        self.kp_straight = 7.0 # 10
        self.ki_straight = 0.0 # 0
        self.kd_straight = 1.0 # 0.12
        self.error_array = []
        self.dt_array = []
        self.previous_error = 0.0
        self.pid_last_time = None
        self.error_integral = 0.0
        self.count_integrated_errors = 5
        self.max_speed = 0.05 # 10% of the max speed
        self.slow_speed_factor = 0.3
        self.kp_turn = 1000.0
        self.ki_turn = 0.0
        self.kd_turn = 0.0
        self.status_change_time = time.time()

        self.heave = 0.0
        self.depth_to_hold = 1.4
        self.kp_depth = 4.0
        self.ki_depth = 0.0
        self.kd_depth = 0.0


    def state_callback(self, msg):
        self.state = msg.data


    def depth_control_callback(self, msg):
        self.heave = msg.data * self.max_speed


    def angle_callback(self, msg):
        self.angle = msg.data

        surge = 0.0
        yaw = 0.0
        previous_cmd = [surge, yaw]
        error = self.angle/180.0

        if self.state == "straight":
            
            yaw = self.compute_pid_control(error, self.kp_straight, self.ki_straight, self.kd_straight) * self.max_speed
            surge = self.max_speed - yaw
            print('state is straight!')
            if self.angle >= 0:
                rospy.loginfo('Adjust Right!')
            else:
                rospy.loginfo('Adjust Left!')

        elif self.state == "turn":
            surge = self.max_speed - yaw
            yaw = self.compute_pid_control(error, self.kp_turn, self.ki_turn, self.kd_turn) * self.max_speed

            if self.angle >= 0:
                rospy.loginfo('Move Right!')
            else:
                rospy.loginfo('Move Left!')
        else:
            surge = 0.0
            yaw = 0.0

        control_cmd = CavepiInput()
        control_cmd.surge = surge
        control_cmd.heave = self.heave
        control_cmd.yaw = yaw
        control_cmd.roll = 0.0
        self.control_input_pub.publish(control_cmd)


    def waypoints_callback(self, msg):
        offset_x = msg.data[0]
        offset_y = msg.data[1]        


    # Function to Calculate PID Control
    def compute_pid_control(self, error, kp, ki, kd):
        current_time = time.time()
        if self.pid_last_time is None:
            dt = 0.1 # seconds
        else:
            dt = current_time - self.pid_last_time
        self.pid_last_time = current_time
        
        self.dt_array.append(dt)
        self.error_array.append(error)
        if len(self.error_array) > self.count_integrated_errors:
            self.error_array.pop(0)
            self.dt_array.pop(0)
        self.error_integral = np.dot(self.error_array, self.dt_array)
        error_derivative = (error - self.previous_error) / dt
        control_signal = (kp * error) + (ki * self.error_integral) + (kd * error_derivative)
        control_signal = np.clip(control_signal, -1.0, 1.0)
        self.previous_error = error

        return control_signal


if __name__ == '__main__':
    try:
        node = CavelineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
