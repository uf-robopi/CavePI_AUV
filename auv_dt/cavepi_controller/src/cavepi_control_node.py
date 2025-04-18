#!/usr/bin/env python3

# =====================================
# Author : Alakrit Gupta
# Email: gupta.alankrit@ufl.edu
# =====================================

import rospy
from cavepi_interfaces.msg import CavepiInput
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float32
from gazebo_msgs.msg import LinkStates
from tf.transformations import *
from scipy.spatial.transform import Rotation as R
import numpy as np
import os


class CavepiControlNode:
    def __init__(self):
        rospy.init_node('cavepi_control_node')

        # Subscribers
        self.user_input_sub = rospy.Subscriber(
            name="/cavepi/user_input", data_class=CavepiInput, callback=self.input_callback)
        self.gazebo_states_sub = rospy.Subscriber(
            name="/gazebo/link_states", data_class=LinkStates, callback=self.states_callback)

        # Publishers
        self.front_right_thrust_pub = rospy.Publisher(
            name="/front_right_thrust", data_class=Wrench, queue_size=1)
        self.front_left_thrust_pub = rospy.Publisher(
            name="/front_left_thrust", data_class=Wrench, queue_size=1)
        self.rear_right_thrust_pub = rospy.Publisher(
            name="/rear_right_thrust", data_class=Wrench, queue_size=1)
        self.rear_left_thrust_pub = rospy.Publisher(
            name="/rear_left_thrust", data_class=Wrench, queue_size=1)
        self.drag_force_pub = rospy.Publisher(
            name="/drag_force", data_class=Wrench, queue_size=1)
        self.deviation_error_publisher = rospy.Publisher(
            name="/deviation_error", data_class=Float32, queue_size=1)

        # Parameters
        self.front_right_thrust_raw = 0.0
        self.front_left_thrust_raw = 0.0
        self.rear_right_thrust_raw = 0.0
        self.rear_left_thrust_raw = 0.0

        self.max_forward_force = 51.485 # Newtons
        self.max_reverse_force = 40.207 # Newtons


        self.log_file_path = os.path.expanduser("/home/alankrit/Desktop/deviation_error_log.txt")
        self.log_file = open(self.log_file_path, "a")
        self.log_file.write("Timestamp,Deviation_Error\n")
        

    def input_callback(self, msg):
        surge = msg.surge
        heave = msg.heave
        roll = msg.roll
        yaw = msg.yaw

        if surge is not None and yaw is not None:
            if surge >= 0:
                surge_force = self.max_forward_force
            else:
                surge_force = self.max_reverse_force

        self.rear_left_thrust_raw = surge_force * (surge + yaw)
        self.rear_right_thrust_raw = surge_force * (surge - yaw)


        if heave is not None and roll is not None:
            if heave <= 0:
                heave_force = self.max_forward_force
            else:
                heave_force = self.max_reverse_force

        self.front_left_thrust_raw = heave_force * (heave + roll)
        self.front_right_thrust_raw = heave_force * (heave - roll)

        # if heave < 0:
        #     if roll >= 0:
        #         self.front_left_thrust_raw = -self.max_forward_force * ((heave**2 + roll**2)**0.5)
        #         self.front_right_thrust_raw = self.max_forward_force * heave
        #     else:
        #         self.front_left_thrust_raw = self.max_forward_force * heave
        #         self.front_right_thrust_raw = -self.max_forward_force * ((heave**2 + roll**2)**0.5)
        # else:
        #     if roll >= 0:
        #         self.front_left_thrust_raw = self.max_reverse_force * ((heave**2 + roll**2)**0.5)
        #         self.front_right_thrust_raw = self.max_reverse_force * heave
        #     else:
        #         self.front_left_thrust_raw = self.max_reverse_force * heave
        #         self.front_right_thrust_raw = self.max_reverse_force * ((heave**2 + roll**2)**0.5)

        # if surge >= 0:
        #     if yaw >= 0:
        #         self.rear_left_thrust_raw = self.max_forward_force * ((surge**2 + yaw**2)**0.5)
        #         self.rear_right_thrust_raw = self.max_forward_force * surge
        #     else:
        #         self.rear_left_thrust_raw = self.max_forward_force * surge
        #         self.rear_right_thrust_raw = self.max_forward_force * ((surge**2 + yaw**2)**0.5)
        # else:
        #     if yaw >= 0:
        #         self.rear_left_thrust_raw = -self.max_reverse_force * ((surge**2 + yaw**2)**0.5)
        #         self.rear_right_thrust_raw = self.max_reverse_force * surge
        #     else:
        #         self.rear_left_thrust_raw = self.max_reverse_force * surge
        #         self.rear_right_thrust_raw = -self.max_reverse_force * ((surge**2 + yaw**2)**0.5)



    def states_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]=="CavePI::front_right_thruster_link_cavepi":
                frt_i = i
            if msg.name[i]=="CavePI::front_left_thruster_link_cavepi":
                flt_i = i
            if msg.name[i]=="CavePI::rear_right_thruster_link_cavepi":
                rrt_i = i
            if msg.name[i]=="CavePI::rear_left_thruster_link_cavepi":
                rlt_i = i
            if msg.name[i]=="CavePI::base_link":
                base_i = i

        deviation_error = Float32()
        deviation_error.data = msg.pose[base_i].position.y
        timestamp = rospy.get_time()
        self.deviation_error_publisher.publish(deviation_error)

        log_entry = f"{timestamp},{deviation_error.data}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()

        
        # Calculation of The Final Thrust on The Front Right Thruster
        fr_rotation = [msg.pose[frt_i].orientation.x, msg.pose[frt_i].orientation.y, msg.pose[frt_i].orientation.z, msg.pose[frt_i].orientation.w]
        frt_vec_before = [0, 0, self.front_right_thrust_raw, 0]
        frt_vec_after = quaternion_multiply(quaternion_multiply(fr_rotation, frt_vec_before), quaternion_conjugate(fr_rotation))
        front_right_thrust = Wrench()
        front_right_thrust.force.x = frt_vec_after[0]
        front_right_thrust.force.y = frt_vec_after[1]
        front_right_thrust.force.z = frt_vec_after[2]
        self.front_right_thrust_pub.publish(front_right_thrust)

        # Calculation of The Final Thrust on The Front Left Thruster
        fl_rotation = [msg.pose[flt_i].orientation.x, msg.pose[flt_i].orientation.y, msg.pose[flt_i].orientation.z, msg.pose[flt_i].orientation.w]
        flt_vec_before = [0, 0, self.front_left_thrust_raw, 0]
        flt_vec_after = quaternion_multiply(quaternion_multiply(fl_rotation, flt_vec_before), quaternion_conjugate(fl_rotation))
        front_left_thrust = Wrench()
        front_left_thrust.force.x = flt_vec_after[0]
        front_left_thrust.force.y = flt_vec_after[1]
        front_left_thrust.force.z = flt_vec_after[2]
        self.front_left_thrust_pub.publish(front_left_thrust)

        # Calculation of The Final Thrust on The Rear Right Thruster
        rr_rotation = [msg.pose[rrt_i].orientation.x, msg.pose[rrt_i].orientation.y, msg.pose[rrt_i].orientation.z, msg.pose[rrt_i].orientation.w]
        rrt_vec_before= [self.rear_right_thrust_raw, 0, 0, 0]
        rrt_vec_after = quaternion_multiply(quaternion_multiply(rr_rotation, rrt_vec_before), quaternion_conjugate(rr_rotation))
        rear_right_thrust = Wrench()
        rear_right_thrust.force.x = rrt_vec_after[0]
        rear_right_thrust.force.y = rrt_vec_after[1]
        rear_right_thrust.force.z = rrt_vec_after[2]
        self.rear_right_thrust_pub.publish(rear_right_thrust)

        # Calculation of The Final Thrust on The Rear Left Thruster
        rl_rotation = [msg.pose[rlt_i].orientation.x, msg.pose[rlt_i].orientation.y, msg.pose[rlt_i].orientation.z, msg.pose[rlt_i].orientation.w]
        rlt_vec_before = [self.rear_left_thrust_raw, 0, 0, 0]
        rlt_vec_after = quaternion_multiply(quaternion_multiply(rl_rotation, rlt_vec_before), quaternion_conjugate(rl_rotation))
        rear_left_thrust = Wrench()
        rear_left_thrust.force.x = rlt_vec_after[0]
        rear_left_thrust.force.y = rlt_vec_after[1]
        rear_left_thrust.force.z = rlt_vec_after[2]
        self.rear_left_thrust_pub.publish(rear_left_thrust)


        # Drage Force Calculations
        '''
        Reference frame {B} is achieved after rotation of reference frame {A} by quaternion A_quat_B. Both reference frames represent the base_link.
        '''
        A_quat_B = [msg.pose[base_i].orientation.x, msg.pose[base_i].orientation.y, msg.pose[base_i].orientation.z, msg.pose[base_i].orientation.w]
        A_quat_B /= np.linalg.norm(A_quat_B)
        A_R_B = (R.from_quat(A_quat_B)).as_matrix()
        
        lin_rel_vel_in_A = [msg.twist[base_i].linear.x, msg.twist[base_i].linear.y, msg.twist[base_i].linear.z]
        ang_rel_vel_in_A = [msg.twist[base_i].angular.x, msg.twist[base_i].angular.y, msg.twist[base_i].angular.z]
        lin_rel_vel_in_B = np.transpose(A_R_B) @ lin_rel_vel_in_A
        ang_rel_vel_in_B = np.transpose(A_R_B) @ ang_rel_vel_in_A

        rho = 1000  # Water Density
        area = [0.053, 0.065, 0.087] # Projected area in m^2
        drag_coef_force = 1.2 # For cylinders
        drag_coef_torque = drag_coef_force * np.array([0.05, 0.1, 0.1])

        drag_force_in_B = np.zeros(3)
        drag_torque_in_B = np.zeros(3)
        for i in range(len(lin_rel_vel_in_B)):
            drag_force_in_B[i] = -(1/2) * rho * drag_coef_force * area[i] * lin_rel_vel_in_B[i] * abs(lin_rel_vel_in_B[i])
            drag_torque_in_B[i] = -(1/2) * rho * drag_coef_torque[i] * area[i] * ang_rel_vel_in_B[i] * abs(ang_rel_vel_in_B[i])

        drag_force_in_A = A_R_B @ drag_force_in_B
        drag_torque_in_A = A_R_B @ drag_torque_in_B

        drag = Wrench()
        drag.force.x = drag_force_in_A[0]
        drag.force.y = drag_force_in_A[1]
        drag.force.z = drag_force_in_A[2]
        drag.torque.x = drag_torque_in_A[0]
        drag.torque.y = drag_torque_in_A[1]
        drag.torque.z = drag_torque_in_A[2]
        self.drag_force_pub.publish(drag)

        # drag = Wrench()
        # drag.force.x = 0.0
        # drag.force.y = 0.0
        # drag.force.z = 0.0
        # drag.torque.x = 0.0
        # drag.torque.y = 0.0
        # drag.torque.z = 0.0
        # self.drag_force_pub.publish(drag)


    def shutdown_hook(self):
        """Close the file when the node shuts down."""
        if self.log_file:
            self.log_file.close()
            rospy.loginfo("Deviation error log file closed.")




if __name__ == '__main__':
    try:
        node = CavepiControlNode()
        rospy.on_shutdown(node.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
