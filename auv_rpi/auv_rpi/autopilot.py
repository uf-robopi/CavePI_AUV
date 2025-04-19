#!/usr/bin/env python3

# =====================================
# Author : Alakrit Gupta
# Email: gupta.alankrit@ufl.edu
# =====================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, Float32MultiArray
from geometry_msgs.msg import Vector3
from pymavlink import mavutil
import time, sys, signal, math
import numpy as np
 
 
class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot')

        '''This is the 'autopilot' node sending control signals to the Pixhawk for CavePI's movement.'''
 
        # MAVLink setup
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to Pixhawk.")
        self.boot_time = time.time()
        signal.signal(signal.SIGINT, self.signal_handler)
        

        # SUBSCRIBERS
        # QR Data Subscriber
        self.qr_subscriber = self.create_subscription(
            msg_type=Int32, topic='/qr_data', callback=self.callback_qr, qos_profile=10)
       
        # Depth-Hold State Information Subscriber
        self.depth_hold_subscriber = self.create_subscription(
            msg_type=Bool, topic='/depth_hold_state', callback=self.callback_depth_hold, qos_profile=10)
            
        # Sonar Data Subscriber
        self.sonar_data_subscriber = self.create_subscription(
            msg_type=Float32, topic='/adjusted_sonar_dist', callback=self.callback_sonar_dist, qos_profile=10)
       
        # Tracking Signal Subscriber
        self.tracking_subscriber = self.create_subscription(
            msg_type=Int32, topic='/tracking_signal', callback=self.callback_track_caveline, qos_profile=10)
            
        # Waypoints Subscriber
        self.waypoints_subscription = self.create_subscription(
            msg_type=Float32MultiArray, topic='/waypoints_data', callback=self.waypoints_callback, qos_profile=10)
        
        # Heading Angle Subscriber
        self.angle_subscriber = self.create_subscription(
            msg_type=Float32, topic='/angle_data', callback=self.angle_callback, qos_profile=10)
       
        # Pixhawk Arm/Disarm Information Publisher
        self.pixhawk_arm_publisher = self.create_publisher(
            msg_type=Bool, topic='/pixhawk_arm_state', qos_profile=10)


        # PUBLISHERS
        # IMU Data Publisher
        self.imu_publisher = self.create_publisher(
            msg_type=Vector3, topic='/imu_data', qos_profile=10)
        self.imu_timer = self.create_timer(timer_period_sec=0.275, callback=self.publish_imu_data)
        
        # Depth Data Publisher
        self.depth_publisher = self.create_publisher(
            msg_type=Float32, topic='/depth_data', qos_profile=10)
        self.depth_timer = self.create_timer(timer_period_sec=0.01, callback=self.publish_depth_data)
        
        # Pressure Data Publisher
        self.pressure_publisher = self.create_publisher(
            msg_type=Float32, topic='/pressure_data', qos_profile=10)
        
        # Depth Hold State Publisher
        self.stable_depth_publisher = self.create_publisher(
            msg_type=Bool, topic='/is_depth_hold_stabilized', qos_profile=10)


        # PARAMETER INITIALIZATION       
        # Tracking parameters
        self.is_turning = False
        self.straight_confirm_count = 0
        self.STRAIGHT_THRESHOLD = 3  # Threshold for confirming straight direction
        self.attitude = [0.0, 0.0, 0.0]
        self.next_heading_deg = 0.0
        
        # PID Controller Parameters
        self.kp_tracking = 3.4
        self.ki_tracking = 0.0
        self.kd_tracking = 0.9
        self.error_array = []
        self.dt_array = []
        self.previous_error = 0.0
        self.pid_last_time = None  
        self.error_integral = 0.0
        self.count_integrated_errors = 10
        self.max_turn_speed = 200

        # QR Code Detection Parameters
        self.invalid_qr = -1
        self.arm_pixhawk_qr = 1
        self.disarm_pixhawk_qr = 0
        self.qr_info = self.disarm_pixhawk_qr

        # Depth Hold Parameters
        self.is_depth_hold_started = False
        self.is_depth_hold_stabilized = False
        self.depth_hold_start_time = time.time()
        self.current_depth = 0.0 # meters
        self.target_depth = 0.35 # meters
        self.bottom_dist = 0.0 # meters
        self.safe_dist_from_bottom = 0.3 # meters
        self.kp_depth_hold = 600.0
        self.ki_depth_hold = 0.0
        self.kd_depth_hold = 50.0

        # Other Parameters
        self.is_pixhawk_armed = False
        self.manual_control_timer = self.create_timer(0.05, self.send_last_manual_control)  # 20 Hz
        self.last_manual_control = {"x": 0, "z": 500, "r": 0}


    def callback_qr(self, msg):
        '''
        Callback Function to Arm/Disarm The Vehicle Using QR Codes
        '''
        
        '''
        QR Data Interpretation
        -1: No/Invalid QR code shown
        0: Disarm the vehicle
        1: Arm the vehicle        
        ''' 
        previous_arm = self.is_pixhawk_armed
        previous_depth_hold = self.is_depth_hold_started
        previous_qr = self.qr_info
        self.qr_info = msg.data
       
        if self.qr_info != previous_qr:
            if self.qr_info == self.arm_pixhawk_qr:
                # If QR code is shown to arm the pixhawk, turn on the lights as well.
                self.turn_on_lights()
                self.is_pixhawk_armed = True
                self.arm_vehicle()
                self.get_logger().info("Vehicle is armed.")
            elif self.qr_info == self.disarm_pixhawk_qr:
                # If QR code is shown to disarm the pixhawk, turn off the lights as well.
                self.turn_off_lights()
                self.is_depth_hold_started = False
                self.is_pixhawk_armed = False
                self.disarm_vehicle()
                self.get_logger().info("Vehicle is disarmed.")
        elif self.qr_info == self.invalid_qr or self.qr_info == previous_qr:
            # In case of no/invalid/previous QR code, keep the AUV at its previous state.
            self.is_depth_hold_started = previous_depth_hold
            self.is_pixhawk_armed = previous_arm

        # Publish the Current Pixhawk Arm State
        pixhawk_arm_state = Bool()
        pixhawk_arm_state.data = self.is_pixhawk_armed
        self.pixhawk_arm_publisher.publish(pixhawk_arm_state)

        out = Bool()
        out.data = self.is_depth_hold_stabilized
        self.stable_depth_publisher.publish(out)
   
 
    def callback_depth_hold(self, msg):
        '''
        Callback Function for Taking Decision Regarding Depth Hold
        '''
        previous_depth_hold_state = self.is_depth_hold_started
        self.is_depth_hold_started = msg.data
        if self.is_depth_hold_started != previous_depth_hold_state:            
            if self.is_depth_hold_started:
                self.activate_depth_hold()
                self.get_logger().info("Depth Hold Activated!")   
            else:
                self.deactivate_depth_hold()
                self.get_logger().info("Depth Hold Deactivated!")
            
    
    def publish_depth_data(self):
        '''
        Function to Publish Depth Data
        '''
        try:
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'SCALED_PRESSURE2':
                    pressure = Float32()
                    pressure.data = msg.press_abs
                    self.pressure_publisher.publish(pressure)
                    self.current_depth = self.calculate_depth_from_pressure(msg.press_abs)
                    depth_data = Float32()
                    depth_data.data = self.current_depth
                    self.depth_publisher.publish(depth_data)
                    # self.get_logger().info(f"Depth: {depth:.2f} meters")

                elif msg_type == 'ATTITUDE':
                    self.attitude[0] = math.degrees(msg.roll)  # Roll angle in degrees
                    self.attitude[1] = math.degrees(msg.pitch) # Pitch angle in degrees
                    self.attitude[2] = math.degrees(msg.yaw)   # Yaw angle in degrees
        except Exception as e:
            self.get_logger().error(f"Error reading data: {e}")
    
 
    def calculate_depth_from_pressure(self, pressure):
        '''
        Function to Calculate Depth from Pressure Data
        '''
        water_density = 1000 # kg/m^3  
        gravity = 9.81 # m/s^2
        reference_pressure = 1021.5 # hPa (Hecto Pascal)
        gauge_pressure = 100*(pressure - reference_pressure) # Pascal
        depth = gauge_pressure / (water_density * gravity)
        return depth
    

    def publish_imu_data(self):
        '''
        Function to Publish IMU Data
        '''
        try:
            imu_data = Vector3()
            imu_data.x = self.attitude[0]  # Roll angle in degrees
            imu_data.y = self.attitude[1]  # Pitch angle in degrees
            imu_data.z = self.attitude[2]  # Yaw angle in degrees
            self.imu_publisher.publish(imu_data)                
        except Exception as e:
            self.get_logger().error(f"Error reading data: {e}")
    

    def callback_sonar_dist(self, msg):
        '''
        Callback Function For Sonar Data
        '''
        distance = msg.data[0]
        confidence = msg.data[1]
        previous_dist = self.bottom_dist
        # Confidence of the sonar data decreases when it is not underwater.
        if confidence >= 90.0:
            self.bottom_dist = distance
        else:
            self.bottom_dist = previous_dist

 
    def waypoints_callback(self, msg):
        '''
        Callback Function for Waypoints
        '''
        waypoint = msg.data
        error_x = waypoint[0]
        error_y = waypoint[1]

    
    def angle_callback(self, msg):
        self.next_heading_deg = msg.data
 
 
    def callback_track_caveline(self, msg):
        '''
        Callback Function for Tracking Caveline
        '''
        tracking_signal = msg.data

        # Initializing parameters for thruster control     
        x = 0
        r = 0
        if self.bottom_dist > self.safe_dist_from_bottom:
            depth_diff = self.current_depth - self.target_depth
        else:
            # If the bottom distance is less than the safe distance, the target depth is adjusted to maintain a safe distance from the bottom.
            depth_diff = self.current_depth - self.target_depth - self.bottom_dist + self.safe_dist_from_bottom

        # Depth is controller by a PID
        # z = 500 + (self.kp_depth_hold * depth_diff)
        z = 500 + self.compute_pid_control(depth_diff, self.kp_depth_hold, self.ki_depth_hold, self.kd_depth_hold)
        z = np.clip(z, 200, 800)
        
        if not self.is_depth_hold_started:
            self.get_logger().info("Depth Hold is not active. Ignoring tracking signal.")
            self.depth_hold_start_time = time.time()
            return
        else:
            if 5 < time.time() - self.depth_hold_start_time < 15:
                self.send_manual_control(x, z, r)
                self.get_logger().info(f'z: {z}')
                return        
        self.is_depth_hold_stabilized = True

        '''
        TRACKING SIGNAL
        0: lost
        1: straight
        2: turn
        3: wait
        '''
        if tracking_signal == 1:
            if self.is_turning:
                # Confirm straight movement after completing a turn
                self.straight_confirm_count += 1
                if self.straight_confirm_count >= self.STRAIGHT_THRESHOLD:
                    self.is_turning = False
                    self.straight_confirm_count = 0
                    self.get_logger().info("Finished turning, moving straight.")
                    x = 190
                    r = 0
            else:
                x = 190
                r = -self.compute_pid_control(self.next_heading_deg, self.kp_tracking, self.ki_tracking, self.kd_tracking)
                r = np.clip(r, -self.max_turn_speed, self.max_turn_speed)
                
        elif tracking_signal == 2:
            self.straight_confirm_count = 0
            self.is_turning = True
            x = -30
            r = -self.compute_pid_control(self.next_heading_deg, 2*self.kp_tracking, self.ki_tracking, self.kd_tracking)
            r = np.clip(r, -self.max_turn_speed, self.max_turn_speed)
            if self.next_heading_deg < 0:
                self.get_logger().info("Turning right.")                
            else:
                self.get_logger().info("Turning left.")
        
        elif tracking_signal == 0:
            self.get_logger().info("Searching for caveline.")
            # If the caveline is lost from the view, CavePI rotates and tries to find the caveline.
            x = 0
            r = 200

        elif tracking_signal == 3:
            self.get_logger().info("Holding position.")
            x = 0
            r = 0
        
        self.send_manual_control(x, z, r)
        self.get_logger().info("callback_track_caveline funtion in pixhawk_logic node is called!")
 
 
    def compute_pid_control(self, error, kp, ki, kd):
        '''
        Function to Calculate PID Control Input
        '''
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
        self.previous_error = error

        return control_signal
        
    
    def arm_vehicle(self):
        '''
        Function to Arm the Vehicle
        '''
        # self.is_pixhawk_armed = True
        self.master.arducopter_arm()
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            pass
        else:
            print("Arming failed or timed out.")
        manual_mode_id = self.master.mode_mapping()['MANUAL']
        self.master.set_mode(manual_mode_id)
        self.get_logger().info('Vehicle is armed in Manual mode!')
 
 
    def disarm_vehicle(self):
        '''
        Function to Disarm the Vehicle
        '''
        manual_mode_id = self.master.mode_mapping()['MANUAL']
        self.master.set_mode(manual_mode_id)
        self.get_logger().info("Vehicle is disarmed in Manual mode!")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()


    def turn_on_lights(self):
        '''
        Function to Turn On Lights
        '''
        # Turn both lights on (~1900us)
        self.set_servo(9, 1900)   # Light 1 on SERVO9
        self.set_servo(10, 1900)  # Light 2 on SERVO10
 
   
    def turn_off_lights(self):
        '''
        Function to Turn Off Lights
        '''
        # Turn both lights off (~1100us)
        self.set_servo(9, 1100)   # Light 1 on SERVO9
        self.set_servo(10, 1100)  # Light 2 on SERVO10


    def set_servo(self, channel, pwm):
        '''
        Function to Send Servo Signals
        '''
        self.master.mav.command_long_send(
            self.master.target_system,        # target_system
            self.master.target_component,     # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
            0,                           # confirmation
            channel,                     # param1: servo channel
            pwm,                         # param2: PWM value
            0,0,0,0,0                   # param3-7 (unused)
        )

    
    def activate_stabilize_mode(self):
        '''
        Function to Activate Stabilize Mode in Pixhawk
        '''
        stabilize_mode = self.master.mode_mapping()['STABILIZE']
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            stabilize_mode)
        self.get_logger().info('Stabilize Mode set!')


    def deactivate_depth_hold(self):
        '''
        Function to Deactivate Depth Hold Mode
        '''
        self.is_depth_hold_started = False
        self.activate_stabilize_mode()
        # self.disarm_vehicle()

    
    def activate_depth_hold(self):
        '''
        Function to Activate Depth Hold Mode
        '''
        # Direct enabling the Depth Hold mode and setting a depth was not working reliably. Hence, the stabilize mode is activated before the Depth Hold mode.
        self.activate_stabilize_mode()
        # self.get_logger().info(f'{self.master.mode_mapping()}')
        # time.sleep(2.0)
        
        depth_hold_mode = self.master.mode_mapping().get('ALT_HOLD', None)
        if depth_hold_mode is None:
            self.get_logger().error("Depth Hold mode not available.")
            return

        self.get_logger().info("Attempting to activate Depth Hold mode...")
        attempt = 0
        max_attempts = 5
        while attempt < max_attempts:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and msg.custom_mode == 0: # 0 means 'STABILIZE MODE'
                self.master.set_mode(depth_hold_mode)
                msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
                time.sleep(0.2)
                attempt += 1
            elif msg and msg.custom_mode == depth_hold_mode:
                # The set_target_depth() function was not working properly, hence a PID depth controller is implemented inside the callback_track_caveline() function.
                # self.set_target_depth(self.target_depth)
                # self.get_logger().info(f"Target depth set to {self.current_depth} meters below the surface.")
                # time.sleep(0.5)
                return
        self.get_logger().error("Failed to activate Depth Hold mode after multiple attempts.")     
        

    def set_target_depth(self, depth):
        '''
        Function to Set The Depth for Depth Hold Mode
        '''
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            ( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), 
            0, 0, -depth, # (x, y WGS84 frame pos - not used), z [m]
            0, 0, 0, # velocities in NED frame [m/s] (not used)
            0, 0, 0, 0, 0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
       
       
    def send_manual_control(self, x, z, r):
        '''
        Function for continuously sending control commands to Pixhawk
        '''
        self.last_manual_control["x"] = int(x)
        self.last_manual_control["z"] = int(z)
        self.last_manual_control["r"] = int(r)
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x),  # Forward movement    # Range [-1000, 1000]
            0,       # No lateral movement # Range [-1000, 1000]
            int(z),  # Constant throttle   # Range [0, 1000]
            int(r),  # Rotation            # Range [-1000, 1000]
            0        # No button press
        )

    def send_last_manual_control(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            self.last_manual_control["x"],
            0,
            self.last_manual_control["z"],
            self.last_manual_control["r"],
            0
        )

        
    def signal_handler(self, sig, frame):
        """Handle CTRL+C for cleanup."""
        self.get_logger().info("Interrupt received. Disarming ROV and exiting.")        
        try:
            self.shutdown_state = True
            self.turn_off_lights()
            self.is_depth_hold_started = False
            self.is_pixhawk_armed = False
            self.disarm_vehicle()
            self.master.close()
        except Exception as e:
            self.get_logger().error(f"Error closing Pixhawk connection: {e}")
        sys.exit(0)
   
 
def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
 
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
        node.turn_off_lights()
        node.is_depth_hold_started = False
        node.is_pixhawk_armed = False
        node.disarm_vehicle()
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
