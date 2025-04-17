#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

 
class AutopilotUtilNode(Node):
    def __init__(self):
        super().__init__('autopilot_util')

        '''This node is sending signals to the 'autopilot' node when to go straight, turn, or search the caveline again if lost.'''

        # SUBSCRIBERS
        # Pixhawk Arm/Disarm Information Subscriber
        self.Pixhawk_arm_subscriber = self. create_subscription(
            msg_type=Bool, topic='/Pixhawk_arm_state', callback=self.callback_Pixhawk_arm, qos_profile=10)
 
        # caveline Status Data Subscriber
        self.motion_direction_subscriber = self.create_subscription(
            msg_type=String, topic='/state_data', callback=self.callback_motion_direction, qos_profile=10)
        
        # PUBLISHERS
        # Depth Hold State Information Publisher
        self.depth_hold_publisher = self.create_publisher(
            msg_type=Bool, topic='/depth_hold_state', qos_profile=10)
        
        # Tracking Signal Publisher
        self.tracking_publisher = self.create_publisher(
            msg_type=Int32, topic='/tracking_signal', qos_profile=10)

        # Parameter Initialization
        self.is_Pixhawk_armed = False
        self.current_motion_state = "lost"
        self.tracking = 0
        self.is_depth_hold_started = False
        self.is_caveline_in_view = False
        self.status_change_instant = self.get_clock().now()        
        
 
    def callback_Pixhawk_arm(self, msg):
        '''
        Callback Function to Know if the Pixhawk is Armed
        '''
        self.is_Pixhawk_armed = msg.data
 
    
    def callback_motion_direction(self, msg):
        '''
        Callback Function to Receive 'state' Data from Jetson Nano
        '''
        previous_motion_state = self.current_motion_state
        previous_tracking = self.tracking
        self.current_motion_state = msg.data
        '''
        TRACKING SIGNAL
        0: lost
        1: straight
        2: turn
        3: wait
        '''        

        if not self.is_Pixhawk_armed: # If Pixhawk is disarmed.
            self.status_change_instant = self.get_clock().now()

        else: # If Pixhawk is armed.
            if not self.is_depth_hold_started: # If Pixhawk is armed and depth hold not started.                
                if self.current_motion_state != "lost": # If Pixhawk is armed, depth hold not started, and current caveline position not 'lost'.
                    # If Pixhawk is armed, depth hold not started, and current caveline position not 'lost' but previous caveline position was 'lost'.
                    if previous_motion_state == "lost":
                        self.status_change_instant = self.get_clock().now()                    
                    else: # If Pixhawk is armed, depth hold not started, current and previous caveline positions are not 'lost'.
                        current_time = self.get_clock().now()
                        elapsed_time = (current_time - self.status_change_instant).nanoseconds/1e9

                        # Depth Hold mode should be started only when the caveline is in the view for more than 5 seconds.
                        if elapsed_time >= 5.0:
                            self.is_caveline_in_view = True
                            self.is_depth_hold_started = True
                            self.tracking = 3  # wait
                            # If the caveline is in the view for more than 10 seconds, CavePI can now start moving.
                            if elapsed_time >= 10.0: 
                                if self.current_motion_state == "straight":
                                    self.tracking = 1
                                elif self.current_motion_state == "turn":
                                    self.tracking = 2

            else: # If Pixhawk is armed and depth hold is started.                
                if self.current_motion_state != "lost": # If Pixhawk is armed, depth hold started, and current caveline position not 'lost'.
                    self.is_caveline_in_view = True
                    if self.current_motion_state == "straight":
                        self.tracking = 1
                    elif self.current_motion_state == "turn":
                        self.tracking = 2
                    
                    # If Pixhawk is armed, depth hold started, and current caveline position not 'lost', but previous caveline position was 'lost'.
                    if previous_motion_state == "lost":
                        self.get_logger().info("Caveline found again!")
                
                else: # If Pixhawk is armed, depth hold started, and current caveline position is 'lost'.
                    # If Pixhawk is armed, depth hold started, and current caveline position 'lost', but previous caveline position was not 'lost'.
                    if previous_motion_state != "lost":
                        self.status_change_instant = self.get_clock().now()
                        self.tracking = previous_tracking
                    else: # If Pixhawk is armed, depth hold started, and current and previous caveline positions are 'lost'.
                        self.tracking = previous_tracking
                        current_time = self.get_clock().now()
                        elapsed_time = (current_time - self.status_change_instant).nanoseconds/1e9

                        # If caveline is lost for more than 5 seconds, inform the vehicle.
                        if elapsed_time >= 5.0:
                            self.is_caveline_in_view = False
                            self.tracking = 0

        # Publish Depth-Hold Data
        depth_hold_state = Bool()
        depth_hold_state.data = self.is_depth_hold_started
        self.depth_hold_publisher.publish(depth_hold_state)
 
        # Publish Tracking Signal Data
        tracking_signal = Int32()
        tracking_signal.data = self.tracking
        self.tracking_publisher.publish(tracking_signal) 
                
 
def main(args=None):
    rclpy.init(args=args)
    node = AutopilotUtilNode()   
 
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
