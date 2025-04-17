#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from brping import Ping1D
import time

class Ping2Publisher(Node):
    def __init__(self):
        super().__init__('ping2_publisher')

        # Publisher for distance and confidence
        self.publisher = self.create_publisher(Float32MultiArray, '/sonar_data', 10)

        # Initialize Ping1D
        self.ping = Ping1D()
        self.serial_port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.get_logger().info(f"Connecting to Ping2 sonar on {self.serial_port} at {self.baudrate} baud.")

        # Retry logic for initialization
        max_attempts = 5
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            self.get_logger().info(f"Attempt {attempt}/{max_attempts} to initialize Ping2 sonar.")
            self.ping.connect_serial(self.serial_port, self.baudrate)

            if self.ping.initialize():
                self.get_logger().info("Ping2 sonar initialized successfully.")
                break
            else:
                self.get_logger().warn("Ping2 initialization failed. Retrying...")
                time.sleep(2)  # Wait before retrying

        if not self.ping.initialize():
            self.get_logger().error("Failed to initialize Ping2 sonar.")
            raise RuntimeError("Failed to initialize Ping2 sonar.")

        # Create a timer to publish data at 10Hz (0.1s interval)
        self.timer = self.create_timer(0.275, self.publish_ping_data)


    def publish_ping_data(self):
        '''
        Callback function for the timer to publish sonar data.
        '''
        data = self.ping.get_distance()
        if data:
            distance = float(data["distance"])/1000.0 # Convert mm to m
            confidence = float(data["confidence"])

            # Create and populate Float32MultiArray message
            msg = Float32MultiArray()
            msg.data = [distance, confidence]

            # Publish the message
            self.publisher.publish(msg)

            self.get_logger().debug(f"Published Distance: {distance} mm, Confidence: {confidence}%")
        else:
            self.get_logger().warn("Failed to get distance data from Ping2 sonar.")

def main(args=None):
    rclpy.init(args=args)
    try:
        ping2_publisher = Ping2Publisher()
        rclpy.spin(ping2_publisher)
    except RuntimeError as e:
        print(f"Runtime error: {e}")
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")
    finally:
        if 'ping2_publisher' in locals():
            ping2_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
