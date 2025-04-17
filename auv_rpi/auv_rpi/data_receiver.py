#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32, Float32
import socket, json, threading, time

class DataReceiver(Node):
   def __init__(self):
       super().__init__('data_receiver')

       '''This node is collecting the data sent from Jetson Nano through udp and publishing them in appropriate ROS topics.'''
    
       self.udp_ip = "0.0.0.0"
       self.udp_port = 5005
       self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       self.sock.bind((self.udp_ip, self.udp_port))
       
       self.qr_publisher = self.create_publisher(
           msg_type=Int32, topic='/qr_data', qos_profile=10)
       self.state_publisher = self.create_publisher(
           msg_type=String, topic='/state_data', qos_profile=10)
       self.waypoints_publisher = self.create_publisher(
           msg_type=Float32MultiArray, topic='/waypoints_data', qos_profile=10)
       self.angle_publisher = self.create_publisher(
           msg_type=Float32, topic='/angle_data', qos_profile=10)
       
       self.thread = threading.Thread(target=self.receive_udp_data)
       self.thread.daemon = True
       self.thread.start()
   
   
   def receive_udp_data(self):
       '''
       Function to Receive Data Through UDP
       '''
       while True:
           try:
               data, addr = self.sock.recvfrom(1024) 
               message = json.loads(data.decode('utf-8'))
               data_type = message.get('type')
               payload = message.get('data')
               if data_type == 'qr_info':
                   self.publish_qr_data(payload)
               elif data_type == 'state':
                   self.publish_state_data(payload)
               elif data_type == 'waypoints':
                   self.publish_waypoints_data(payload)
               elif data_type == 'angle':
                   self.publish_angle_data(payload)
               else:
                   self.get_logger().warn(f"Unknown data type received: {data_type}")
           except Exception as e:
               self.get_logger().error(f"Error receiving or processing data: {e}")


   def publish_qr_data(self, data):
       '''
       Function to Publish QR Data in ROS Topic
       '''
       msg = Int32()
       msg.data = data.get('qr_data', '')
       self.qr_publisher.publish(msg)
   
   
   def publish_state_data(self, data):
       '''
       Function to Publish State Data in ROS Topic
       '''
       msg = String()
       msg.data = data.get('direction', '')
       self.state_publisher.publish(msg)
   
   
   def publish_waypoints_data(self, data):
       '''
       Function to Publish Waypoints Data in ROS Topic
       '''
       msg = Float32MultiArray()
       x = data.get('x', 0.0)
       y = data.get('y', 0.0)
       msg.data = [x, y]
       self.waypoints_publisher.publish(msg)


   def publish_angle_data(self, data):
       '''
       Function to Publish Angle Data in ROS Topic
       '''
       msg = Float32()
       msg.data = data.get('angle', 0.0)
       self.angle_publisher.publish(msg)


def main(args=None):
   rclpy.init(args=args)
   node = DataReceiver()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
   main()
