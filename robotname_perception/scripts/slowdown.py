#!/usr/bin/env python3
import cv2
import numpy as np
import math

import rclpy
from rclpy.publisher import Publisher
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion


def callback(msg, publisher):
    publisher.publish(msg)

def main(args=None):
    # Create trackbars for saturation, brightness, and contrast
    # Node name (same as ROS1)
    rclpy.init()
    node_name = "my_ros2_node"

    # # Create a node (similar to ROS1 init_node)
    node = rclpy.create_node(node_name)
    
    # # --- Publisher (similar to ROS1 Publisher) ---
    # # Define topic name and message type
    topic_name = "goal_pose"
    msg_type = PoseStamped  # Replace with your message type

    # # Create publisher with QoS options (optional)
    qos = rclpy.qos.QoSProfile(depth=10)  # Adjust QoS settings as needed
    publisher = node.create_publisher(msg_type, topic_name, qos)
    subscriber = node.create_subscription(msg_type, 'objects',  lambda msg: callback(msg, publisher), qos)
    timer = node.create_timer(0.5, lambda: None)
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node and shutdown
    node.destroy_node()
    rclpy.shutdown()
        
      
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
  main()