
from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from robotname_msgs.msg import DetectSilo
from robotname_msgs.msg import DetectSiloArray
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

import numpy as np
import math
import tf2_geometry_msgs

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import numpy as np

import tf2_ros

from geometry_msgs.msg import Quaternion
from rclpy.time import Time
from rclpy.duration import Duration


class SiloVisualizer(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("silo_visualizer", **kwargs)
        self.enable = True

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')
        self._pub = self.create_lifecycle_publisher(
            MarkerArray, "/silo/objects/visualize", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')
        self._sub = self.create_subscription(
            DetectSiloArray,
            "/silo/objects/raw",
            self.show_marker, 10
        )
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')
        self.destroy_publisher(self._pub)
        return TransitionCallbackReturn.SUCCESS

    def show_marker(self, msg: DetectSiloArray):
        if self.enable:
            silo_coor = [
                [12, 1],
                [10, 2],
                [8, 3],
                [6, 4],
                [4, 5]
            ]
            if len(msg.detections) == 5:
                for i, object in enumerate(msg.detections):
                    markarray_ = MarkerArray()
                    x, y = silo_coor[i]
                    for ball_index, ball in enumerate(msg.detections.ball):
                        marker_ = Marker()
                        marker_.type = marker_.CUBE
                        marker_.action = marker_.ADD
                        marker_.header.frame_id = object.header
                        marker_.id = ball_index
                        marker_.ns = f'{object.classname}_{ball_index}'
                        
                        marker_.pose.position.x = x + ball_index * 0.5
                        marker_.pose.position.y = y + ball_index * 0.5
                        marker_.pose.position.z = 0.0
                        marker_.pose.orientation.x = 0.0
                        marker_.pose.orientation.y = 0.0
                        marker_.pose.orientation.z = 0.0
                        marker_.pose.orientation.w = 1.0

                        marker_.scale.x = 0.2
                        marker_.scale.y = 0.2
                        marker_.scale.z = 0.2
                        marker_.color.a = 1.0

                        if ball == "redball":
                            marker_.color.r = 1.0
                            marker_.color.g = 0.0
                            marker_.color.b = 0.0
                        elif ball == "blueball":
                            marker_.color.r = 0.0
                            marker_.color.g = 0.0
                            marker_.color.b = 1.0

                        marker_.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
                        markarray_.markers.append(marker_)
                    self.viz_pub.publish(markarray_)
                    del markarray_

def main():
    rclpy.init()
    node = SiloVisualizer()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()