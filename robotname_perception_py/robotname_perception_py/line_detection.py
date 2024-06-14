
from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String

from robotname_msgs.msg import DetectionArray
from robotname_msgs.msg import Detection
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2


import math
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import numpy as np

import tf2_ros

from geometry_msgs.msg import Quaternion
from rclpy.time import Time
from rclpy.duration import Duration

from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer, Cache

from image_geometry import PinholeCameraModel

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class LineDetection(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("depth_camera_node", **kwargs)

        # params
        self.declare_parameter("model", "omniomniomni.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('linedetection created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self._pub_ann = self.create_lifecycle_publisher(
            Image, "/camera/line_ann", 1) 
        
        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.realsense_cam_model = PinholeCameraModel()
        self.cam_transform = None


        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        self._sub = self.create_subscription(
            RGBD,
            "/camera1/rgbd",
            self.image_cb,
            self.image_qos_profile
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

            

    def image_cb(self, image: RGBD) -> None:
        if self.enable:
            self.get_logger().info('blaba')

            self.realsense_cam_model.fromCameraInfo(image.rgb_camera_info)
            # # convert image + predict                   
            cv_rgb_image = self.cv_bridge.imgmsg_to_cv2(image.rgb, desired_encoding='bgr8')
            cv_depth_image = self.cv_bridge.imgmsg_to_cv2(image.depth, desired_encoding="passthrough")

            
            # Convert the image to HLS color space
            hls_image = cv2.cvtColor(cv_rgb_image, cv2.COLOR_BGR2HLS)

            
            # Extract the Lightness component
            lightness = hls_image[:,:,1]

            # # Apply a threshold to the Lightness component
            _, binary_image = cv2.threshold(lightness, 200, 255, cv2.THRESH_BINARY)

            # extract edges with canny edge detector
            edges = cv2.Canny(binary_image, 20, 150)

        # Find contours in the edges
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the original image
            cv2.drawContours(cv_rgb_image, contours, -1, (0, 255, 0), 2)

        # apply HoughLinesP
        # cv2.HoughLinesP() returns an array of lines detected in the image.
        # lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=75, minLineLength=100, maxLineGap=10)

        # if lines is not None:
        #     for i, line in enumerate(lines):
        #         x1, y1, x2, y2 = line[0]
        #         dy = (y2 - y1)
        #         dx = (x2 - x1)
        #         # convert radian to degree and extract angle
        #         angle = np.rad2deg(np.arctan2(dy, dx))
                
        #         # Since the Y-axis increases downwards(in opencv), invert the angle.
        #         angle = 180 - angle if angle > 0 else -angle
                
        #         # different color for every line
        #         color = tuple(np.random.randint(0, 255, 3).tolist())
                
        #         # detected line
        #         cv2.line(cv_rgb_image, (x1, y1), (x2, y2), color, 3)
        #         # draw shape to starting and finishing points of lines
        #         cv2.putText(cv_rgb_image, '>', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
        #         cv2.putText(cv_rgb_image, '<', (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
        #         # angle
        #         cv2.putText(cv_rgb_image, str(round(angle, 1)),(x1 , int((y1+y2)/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

            # Find contours in the binary mask
            newmsg = self.cv_bridge.cv2_to_imgmsg(cv_rgb_image)
            self._pub_ann.publish(newmsg)


def main():
    rclpy.init()
    node = LineDetection()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
