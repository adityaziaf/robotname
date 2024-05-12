
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

class Yolov8Node(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)

        # params
        self.declare_parameter("model", "omniv8.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('Yolov8Node created')

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

        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "/omni/objects/raw", 1)
        
        self._pub_ann = self.create_lifecycle_publisher(
            Image, "/omni/annotated_img", 1)
        

        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cam_transform = None
        self.base_transform = None

        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        self.yolo = YOLO(self.model)
        self.yolo.fuse()

        # subs
        self._sub = self.create_subscription(
            Image,
            "/omni/image_raw",
            self.image_cb,
            self.image_qos_profile
        )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        del self.yolo
        if 'cuda' in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_transformation(self, results: Results) -> List[PoseStamped]:

        boxes_list = []

        box_data: Boxes

        for box_data in results.boxes:

            msg = PoseStamped()

            # get boxes values
            box = box_data.xyxy[0]
            # msg.center.position.x = float(box[0])
            # msg.center.position.y = float(box[1])
            # msg.size.x = float(box[2])
            # msg.size.y = float(box[3])

            # append msg
            distancex = int((box[0] + box[2]) / 2) - 313 #center x
            distancey = int((box[1] + box[3]) / 2) - 230 #center y

            objdistance = math.hypot(distancex,distancey)
            theta = math.atan2(distancey, distancex)

            real_dist = 0.006310*objdistance - 0.475
            #real_dist = 0.004310*objdistance - 0.475
            if real_dist < 0.0: 
                real_dist = 0.0  
                        
                        # omnicam_link to base_link
            poseInSourceFrame = PoseStamped()
            poseInTargetFrame = PoseStamped() 
            poseinTargetMap = PoseStamped()
                        
            poseInSourceFrame.pose.position.x = real_dist*math.sin(theta)
            poseInSourceFrame.pose.position.y = real_dist*math.cos(theta)
            poseInSourceFrame.pose.position.z = 0.0
            poseInSourceFrame.header.frame_id = "omnicam_link"

            poseInTargetFrame = tf2_geometry_msgs.do_transform_pose_stamped(poseInSourceFrame, self.cam_transform)
                        
            quaternion = get_quaternion_from_euler(0.0, 0.0, math.atan2(poseInTargetFrame.pose.position.y, poseInTargetFrame.pose.position.x))
            quat = Quaternion()
                        
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]
                        
            poseInTargetFrame.pose.orientation = quat
            poseinTargetMap = tf2_geometry_msgs.do_transform_pose_stamped(poseInTargetFrame, self.base_transform)
    
            boxes_list.append(poseinTargetMap)

        return boxes_list

    def image_cb(self, msg: Image) -> None:

        if self.enable:
            
            try:
                self.cam_transform = self.tf_buffer.lookup_transform('base_link', 'omnicam_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1))
                # self.get_logger().info('Got transform from {} to {}: {}'.format(
                # self.cam_transform.header.frame_id, self.cam_transform.child_frame_id,
                # self.cam_transform.transform))
            except tf2_ros.LookupException as e:
                    self.get_logger().error('Failed to lookup transform: {}'.format(e))
                    return 
            except tf2_ros.ExtrapolationException as e:
                    self.get_logger().error('Failed to extrapolate transform: {}'.format(e))
                    return 

            try:    
                    self.base_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1))
                    # self.get_logger().info('Got transform from {} to {}: {}'.format(
                    # self.base_transform.header.frame_id, self.base_transform.child_frame_id,
                    # self.base_transform.transform))
                    
            except tf2_ros.LookupException as e:
                    self.get_logger().error('Failed to lookup transform: {}'.format(e))
                    return 
            except tf2_ros.ExtrapolationException as e:
                    self.get_logger().error('Failed to extrapolate transform: {}'.format(e))
                    return 

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.flip(cv_image, 1)

            results = self.yolo.track(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=0.5,
                persist=False,
                device=self.device,
                iou=0.5,
                imgsz=640
            )

            ann_results: Results = results[0].cpu()

            if ann_results.boxes:
                hypothesis = self.parse_hypothesis(ann_results)
                poses = self.parse_transformation(ann_results)

            # if results.masks:
            #     masks = self.parse_masks(results)

            # if results.keypoints:
            #     keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(ann_results)):

                aux_msg = Detection()

                if ann_results.boxes:
                    aux_msg.id = hypothesis[i]["class_id"]
                    aux_msg.classname = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]
                    aux_msg.pose = poses[i]


                detections_msg.detections.append(aux_msg)

            # # publish detections
            # #detections_msg.header = msg.header
            self._pub.publish(detections_msg)

            newmsg = self.cv_bridge.cv2_to_imgmsg(results[0].plot())
            self._pub_ann.publish(newmsg)
            del results
            del cv_image


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()