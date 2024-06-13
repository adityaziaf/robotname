
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


# from norfair import (
#     AbsolutePaths,
#     Detection,
#     FixedCamera,
#     Tracker,
#     Video,
#     draw_absolute_grid,
# )

# from norfair.camera_motion import (
#     HomographyTransformationGetter,
#     MotionEstimator,
#     TranslationTransformationGetter,
# )

# from norfair.drawing import draw_points, draw_boxes

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

class DepthCameraNode(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("depth_camera_node", **kwargs)

        # params
        self.declare_parameter("model", "regional.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('DepthCameraNode created')

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
            DetectionArray, "/camera1/objects/raw", 1)
        
        self._pub_ann = self.create_lifecycle_publisher(
            Image, "/camera1/ann_image", 1) 
        
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

        self.yolo = YOLO(self.model)
        self.yolo.fuse()
        self._sub = self.create_subscription(
            RGBD,
            "/camera1/rgbd",
            self.image_cb,
            self.image_qos_profile
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # motion_estimator = MotionEstimator(
        #         max_points=900,
        #         min_distance=14,
        #         transformations_getter=HomographyTransformationGetter(),
        #     )
        
        # tracker = Tracker(
        #     distance_function="iou",
        #     detection_threshold=0.5,
        #     distance_threshold=200,
        #     initialization_delay=3,
        #     hit_counter_max=30,
        # )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        del self.yolo
        if 'cuda' in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS


    # def yolo_detections_to_norfair_detections(self, results: Results):
    #     norfair_detections = []
    #     boxes = []
    #     box_data: Boxes
    #     for box_data in results.boxes:
    #         bbox = np.array(
    #             [
    #                 [box_data[0].item(), box_data[1].item()],
    #                 [box_data[2].item(), box_data[3].item()],
    #             ]
    #         )
    #         boxes.append(bbox)
    #         # if track_boxes:
    #         points = bbox
    #         scores = np.array([box_data[4], box_data[4]])
    #         # else:
    #         #     points = bbox.mean(axis=0, keepdims=True)
    #         #     scores = detection_as_xyxy[[4]]

    #         norfair_detections.append(
    #             Detection(points=points, scores=scores, label=box_data[-1].item())
    #         )

    #     return norfair_detections, boxes


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

    def parse_transformation(self, results: Results, depth_img) -> List[PoseStamped]:

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

            # center of pixel
            distancex = int((box[0] + box[2]) / 2)
            distancey = int((box[1] + box[3]) / 2)

            #get depth value at x,y center 
            depth_at = 0.001 * depth_img[distancey][distancex]
            #project pixel to 3d world 
            obj_coor = self.realsense_cam_model.projectPixelTo3dRay([distancex,distancey])
            # omnicam_link to base_link
            poseInSourceFrame = PoseStamped()
            poseInTargetFrame = PoseStamped() 

            # depth times the projection value
            poseInSourceFrame.pose.position.x = depth_at * obj_coor[2]
            poseInSourceFrame.pose.position.y = -depth_at * obj_coor[0]
            poseInSourceFrame.pose.position.z = -depth_at * obj_coor [1]
            poseInSourceFrame.pose.orientation.x = 0.0
            poseInSourceFrame.pose.orientation.y = 0.0
            poseInSourceFrame.pose.orientation.z = 0.0
            poseInSourceFrame.pose.orientation.w = 1.0
            poseInSourceFrame.header.frame_id = "camera_link"
            quaternion = get_quaternion_from_euler(0.0, 0.0, math.atan2(poseInSourceFrame.pose.position.y, poseInSourceFrame.pose.position.x))
            quat = Quaternion()
                        
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]

            poseInSourceFrame.pose.orientation = quat

            #poseInSourceFrame.pose.position.x = math.hypot(poseInSourceFrame.pose.position.x, poseInSourceFrame.pose.position.y)
            #poseInSourceFrame.pose.position.y = 0.0
            
            poseInTargetFrame = tf2_geometry_msgs.do_transform_pose_stamped(poseInSourceFrame, self.cam_transform)

            boxes_list.append(poseInTargetFrame)

        return boxes_list
            

    def image_cb(self, image: RGBD) -> None:
        if self.enable:
            #self.get_logger().info('blaba')
            try:
                self.cam_transform = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time(), rclpy.duration.Duration(nanoseconds=1000))
            #     # self.get_logger().info('Got transform from {} to {}: {}'.format(
            #     # self.cam_transform.header.frame_id, self.cam_transform.child_frame_id,
            #     # self.cam_transform.transform))
            except tf2_ros.LookupException as e:
                self.get_logger().error('Failed to lookup transform: {}'.format(e))
                return 
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().error('Failed to extrapolate transform: {}'.format(e))
                return 
    
            self.realsense_cam_model.fromCameraInfo(image.rgb_camera_info)
            # # convert image + predict                   
            cv_rgb_image = self.cv_bridge.imgmsg_to_cv2(image.rgb, desired_encoding='bgr8')
            cv_depth_image = self.cv_bridge.imgmsg_to_cv2(image.depth, desired_encoding="passthrough")

            results = self.yolo.predict(
                source=cv_rgb_image,
                verbose=False,
                stream=False,
                conf=0.5,
                device=self.device,
                iou=0.5,
                imgsz=640
            )
            ann_results: Results = results[0].cpu()

            # detections, boxes = self.yolo_detections_to_norfair_detections(ann_results)

            # coord_transformations = self.motion_estimator.update(cv_rgb_image, None)

            # tracked_objects = self.tracker.update(
            #     detections= detections, coord_transformations=coord_transformations
            # )

            
            if ann_results.boxes:
                hypothesis = self.parse_hypothesis(ann_results)
                poses = self.parse_transformation(ann_results, cv_depth_image)

            newmsg = self.cv_bridge.cv2_to_imgmsg(results[0].plot())
            self._pub_ann.publish(newmsg)

            detections_msg = DetectionArray()

            for i in range(len(ann_results)):
                aux_msg = Detection()

                aux_msg.id = hypothesis[i]["class_id"]
                aux_msg.classname = hypothesis[i]["class_name"]
                aux_msg.score = hypothesis[i]["score"]
                aux_msg.pose = poses[i]
                detections_msg.detections.append(aux_msg)
                
            self._pub.publish(detections_msg)

            del results
            del cv_rgb_image
            del cv_depth_image


def main():
    rclpy.init()
    node = DepthCameraNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()