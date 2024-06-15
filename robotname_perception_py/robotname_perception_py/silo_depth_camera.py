
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

import torch
from torch import cuda

from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD

from robotname_msgs.msg import DetectSilo
from robotname_msgs.msg import DetectSiloArray
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2

import numpy as np
import math
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import numpy as np

import tf2_ros

from geometry_msgs.msg import Quaternion
from rclpy.time import Time
from rclpy.duration import Duration

import norfair
from norfair import Detection, Tracker

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

def euclidean_distance(point1, point2):
    x_dif = (point2[0] - point1[0])**2
    y_dif = (point2[1] - point1[1])**2
    z_dif = (point2[2] - point1[2])**2
    return math.sqrt(x_dif + y_dif)

class SiloDepthCameraNode(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)

        # params
        self.declare_parameter("model", "silo7Jun.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('SiloDepthCameraNode created')

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
            poseInSourceFrame.header.frame_id = "camera2_link"
            # quaternion = get_quaternion_from_euler(0.0, 0.0, math.atan2(poseInSourceFrame.pose.position.y, poseInSourceFrame.pose.position.x))
            # quat = Quaternion()
                        
            # quat.x = quaternion[0]
            # quat.y = quaternion[1]
            # quat.z = quaternion[2]
            # quat.w = quaternion[3]

            # poseInSourceFrame.pose.orientation = quat

            #poseInSourceFrame.pose.position.x = math.hypot(poseInSourceFrame.pose.position.x, poseInSourceFrame.pose.position.y)
            #poseInSourceFrame.pose.position.y = 0.0
            
            poseInTargetFrame = tf2_geometry_msgs.do_transform_pose_stamped(poseInSourceFrame, self.cam_transform)

            boxes_list.append(poseInTargetFrame)

        return boxes_list
    
    def parse_information(self, results: Results):
        information_list = []
        for box in results.boxes:
            box_coor = box.xyxy.tolist()
            information = {
                "class_name": self.yolo.names[int(box.cls)],
                "size": self.get_area(box_coor[0]),
            }
            information_list.append(information)
        return information_list



    def yolo_detections_to_norfair_detections(self,
        yolo_detections: Results, track_points: str = "bbox" ) -> List[Detection]:
       
        """convert detections_as_xywh to norfair detections"""
        norfair_detections: List[Detection] = []

        # if track_points == "centroid":
        #     detections_as_xywh = yolo_detections.boxes.xywh[0]
        #     for detection_as_xywh in detections_as_xywh:
        #         centroid = np.array(
        #             [detection_as_xywh[0].item(), detection_as_xywh[1].item()]
        #         )
        #         scores = np.array([detection_as_xywh[4].item()])
        #         norfair_detections.append(
        #             Detection(
        #                 points=centroid,
        #                 scores=scores,
        #                 label=int(detection_as_xywh[-1].item()),
        #             )
        #         )

        # for box_data in yolo_detections.boxes:
        #     box = box_data.xyxy[0]
        #     bbox = np.array(
        #             [
        #                 [box[0].item(), box[1].item()],
        #                 [box[2].item(), box[3].item()],
        #             ]
        #         )
        #     norfair_detections.append(
        #             Detection(
        #                 points=bbox, scores=float(box_data.conf), label=int(box_data.cls)
        #             )
        #     )

        for box_data in yolo_detections.boxes:
            box = box_data.xyxy[0]
            distancex = int((box[0] + box[2]) / 2)
            distancey = int((box[1] + box[3]) / 2)
            # bbox = np.array(
            #         [
            #             [box[0], box[1]],
            #             [box[2], box[3]],
            #         ]
            #     )
            norfair_detections.append(
                    Detection(
                        points=np.array([distancex, distancey]),
                        scores=np.array([float(box_data.conf)]),
                        label=int(box_data.cls)
                    )
            )
            
        # elif track_points == "bbox":
        #     detections_as_xyxy = yolo_detections.boxes.xyxy[0]
        #     for detection_as_xyxy in detections_as_xyxy:
        #         bbox = np.array(
        #             [
        #                 [detection_as_xyxy[0].item(), detection_as_xyxy[1].item()],
        #                 [detection_as_xyxy[2].item(), detection_as_xyxy[3].item()],
        #             ]
        #         )
        #         scores = np.array(
        #             [detection_as_xyxy[4].item(), detection_as_xyxy[4].item()]
        #         )
        #         norfair_detections.append(
        #             Detection(
        #                 points=bbox, scores=scores, label=int(detection_as_xyxy[-1].item())
        #             )
        #         )

        return norfair_detections    

    def check_intersection(self, box1, box2):

        """Check if two bounding boxes intersect.
        
        Args:
            box1: List or tuple of four integers [x1, y1, x2, y2] for the first bounding box.
            box2: List or tuple of four integers [x1, y1, x2, y2] for the second bounding box.
            
        Returns:
            bool: True if the boxes intersect, False otherwise.
        """
        x1_min, y1_min, x1_max, y1_max = box1
        x2_min, y2_min, x2_max, y2_max = box2
        
        # Check if there is an intersection
        if x1_min < x2_max and x1_max > x2_min and y1_min < y2_max and y1_max > y2_min:
            return True
        return False

    def get_area(self, box):
        width = box[2] - box[0]
        height = box[3] - box[1]
        return width * height
    
    def get_percent_area_under_threshold (self, box, image_source, threshold):
        x_min, y_min, x_max, y_max = box
        x = range(int (x_min), int(x_max))
        y = range(int(y_min), int(y_max))
        sum = 0
        for i in y:
            for j in x:
                if image_source[i][j] < threshold:
                    sum+=1
        area = self.get_area(box)
        return sum/area*100

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
            DetectSiloArray, "/silo/objects/raw", 1)
        
        self._pub_ann = self.create_lifecycle_publisher(
            Image, "/silo/annotated_img", 1)
        

        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.realsense_cam_model = PinholeCameraModel()

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
            RGBD,
            "/camera2/rgbd",
            self.image_cb,
            self.image_qos_profile
        )

        self.tracker = Tracker( 
            distance_function="euclidean",#norfair_setup["distance_function"],
            distance_threshold=48,#norfair_setup["distance_threshold"],
            # hit_counter_max=15,#norfair_setup["hit_counter_max"],
            initialization_delay=3,#norfair_setup["initialization_delay"],
            # pointwise_hit_counter_max=4,#norfair_setup["pointwise_hit_counter_max"],
            # detection_threshold=10,#norfair_setup["detection_threshold"],
            past_detections_length=5,#norfair_setup["past_detections_length"],
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

    def image_cb(self, image: RGBD) -> None:

        if self.enable:
            try:
                self.cam_transform = self.tf_buffer.lookup_transform('map', 'camera2_link', rclpy.time.Time(), rclpy.duration.Duration(nanoseconds=1000))
            #     # self.get_logger().info('Got transform from {} to {}: {}'.format(
            #     # self.cam_transform.header.frame_id, self.cam_transform.child_frame_id,
            #     # self.cam_transform.transform))
            except tf2_ros.LookupException as e:
                self.get_logger().error('Failed to lookup transform: {}'.format(e))
                return 
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().error('Failed to extrapolate transform: {}'.format(e))
                return 
            # convert image + predict
            self.realsense_cam_model.fromCameraInfo(image.rgb_camera_info)
            cv_rgb_image = self.cv_bridge.imgmsg_to_cv2(image.rgb, desired_encoding='bgr8')
            cv_depth_image = self.cv_bridge.imgmsg_to_cv2(image.depth, desired_encoding="passthrough")

            results = self.yolo.track(
                source=cv_rgb_image,
                verbose=False,
                stream=False,
                conf=0.5,
                persist=False,
                device=self.device,
                iou=0.5,
                imgsz=640
            )
            arr = ["blue","purple","red","silo"] #define detection label
            ann_results: Results = results[0].cpu()
            if ann_results.boxes:                
                poses = self.parse_transformation(ann_results, cv_depth_image)
                informations = self.parse_information(ann_results)
                detectedobj = self.yolo_detections_to_norfair_detections( ann_results, track_points="bbox")
                tracked_object = self.tracker.update( detections = detectedobj )
                norfair.draw_points( cv_rgb_image, tracked_object )
            
            silo_array = DetectSiloArray()

            for i in range(len(ann_results)):
                silo_msg = DetectSilo()
                # silo_msg.ball = informations[i]["class_name"]
                if informations[i]["class_name"] == "silo": continue
                silo_msg.pose = poses[i]
                self.get_logger().info(f'{silo_msg.pose}')
                silo_array.detections.append(silo_msg)

            self._pub.publish(silo_array)
            newmsg = self.cv_bridge.cv2_to_imgmsg(cv_rgb_image)
            self._pub_ann.publish(newmsg)
            del results
            del cv_rgb_image


def main():
    rclpy.init()
    node = SiloDepthCameraNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()