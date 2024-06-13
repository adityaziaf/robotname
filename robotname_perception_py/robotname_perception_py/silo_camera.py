
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


class SiloCameraNode(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)

        # params
        self.declare_parameter("model", "silo7Jun.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.4)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        self.get_logger().info('SiloCameraNode created')

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

            ann_results: Results = results[0].cpu()
            
            if ann_results.boxes:
                
                detectedobj = self.yolo_detections_to_norfair_detections( ann_results, track_points="bbox")
                tracked_object = self.tracker.update( detections = detectedobj )
                norfair.draw_points( cv_rgb_image, tracked_object )
                silo_boxes = []
                arr = ["blue","purple","red","silo"] #define detection label
                for box in ann_results.boxes: 
                    label = arr[int(box.cls)]
                    # get all silo coordinate
                    if label == "silo": 
                        silo_coor = box.xyxy.tolist()
                        percent_depth = self.get_percent_area_under_threshold(silo_coor[0], cv_depth_image, 3)
                        print(percent_depth)
                        if(percent_depth<2):continue
                        x_min, y_min, x_max, y_max = silo_coor[0]
                        cv2.rectangle(cv_rgb_image, (int(x_min),int(y_min)),(int(x_max),int(y_max)),(0,0,255),1.5)
                        silo_boxes.append(box.xyxy.tolist())
                if len(silo_boxes) == 5:  #check if all silo detected
                    silo_boxes.sort() #sort by x_min
                    data = []
                    index = 1
                    silo_array = DetectSiloArray()
                    for silo_box in silo_boxes:  
                        silo_msg = DetectSilo()
                        silo_msg.number = index
                        balls = []
                        temp_arr = []
                        # get ball that intersection with silo
                        for box in ann_results.boxes:
                            box_coor = box.xyxy.tolist()
                            label = arr[int(box.cls)]
                            if label == "silo": continue
                            area = self.get_area(box_coor[0])
                            if area<500: continue
                            if self.check_intersection(silo_box[0], box_coor[0]):
                                x_min, y_min, x_max, y_max = box_coor[0]
                                cv2.rectangle(cv_rgb_image, (int(x_min),int(y_min)),(int(x_max),int(y_max)),(255,0,0),1)
                                y_position = box_coor[0][1]
                                if label == "blue" or "red":
                                    balls.append((y_position, label))
                        # sort ball by y position
                        balls.sort(reverse=True, key=lambda x:x[0])
                        for y_position, color in balls:
                            silo_msg.ball.append(color)
                            temp_arr.append(color)
                        #if silo not full, fill with null (biar lebih gmpg buat algoritma decisionnya)
                        while len(temp_arr) < 3:
                            temp_arr.append("null")
                            silo_msg.ball.append('null')
                        data.append(temp_arr)
                        silo_array.detections.append(silo_msg)
                        index+=1
                    self.get_logger().info(f'{data}')
                    self._pub.publish(silo_array)
            newmsg = self.cv_bridge.cv2_to_imgmsg(cv_rgb_image)
            self._pub_ann.publish(newmsg)
            del results
            del cv_rgb_image


def main():
    rclpy.init()
    node = SiloCameraNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()