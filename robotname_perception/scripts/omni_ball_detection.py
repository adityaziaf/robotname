#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # OpenCV library
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

def rpy_to_quaternion(roll, pitch, yaw):
    # Create a quaternion message
    quaternion = Quaternion()

    # Convert Roll-Pitch-Yaw to Quaternion
    quaternion.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    quaternion.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    quaternion.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    quaternion.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return quaternion

class omniBallDetection(Node):

    def __init__(self):
        super().__init__('omniBallDetection')
        self.publisher_ = self.create_publisher(PoseStamped, 'detectedball', 10)

        self.publisher_img = self.create_publisher(Image, 'imageresult', 10)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

        frame_param_descriptor_ = ParameterDescriptor()
        frame_param_descriptor_.description = 'frame value'
        frame_param_descriptor_.type = Parameter.Type.INTEGER.value
        
        range_ = IntegerRange()
        range_.from_value = 0
        range_.to_value = 100
        range_.step = 1

        frame_param_descriptor_.integer_range.append(range_)

        self.declare_parameter("hmin_red",0)
        self.declare_parameter("hmax_red",10)
        self.declare_parameter("hmin_blue",105)
        self.declare_parameter("hmax_blue",115)
        self.declare_parameter("hmin_purple", 121)
        self.declare_parameter("hmax_purple", 130)

        self.declare_parameter("x_min", 0, frame_param_descriptor_)
        self.declare_parameter("y_min", 0, frame_param_descriptor_)
        self.declare_parameter("x_max", 100, frame_param_descriptor_)
        self.declare_parameter("y_max", 100, frame_param_descriptor_)
        
        self.hmin_red_ = self.get_parameter('hmin_red').value
        self.hmin_blue_ = self.get_parameter('hmin_blue').value
        self.hmin_purple_ = self.get_parameter('hmin_purple').value
        self.hmax_red_ = self.get_parameter('hmax_red').value
        self.hmax_blue_ = self.get_parameter('hmax_blue').value
        self.hmax_purple_ = self.get_parameter('hmax_purple').value

        self.x_min_ = self.get_parameter('x_min').value
        self.x_max_ = self.get_parameter('x_max').value
        self.y_min_ = self.get_parameter('y_min').value
        self.y_max_ = self.get_parameter('y_max').value

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.br = CvBridge()

    def camera_callback(self, msg):
        img = self.br.imgmsg_to_cv2(msg,"bgr8")

        blurred_image = cv2.GaussianBlur(img, (5, 5), 0)

        hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([20, 255, 255])
        
        # create a mask for red color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        kernel = np.ones((3, 3), np.uint8)
        #mask = cv2.dilate(mask_red, kernel, iterations=2)
        mask = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        contour_list = sorted(contours_red, key=cv2.contourArea, reverse=True)

        filtered_contours = self.filter_contours_by_radius(contour_list, 20.0, 200)

        objects = self.calculate_circularity(filtered_contours, 0.7, 1.2)

        if objects is not None:
            for object in objects:
                
                pixelx = int(object[0][0] - 321)
                pixely = int(object[0][1] - 246)
                
                distance = math.hypot(pixelx,pixely)
                theta = math.atan2(pixelx, pixely)

                #real_dist = 0.0001*(distance**2) - 0.0181*distance + 1.0698
                real_dist = 0.0161*distance - 0.8733  - 0.45
                if real_dist < 0.0:
                    real_dist = 0.0      

                datainput = PoseStamped()
                datainput.header.frame_id = 'base_link'
                datainput.header.stamp = self.get_clock().now().to_msg()
                datainput.pose.position.x = real_dist
                datainput.pose.orientation = rpy_to_quaternion(0,0,theta + math.pi)

                self.publisher_.publish(datainput)     

                # Draw the circle on the image
                cv2.circle(mask_red, (int(object[0][0]),int(object[0][1])), int(object[1]), 255, 1)

                break 
                
        ros_image_msg = self.br.cv2_to_imgmsg(mask_red, encoding="mono8")
        self.publisher_img.publish(ros_image_msg)
        

    def parameters_callback(self, params):
        # do some actions, validate parameters, update class attributes, etc.
        for param in params:
            if param.name == "x_min":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.x_min_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "y_min":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.y_min_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "x_max":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.x_max_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "y_max":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.y_max_ = param.value
                else:
                    return SetParametersResult(successful=False)
                
            if param.name == "hmin_red":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmin_red_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "hmax_red":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmax_red_ = param.value
                else:
                    return SetParametersResult(successful=False)
            
            if param.name == "hmin_blue":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmin_blue_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "hmax_blue":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmax_blue_ = param.value
                else:
                    return SetParametersResult(successful=False)
                
            if param.name == "hmin_purple":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmin_purple_ = param.value
                else:
                    return SetParametersResult(successful=False)
            if param.name == "hmax_purple":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    self.hmax_purple_ = param.value
                else:
                    return SetParametersResult(successful=False)
                
                
        return SetParametersResult(successful=True)



    def calculate_circularity(self, contours,min_circularity_ratio, max_aspect_ratio):

        objects = []
        for contour in contours:

            # Find Minimum Enclosing Circle (MEC)
            object = cv2.minEnclosingCircle(contour)

            # Calculate MEC area
            area_mec = np.pi * object[1]**2

            # Calculate contour area
            area_contour = cv2.contourArea(contour)

            # Calculate circularity ratio
            circularity_ratio = 4 * np.pi * area_contour / (area_mec * np.pi)

            # Calculate bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate aspect ratio
            aspect_ratio = float(w) / h

            if circularity_ratio > min_circularity_ratio and aspect_ratio < max_aspect_ratio:
                        objects.append(object)

        return objects

    def filter_contours_by_radius(self, contours, min_radius, max_radius):
        filtered_contours = []
        for contour in contours:
            # Find the minimum enclosing circle for the contour
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if min_radius <= radius <= max_radius:
                filtered_contours.append(contour)
        return filtered_contours

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = omniBallDetection()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()