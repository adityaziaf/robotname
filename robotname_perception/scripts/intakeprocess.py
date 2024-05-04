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

class intakeBallDetection(Node):

    def __init__(self):
        super().__init__('intakeBallDetection')
        self.publisher_ = self.create_publisher(String, '/intake/detectedcolor', 10)

        self.publisher_img = self.create_publisher(Image, '/intake/imageresult', 10)
        self.subscription = self.create_subscription(
            Image,
            '/intake/image_raw',
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
        current_frame = self.br.imgmsg_to_cv2(msg,"bgr8")
        
        rows = current_frame.shape[0]
        cols = current_frame.shape[1]
        windowsize = rows*cols

        x_min_px    = int(cols*self.x_min_/100)
        y_min_px    = int(rows*self.y_min_/100)
        x_max_px    = int(cols*self.x_max_/100)
        y_max_px    = int(rows*self.y_max_/100)

        img = current_frame[y_min_px:y_max_px, x_min_px:x_max_px]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0, 100, 50])
        upper_red = np.array([10, 255, 255])
        
        # define range of green color in HSV
        lower_green = np.array([120, 50, 50])
        upper_green = np.array([145, 255, 255])
        
        # define range of blue color in HSV
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([110, 255, 255])

        # create a mask for red color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        # create a mask for green color
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        # create a mask for blue color
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # find contours in the red mask
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find contours in the green mask
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find contours in the blue mask
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        msg = String()
        msg.data = 'null'

        # loop through the red contours and draw a rectangle around them
        for cnt in contours_red:
            contour_area = cv2.contourArea(cnt)/windowsize*100
            if contour_area > 10:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(img, 'Red', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                msg.data = 'red'
                
                
                

        # loop through the green contours and draw a rectangle around them
        for cnt in contours_green:
            contour_area = cv2.contourArea(cnt)/windowsize*100
            if contour_area > 10:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, 'Green', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                msg.data = 'purple'
               
                
                

        # loop through the blue contours and draw a rectangle around them
        for cnt in contours_blue:
            contour_area = cv2.contourArea(cnt)/windowsize*100
            if contour_area > 10:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img, 'Blue', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                msg.data = 'blue'
                
                
                
        self.publisher_.publish(msg)
        ros_image_msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_img.publish(ros_image_msg)


    def bgr_to_hsv(self, bgr):
        # Convert the BGR values to a NumPy array
        bgr_array = np.uint8([[bgr]])

        # Convert BGR to HSV
        hsv = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)

        # Extract HSV values
        h, s, v = hsv[0][0]

        return h, s, v
        
        
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


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = intakeBallDetection()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()