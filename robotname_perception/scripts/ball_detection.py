#!/usr/bin/env python3
import cv2
import numpy as np
import math

import rclpy
from rclpy.publisher import Publisher
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

def adjust_image(image, saturation_value, brightness_value, contrast_value):
  
  hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert to HSV

  # Adjust saturation
  h, s, v = cv2.split(hsv_image)
  s = cv2.addWeighted(s, 1.0, np.zeros_like(s), 0.0, saturation_value)

  # Adjust brightness
  v = cv2.addWeighted(v, 1.0, np.ones_like(v), 0.0, brightness_value)

  # Adjust contrast
  v = cv2.convertScaleAbs(v, alpha=contrast_value, beta=0)

  # Merge channels and convert back to BGR
  hsv_image = cv2.merge((h, s, v))
  return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)


def isolate_ball(image,h_min,s_min,v_min,h_max,s_max,v_max):

    if image is None:
      print("Error: Could not read image")
      return None

    # Convert to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define blue Hue range (adjust based on your image)
    lower_blue = np.array([h_min, s_min, v_min])  # Adjust these values as needed
    upper_blue = np.array([h_max, s_max, v_max])  # Adjust these values as needed

    # Create mask for blue color and reasonable saturation/value ranges
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    #mask = cv2.medianBlur(mask,3)
    # crete opening (erode + dilation)
    kernel = np.ones((3, 3), np.uint8)
    # mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    
    #cv2.imshow("mask",mask)
    # Apply mask to isolate blue region
    #blue_region = cv2.bitwise_and(image, image, mask=mask)

    return mask


def f(thing):
    pass

def calculate_circularity(contours,min_circularity_ratio, min_aspect_ratio):
  """
  Calculates the circularity ratio and aspect ratio of a contour.

  Args:
      contour (numpy.ndarray): The contour representing the potential circle.

  Returns:
      tuple: A tuple containing the circularity ratio and aspect ratio.
  """
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
    # rect = cv2.boundingRect(contour)

    # Calculate aspect ratio
    # aspect_ratio = rect.w / rect.h

    if circularity_ratio > min_circularity_ratio:
                objects.append(object)

  return objects

def filter_contours_by_radius(contours, min_radius, max_radius):
    filtered_contours = []
    for contour in contours:
        # Find the minimum enclosing circle for the contour
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        if min_radius <= radius <= max_radius:
            filtered_contours.append(contour)
    return filtered_contours

def rpy_to_quaternion(roll, pitch, yaw):
    # Create a quaternion message
    quaternion = Quaternion()

    # Convert Roll-Pitch-Yaw to Quaternion
    quaternion.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    quaternion.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    quaternion.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    quaternion.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return quaternion

def main(args=None):
    # Create trackbars for saturation, brightness, and contrast
    # Node name (same as ROS1)
    rclpy.init()
    node_name = "my_ros2_node"

    # # Create a node (similar to ROS1 init_node)
    node = rclpy.create_node(node_name)
    
    # # --- Publisher (similar to ROS1 Publisher) ---
    # # Define topic name and message type
    topic_name = "objects"
    msg_type = PoseStamped  # Replace with your message type

    # # Create publisher with QoS options (optional)
    qos = rclpy.qos.QoSProfile(depth=10)  # Adjust QoS settings as needed
    publisher = node.create_publisher(msg_type, topic_name, qos)

    cv2.namedWindow("Image")

    cap = cv2.VideoCapture(0)

    # Check if video capture object is opened successfully
    if not cap.isOpened():
        print("Error: Could not open video capture device")
        return False

    cv2.createTrackbar("Saturation", "Image", 0, 255, f)
    cv2.createTrackbar("Brightness", "Image", 0, 255, f)
    cv2.createTrackbar("Contrast", "Image", 0, 255, f)  # Initial contrast: 1.0
    cv2.createTrackbar("Hue Min", "Image", 0, 255, f)
    cv2.createTrackbar("Sat Min", "Image", 0, 255, f)
    cv2.createTrackbar("Val Min", "Image", 0, 255, f)
    cv2.createTrackbar("Hue Max", "Image", 0, 255, f)
    cv2.createTrackbar("Sat Max", "Image", 0, 255, f)
    cv2.createTrackbar("Val Max", "Image", 0, 255, f)

    cv2.setTrackbarPos("Saturation", "Image", 0)  # Adjust based on preference
    cv2.setTrackbarPos("Brightness", "Image", 0)  # Adjust based on preference
    cv2.setTrackbarPos("Contrast", "Image", 100)  # Maintain initial contrast (1.0)
    cv2.setTrackbarPos("Hue Min", "Image", 0)
    cv2.setTrackbarPos("Sat Min", "Image", 100)
    cv2.setTrackbarPos("Val Min", "Image", 100)
    cv2.setTrackbarPos("Hue Max", "Image", 15)
    cv2.setTrackbarPos("Sat Max", "Image", 255)
    cv2.setTrackbarPos("Val Max", "Image", 255)

    while True:
        # Get current trackbar positions
        ret, frame = cap.read()
        # Check if frame capture was successful
        if not ret:
            print("Error: Could not capture frame from video device")
            break
        frame = cv2.flip(frame, 1)
        saturation_value = cv2.getTrackbarPos("Saturation", "Image")
        brightness_value = cv2.getTrackbarPos("Brightness", "Image")
        contrast_value = cv2.getTrackbarPos("Contrast", "Image") / 100.0  # Convert to float
        h_min = cv2.getTrackbarPos("Hue Min", "Image")
        s_min = cv2.getTrackbarPos("Sat Min", "Image")
        v_min = cv2.getTrackbarPos("Val Min", "Image")
        h_max = cv2.getTrackbarPos("Hue Max", "Image")
        s_max = cv2.getTrackbarPos("Sat Max", "Image")
        v_max = cv2.getTrackbarPos("Val Max", "Image")

        adjusted_image = adjust_image(frame.copy(), saturation_value, brightness_value, contrast_value)
        treshold_image = isolate_ball(adjusted_image,h_min,s_min,v_min,h_max,s_max,v_max)

        contour_list, _ = cv2.findContours(treshold_image, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        
        filtered_contours = filter_contours_by_radius(contour_list, 10.0, 200)
            
        objects = calculate_circularity(filtered_contours, 0.8, 0.9)

        for object in objects:
            pixelx = object[0][0] - 321
            pixely = object[0][1] - 246

            distance = math.hypot(pixelx,pixely)
            theta = math.atan2(pixelx, pixely)

            #real_dist = 0.0001*(distance**2) - 0.0181*distance + 1.0698
            real_dist = 0.0161*distance - 0.8733  - 0.45
            if real_dist < 0.0:
                real_dist = 0.0                                                 
            datainput = PoseStamped()
            datainput.header.frame_id = 'base_link'
            datainput.header.stamp = node.get_clock().now().to_msg()
            datainput.pose.position.x = real_dist
            datainput.pose.orientation = rpy_to_quaternion(0,0,theta + math.pi)

            publisher.publish(datainput)


        cv2.imshow("Adjusted Image", adjusted_image)
        cv2.imshow("Treshold Image", treshold_image)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # rclpy.spin_once(node)

    # Clean up
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
  main()