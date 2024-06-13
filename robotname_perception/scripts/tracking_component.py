#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from norfair import Detection, Tracker

from robotname_msgs.msg import DetectionArray as dta
from robotname_msgs.msg import Detection as dt

class trackingComponent(Node):
    def __init__(self):    
        super().__init__('trackingnode')

          # Load parameters
        #norfair_setup = rclpy.get_param("norfair_setup")
        self.tracked_obj_pub = self.create_publisher(dta, '/camera1/objects/tracked',1)
        self.detected_obj_subs = self.create_subscription(dta, '/camera1/objects/raw',self.trackingCallback,1)

        # Norfair tracker initialization
        self.tracker = Tracker(
            distance_function="euclidean",#norfair_setup["distance_function"],
            distance_threshold=0.5,#norfair_setup["distance_threshold"],
            # hit_counter_max=15,#norfair_setup["hit_counter_max"],
            initialization_delay=3,#norfair_setup["initialization_delay"],
            # pointwise_hit_counter_max=4,#norfair_setup["pointwise_hit_counter_max"],
            # detection_threshold=10,#norfair_setup["detection_threshold"],
            past_detections_length=3,#norfair_setup["past_detections_length"],
        )
        

    def trackingCallback(self, msg : dta):
        """tracking callback function

        Args:
            msg (DetectionArray): 
        """
        
        detections = []
        tracked_objects = []

        for object in msg.detections:
            object : dt

            detections.append(
                Detection(
                    points=np.array([object.pose.pose.position.x, object.pose.pose.position.y]),
                    scores=np.array([object.score]),
                    label=object.classname,
                    data=object.pose
                )
            )
        
        tracked_objects = self.tracker.update(detections)

        newobjs = dta()

        for obj in tracked_objects:
            newobj = dt()
            newobj.id = obj.id
            newobj.classname = obj.label
            newobj.pose = obj.last_detection.data
            newobjs.detections.append(newobj)

        self.tracked_obj_pub.publish(newobjs)
    

def main(args=None):
    rclpy.init(args=args)

    yolo_tracker = trackingComponent()

    rclpy.spin(yolo_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
