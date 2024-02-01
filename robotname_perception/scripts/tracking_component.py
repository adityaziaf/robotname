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
        self.tracked_obj_pub = self.create_publisher(dta, '/objects/transformed/tracked',10)
        self.detected_obj_subs = self.create_subscription(dta, '/objects/transformed',self.trackingCallback,10)

        # Norfair tracker initialization
        self.tracker = Tracker(
            distance_function="euclidean",#norfair_setup["distance_function"],
            distance_threshold=0.1,#norfair_setup["distance_threshold"],
            hit_counter_max=15,#norfair_setup["hit_counter_max"],
            # initialization_delay=7.5,#norfair_setup["initialization_delay"],
            # pointwise_hit_counter_max=4,#norfair_setup["pointwise_hit_counter_max"],
            # detection_threshold=0.01,#norfair_setup["detection_threshold"],
            # past_detections_length=4,#norfair_setup["past_detections_length"],
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
                    points=np.array([object.point.x, object.point.y]),
                    scores=np.array([object.score]),
                    label=object.classname,
                )
            )
        
        tracked_objects = self.tracker.update(detections)

        if len(msg.detections) != len(tracked_objects): 
            return

        for idx, detected_object in enumerate(msg.detections):
            detected_object.id = tracked_objects[idx].id

            
        self.tracked_obj_pub.publish(msg)
    

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