#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
import argparse

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class turtle_test_publish(Node):

    def __init__(self):                  # this is constructor 

        super().__init__("turtle_test_publish")   # node name, super() give access to 
                                         # methods and properties of a parent or sibling class.
        self.cmd_data_publisher_ = self.create_publisher(String, 'camera_data',10)
        self.get_logger().info("Data publisher has been started")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)  # create a timer callback every 1 sec

    def timer_callback(self):
        msg = String()                   # we publish "msg" to the topic
        msg.data = 'this is published msg... %d' % self.counter_
        self.get_logger().info("Hello!" + str(self.counter_))
        self.counter_ += 1                                 


def main(args=None):
    rclpy.init(args=args) # initialize rclpy
    ##(node)##
    node = turtle_test_publish()     # create node
    rclpy.spin(node)      # node will be kept alive until ^C
    rclpy.shutdown()


################################################################

# labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
#             "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# nnPathDefault = str((Path(__file__).parent / Path('../models/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
# parser = argparse.ArgumentParser()
# parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
# parser.add_argument('-ff', '--full_frame', action="store_true", help="Perform tracking on full RGB frame", default=False)

# args = parser.parse_args()

# fullFrameTracking = args.full_frame

# # Create pipeline
# pipeline = dai.Pipeline()

# # Define sources and outputs
# camRgb = pipeline.create(dai.node.ColorCamera)
# spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
# monoLeft = pipeline.create(dai.node.MonoCamera)
# monoRight = pipeline.create(dai.node.MonoCamera)
# stereo = pipeline.create(dai.node.StereoDepth)
# objectTracker = pipeline.create(dai.node.ObjectTracker)

# xoutRgb = pipeline.create(dai.node.XLinkOut)
# trackerOut = pipeline.create(dai.node.XLinkOut)

# xoutRgb.setStreamName("preview")
# trackerOut.setStreamName("tracklets")

# # Properties
# camRgb.setPreviewSize(300, 300)
# camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
# camRgb.setInterleaved(False)
# camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# monoLeft.setCamera("left")
# monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# monoRight.setCamera("right")


























if __name__ == '__main__':
    main()


