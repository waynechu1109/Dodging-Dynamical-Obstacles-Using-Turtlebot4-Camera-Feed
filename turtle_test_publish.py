#!/usr/bin/env python3

# import sys
# sys.path.append('/home/waynechu/ros2_ws/src/my_robot_controller/my_robot_controller')
# import spatial_detextion_nopreview      # import the file that access the camera

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

        import sys
        sys.path.append('/home/waynechu/ros2_ws/src/my_robot_controller/my_robot_controller')
        import spatial_detextion_nopreview      # import the file that access the camera

        print(spatial_detextion_nopreview.data_x)

        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)  # create a timer callback every 1 sec

    def timer_callback(self):
        msg = String()                   # we publish "msg" to the topic
        msg.data = 'this is published msg... %d' % self.counter_
        self.get_logger().info("Hello!" + str(self.counter_))
        self.counter_ += 1                                 


def main(args=None):
    rclpy.init(args=args) # initialize rclpy
    node = turtle_test_publish()     # create node
    rclpy.spin(node)      # node will be kept alive until ^C
    rclpy.shutdown()


################################################################

# if __name__ == '__main__':
#     main()


