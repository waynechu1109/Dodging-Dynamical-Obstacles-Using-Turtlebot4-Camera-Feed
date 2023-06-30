#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import argparse

import rclpy
from rclpy.node import Node
import threading
import time

from std_msgs.msg import String

raw_data = [0, 0]
with open("/home/parallels/ros2_ws/src/my_robot_controller/my_robot_controller/data.txt", "w") as file:
    file.write("")  # Overwrite the file with an empty string

class turtle_test_publish(Node):

    def __init__(self):                  # this is constructor 

        super().__init__("turtle_test_publish")   # node name, super() give access to 
                                         # methods and properties of a parent or sibling class.
        self.publisher_ = self.create_publisher(String, 'chatter',10)
        timer_period = 1  # seconds
        self.get_logger().info("Data publisher has been started")
        self.counter_ = 0
        self.create_timer(timer_period, self.timer_callback)  # create a timer callback every 1 sec

    def timer_callback(self):
        global raw_data
        msg = String()                   # we publish "msg" to the topic
        # msg.data = 'this is published msg... %d' % self.counter_
        msg.data = str(raw_data[1])
        self.publisher_.publish(msg)
        # self.get_logger().info("Hello!" + str(self.counter_))
        self.counter_ += 1                                 


def start_camera(args=None):
    import sys
    sys.path.append('/home/parallels/ros2_ws/src/my_robot_controller/my_robot_controller')
    import spatial_detextion_nopreview


def publish(args=None):
    time.sleep(6)  # wait 6 seconds for the camera to be ready
    counter = 0
    while rclpy.ok():
        global raw_data  # Use the global variable within the method
        raw_data[1] = raw_data[0]
        # print("now in node...")
        print(raw_data[1])
        
        # print("in node ...")
        with open("/home/parallels/ros2_ws/src/my_robot_controller/my_robot_controller/data.txt", "r") as file:
            lines = file.readlines()

        # raw_data = get_data()
        # print("get data with line: ", lines)
        time.sleep(1)  # seconds

        raw_data[0] = lines[counter]
        counter += 1



def main(args=None):
    rclpy.init(args=args) # initialize rclpy
    node = turtle_test_publish()     # create node

    publish_thread = threading.Thread(target = publish)
    camera_thread = threading.Thread(target = start_camera)
    publish_thread.start()
    camera_thread.start()

    rclpy.spin(node)      # node will be kept alive until ^C
    rclpy.shutdown()


################################################################

# if __name__ == '__main__':
#     main()


