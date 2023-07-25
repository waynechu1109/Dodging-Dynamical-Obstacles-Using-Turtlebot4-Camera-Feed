#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


class test_driver(Node):
    def __init__(self):
        super().__init__('test_driver')
        self.publisher_ = self.create_publisher(Twist, '/redwood/cmd_vel', 10)
        self.get_logger().info("Driver has been started!!")

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        # time.sleep(3)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = test_driver()

    while rclpy.ok():

        # for speed in range (5):
        #     linear = float(speed)/2.0     # let turtlebot slower
        #     angular = float(0)
        #     teleop_node.send_command(linear, angular)
        #     time.sleep(0.5)

        print("sending!!")
        linear = float(0)
        angular = float(-0.05)
        teleop_node.send_command(linear, angular)
        time.sleep(0.5)
        
        # linear = float(0)
        # angular = float(0)
        # teleop_node.send_command(linear, angular)

        # linear = 5.0
        # angular = 0.0

        # Send the Twist command
        # teleop_node.send_command(linear, angular)
        # print("published")
        # time.sleep(0.3)

        # linear = -5.0
        # angular = 0.0

        # Send the Twist command
        # teleop_node.send_command(linear, angular)
        # print("published")
        # time.sleep(0.3)


    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


