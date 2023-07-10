#!/usr/bin/env python3

import rclpy
import time
import threading
import numpy as np
import transforms3d.euler as euler
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# state_arr = np.zeros((50, 3))
desired_arr = np.zeros((1, 3))
state_arr = np.zeros((2, 3))
diff_arr = np.zeros((1, 3))
update_freq = 20.0  # Hz
velocity = 0.0
omega = 0.0
x_velo = 0.0
y_velo = 0.0
# counter = 1                        # record the index

# constants
k1 = 0.001
k2 = 0.001

class odom_data_subscriber(Node):

    def __init__(self):
        super().__init__("odom_data_subscriber")
        self.publisher_ = self.create_publisher(Twist, '/redwood/cmd_vel', 10)
        self.get_logger().info("start!789!")

        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Odometry,
            "/redwood/odom",
            self.odom_callback,
            qos_profile
        )

    def odom_callback(self, msg):
        global counter, state_arr, diff_arr, desired_arr, update_freq, velocity, omega

        # store the old data
        for i in range(3):
            state_arr[1][i] = state_arr[0][i]

        # record the data
        # state_arr[counter-1][0] = msg.pose.pose.position.x
        # state_arr[counter-1][1] = msg.pose.pose.position.y
        state_arr[0][0] = msg.pose.pose.position.x
        state_arr[0][1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w
                     )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        # print(euler_angles)
        # state_arr[counter-1][2] = euler_angles[0]  # get the "yaw" data
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

        # Print received odometry data
        # self.get_logger().info("Received Odometry: x=%.2f, y=%.2f" % (msg.pose.pose.position.x, msg.pose.pose.position.y))
        # print("theta: ", euler_angles[0])

        # calculate the difference
        for j in range(3):
            diff_arr[0][j] = state_arr[0][j] - desired_arr[0][j]

        z1 = diff_arr[0][2]
        z2 = diff_arr[0][0]*np.cos(diff_arr[0][2]) + diff_arr[0][1]*np.sin(diff_arr[0][2])
        z3 = diff_arr[0][0]*np.sin(diff_arr[0][2]) - diff_arr[0][1]*np.cos(diff_arr[0][2])

        x1 = z1
        x2 = z2
        x3 = -2*z3+z1*z2

        u1 = -k1*x1 + ((k2*x3)/(x1**2+x2**2))*x2
        u2 = -k1*x2 - ((k2*x3)/(x1**2+x2**2))*x1

        omega = u1
        velocity = u2 + z3*u1

        print("state: ", state_arr[0])
        # print("diff: ", diff_arr)
        print(" velocity: ", velocity , "omega: ", omega)
        print()

        # counter += 1

    def drive(self):
        while rclpy.ok():
            # print("now sending driving command")
            global x_velo, y_velo, velocity, diff_arr, omega
            # x_velo = velocity*np.cos(diff_arr[0][2])
            # y_velo = velocity*np.sin(diff_arr[0][2])
            msg = Twist()

            if velocity >= 0:
                msg.linear.x = 0.1* velocity      # need to modify
                # msg.linear.y = y_velo
                msg.angular.z = omega
                self.publisher_.publish(msg)
                time.sleep(0.3)
            else:
                self.turn_around(msg)

    def turn_around(self, msg):
        global state_arr
        old_angle = state_arr[0][2]
        # print("old angle: ", old_angle)
        while np.abs(state_arr[0][2] - old_angle) < 1.5 or np.abs(state_arr[0][2] - old_angle) > 1.7:
            print("diff.: ", np.abs(state_arr[0][2] - old_angle))
            msg.linear.x = float(0)
            msg.angular.z = float(0.05)
            self.publisher_.publish(msg)
            time.sleep(0.5)

def main(args=None):
    global desired_arr, state_arr, diff_arr

    # user input
    desired_arr[0][0] = float(input("desired x: "))
    desired_arr[0][1] = float(input("desierd y: "))
    desired_arr[0][2] = float(input("desired theta: "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()

    driving_thread = threading.Thread(target = Odom_data_subscriber.drive)
    driving_thread.start()

    rclpy.spin(Odom_data_subscriber)

    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

