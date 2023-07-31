#!/usr/bin/env python3

import rclpy
import time
import threading
import numpy as np
import transforms3d.euler as euler
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# state_arr = np.zeros((50, 3))
desired_arr = np.zeros((1, 3))
state_arr = np.zeros((2, 3))
diff_arr = np.zeros((1, 3))
obstacle_state_arr = [[0., 0.], [0., 0.]]  # in odom frame
velocity = 0.0
omega = 0.0
x_velo = 0.0
y_velo = 0.0
iteration = 0                        # record the index

last_callback_time = None

# controller constants
k1 = 400e-4      # speed
k2 = 5*k1       # trajectory

class odom_data_subscriber(Node):

    def __init__(self):
        super().__init__("odom_data_subscriber")

        # publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/arches/cmd_vel', 10)
        self.get_logger().info("start!###!")

        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # subscriber for odometry
        self.subscription = self.create_subscription(
            Odometry,
            "/arches/odom",
            self.odom_callback,
            qos_profile
        )

        # subscriber for obstacle_info
        self.subscription_obstacle_info = self.create_subscription(
            Float64MultiArray,
            "/obstacle_info",
            self.obstacle_info_callback,
            10
            )

    def obstacle_info_callback(self, msg):
        global state_arr, last_callback_time
        elapsed_time = 0
        callback_start_time = time.time()
        # Calculate elapsed time from the previous callback
        if last_callback_time is not None:
            elapsed_time = callback_start_time - last_callback_time
            # print('Elapsed time since last callback:', elapsed_time)
        last_callback_time = callback_start_time

        # refresh data
        for i in range(2):
            obstacle_state_arr[1][i] = obstacle_state_arr[0][i]

        self.get_logger().info('The data from obstacle_info x, y, z: %f, %f, %f' % (msg.data[0], msg.data[1], msg.data[2]))
        # print('camera frame:', msg.data[0], msg.data[1], msg.data[2])

        # calculate obstacle's position in odom frame
        x = state_arr[0][0]
        y = state_arr[0][1]
        theta = state_arr[0][2]
        x_bar = msg.data[0]
        z_bar = msg.data[2]

        # record data
        obstacle_state_arr[0][0] = x+z_bar*np.cos(theta)+x_bar*np.sin(theta)
        obstacle_state_arr[0][1] = y+z_bar*np.sin(theta)-x_bar*np.cos(theta)

        self.get_logger().info('obstacle in odom: %f, %f' % (obstacle_state_arr[0][0], obstacle_state_arr[0][1]))
        print('velo of obstacle:', self.get_obstacle_velo(elapsed_time))

    def get_obstacle_velo(self, elapsed_time):
        obstacle_x_velo = (obstacle_state_arr[0][0]-obstacle_state_arr[1][0])/elapsed_time
        obstacle_y_velo = (obstacle_state_arr[0][1]-obstacle_state_arr[1][1])/elapsed_time
        obstacle_velo = np.sqrt((obstacle_x_velo)**2 + (obstacle_y_velo)**2)
        return obstacle_velo

    def odom_callback(self, msg):
        global iteration, state_arr, diff_arr, desired_arr, velocity, omega, obstacle_state_arr

        # store the old data
        for i in range(3):
            state_arr[1][i] = state_arr[0][i]  
            #  old one           new one

        # record the data
        state_arr[0][0] = msg.pose.pose.position.x
        state_arr[0][1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w
                     )
        euler_angles = euler.quat2euler(quaternion, 'sxyz')
        state_arr[0][2] = euler_angles[0]  # get the "yaw" data

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
        # get start from zero velocity
        if iteration <= 50:
            velocity = ((iteration+5)/(50+5))*(u2 + z3*u1)
        else:
            velocity = 1*(u2 + z3*u1)   # the linear velocoty of the robot

        print("state: ", state_arr[0])
        # print("obstacle in odom frame (x, y):", obstacle_state_arr[0], obstacle_state_arr[1])
        # print(" velocity: ", velocity , "omega: ", omega)
        print()

        # count iteration only if the robot starts moving
        if np.abs(state_arr[1][0] - state_arr[0][0]) > 0.001 or np.abs(state_arr[1][1] - state_arr[0][1]) > 0.001:
            iteration += 1

    def drive(self):
        while rclpy.ok():
            # print("now sending driving command")
            global x_velo, y_velo, velocity, diff_arr, omega
            # x_velo = velocity*np.cos(diff_arr[0][2])
            # y_velo = velocity*np.sin(diff_arr[0][2])
            msg = Twist()

            if velocity >= 0:
                msg.linear.x = velocity
                # msg.linear.y = y_velo
                msg.angular.z = omega
                # print("published: ", msg.linear.x, msg.angular.z)
                self.publisher_.publish(msg)
                time.sleep(0.3)
            else:
                self.turn_around(msg)

    def turn_around(self, msg):
        global state_arr
        old_angle = state_arr[0][2]
        # print("old angle: ", old_angle)
        while np.abs(state_arr[0][2] - old_angle) < 1.5 or np.abs(state_arr[0][2] - old_angle) > 1.7:
            # print("diff.: ", np.abs(state_arr[0][2] - old_angle))
            msg.linear.x = float(0)
            msg.angular.z = float(0.05)
            self.publisher_.publish(msg)
            time.sleep(0.3)

def main(args=None):
    global desired_arr, state_arr, diff_arr

    # user input
    desired_arr[0][0] = float(input("desired x: "))
    desired_arr[0][1] = float(input("desierd y: "))
    desired_arr[0][2] = float(input("desired theta: "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()

    # driving_thread = threading.Thread(target = Odom_data_subscriber.drive)
    # driving_thread.start()

    rclpy.spin(Odom_data_subscriber)

    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()






# if __name__ == '__main__':
#     main()


