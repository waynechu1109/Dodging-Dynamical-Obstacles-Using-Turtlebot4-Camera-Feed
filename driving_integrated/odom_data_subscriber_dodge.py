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

# import other function file
import controller as ctr
import create_BezierCurve as bcurve
import calculate_slope
import RRT_star as rrtSetting
import do_RRTstar as rrtstar
import move_next_position

# obstacle information
obstacle_state_arr = [[0., 0.], [0., 0.]]  # in odom frame
#(new)
initial_obstacle_position = [0., 0., 0.]
current_obstacle_position = initial_obstacle_position
obstacle_old_position = [0., 0.]
obstacle_velocity = 5.
obstacle_radius = 10.    # (R)
obstacle_x_velo = obstacle_velocity* np.cos(current_obstacle_position[2])
obstacle_y_velo = obstacle_velocity* np.sin(current_obstacle_position[2])

# robot information
robot_radius = 17.    # cm (r)
# initial_robot_position = [249., 250., 1.25*np.pi]    # [x, y, theta]
# current_robot_position = initial_robot_position
robot_velocity = 0.0
robot_omega = 0.0
robot_x_velo = 0.0
robot_y_velo = 0.0
see_obstacle = 0

temp_target = [0., 0., 0.]
target = [0., 0., 0.]
state_arr = np.zeros((2, 3))  # robot state
diff_arr = [0., 0., 0.]
controller_index = 0

safety_margin = 10*robot_radius

dodge = 0
nearby = 0

iteration = 0                        # record the index
iteration_since_see = 0              # number of iteration since robot saw obstacle

last_callback_time = None

# # RRT* parameters
# start = (0., 0.)
# goal = (0., 0.)
# obstacle_list = []  # Format: (x, y, radius), the position after robot see it
# x_limit = (0., size)    # need the size of the map
# y_limit = (0., size)    # need the size of the map
# step_size = 5.0
# max_iterations = 5000
# # RRT* parameters

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
        # self.subscription_obstacle_info = self.create_subscription(
        #     Float64MultiArray,
        #     "/obstacle_info",
        #     self.obstacle_info_callback,
        #     10
        #     )

    def odom_callback(self, msg):
        global iteration, state_arr, diff_arr, target, robot_velocity, robot_omega
        global obstacle_state_arr, nearby, see_obstacle, controller_index, dodge

        # store the old data
        for i in range(3):
            state_arr[1][i] = state_arr[0][i]  
            #  old one         #  new one

        # check if robot near obstacle, if true:
        if np.sqrt((current_obstacle_position[0]-state_arr[0][0])**2+
               (current_obstacle_position[1]-state_arr[0][1])**2
               ) <= obstacle_radius+robot_radius+safety_margin and dodge == 0:
            print("iteration:", iteration+1, "obstacle nearby!!")
            print('RRT* Path Planning...')
            nearby = 1
            see_obstacle = 1
            controller_index = 0
            dodge = 1
            robot_velocity = 0
            robot_omega = 0

        else:
            # The control to final distination
            # recording data from /odom as new data
            state_arr[0][0] = msg.pose.pose.position.x
            state_arr[0][1] = msg.pose.pose.position.y
            quaternion = (msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w
                        )
            euler_angles = euler.quat2euler(quaternion, 'sxyz')
            state_arr[0][2] = euler_angles[0]  # get the "yaw" data

            # calculate the difference to final distination
            for j in range(3):
                diff_arr[0][j] = state_arr[0][j] - target[0][j]
            #### new controller
            robot_velocity, robot_omega = ctr.controller(diff_x=diff_arr[0][0], diff_y=diff_arr[0][1], diff_theta=diff_arr[0][2],
                                                            iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega)
            # escape the callback function
            return
        
        if dodge:
            if iteration_since_see%5 == 0:
                # set RRT* starting point and goal
                rrtSetting.start = (state_arr[0][0], state_arr[0][1])
                rrtSetting.goal = (target[0], target[1])
                # RRT*
                path = rrtstar.do_RRTstar(start=rrtSetting.start,
                                          goal=rrtSetting.goal,
                                          obstacle_list=rrtSetting.obstacle_list,
                                          x_limit=rrtSetting.x_limit,
                                          y_limit=rrtSetting.y_limit,
                                          step_size=rrtSetting.step_size,
                                          max_iterations=rrtSetting.max_iterations,
                                          current_robot_position=state_arr
                                          )
                
                # Create Bezier Curve
                smooth_data, smooth_theta, x_smooth_reversed, y_smooth_reversed = bcurve.create_BezierCurve(path)
                controller_index = 0
                smooth_data_index = 150      # record which point is temporary target
                smooth_data_increase = smooth_data_index

                # get the difference
                for j in range(2):
                    diff_arr[j] = state_arr[0][j] - smooth_data[smooth_data_index][j]
                diff_arr[2] = state_arr[0][2] - smooth_theta[smooth_data_index]

                # path = rrtstar.do_RRTstar(start=start, goal=goal, obstacle_list= ,)


        print("state: ", state_arr[0])
        print()

        # count iteration only if the robot starts moving
        if np.abs(state_arr[1][0] - state_arr[0][0]) > 0.001 or np.abs(state_arr[1][1] - state_arr[0][1]) > 0.001:
            iteration += 1

    def drive(self):
        # keep publishing the data to cmd_velocity to drive the robot
        while rclpy.ok():
            global robot_velocity, robot_omega
            msg = Twist()

            if robot_velocity >= 0:
                msg.linear.x = robot_velocity
                # msg.linear.y = robot_y_velo
                msg.angular.z = robot_omega
                # print("published: ", msg.linear.x, msg.angular.z)
                self.publisher_.publish(msg)
                time.sleep(0.3)
            else:
                self.turn(msg, turning_index=1)

    # publish "turn around" to cmd_velocity, "turning_index" indicate the type of turn
    # turning_index 1 => turn around
    #               2 => orientation adjustment
    def turn(self, msg, turning_index):
        global state_arr
        old_angle = state_arr[0][2]
        # print("old angle: ", old_angle)
        if turning_index == 1:
            while np.abs(state_arr[0][2] - old_angle) < 1.5 or np.abs(state_arr[0][2] - old_angle) > 1.7:
                # print("diff.: ", np.abs(state_arr[0][2] - old_angle))
                msg.linear.x = float(0)
                msg.angular.z = float(0.05)
                self.publisher_.publish(msg)
                time.sleep(0.3)
            return
        elif turning_index == 2:
            pass

def main(args=None):
    global target, state_arr, diff_arr

    # user input
    target[0][0] = float(input("desired x: "))
    target[0][1] = float(input("desierd y: "))
    target[0][2] = float(input("desired theta: "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()

    driving_thread = threading.Thread(target = Odom_data_subscriber.drive)
    driving_thread.start()

    rclpy.spin(Odom_data_subscriber)

    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()



























# if __name__ == '__main__':
#     main()



    # def obstacle_info_callback(self, msg):
    #     global state_arr, last_callback_time
    #     elapsed_time = 0
    #     callback_start_time = time.time()
    #     # Calculate elapsed time from the previous callback
    #     if last_callback_time is not None:
    #         elapsed_time = callback_start_time - last_callback_time
    #         # print('Elapsed time since last callback:', elapsed_time)
    #     last_callback_time = callback_start_time

    #     # refresh data
    #     for i in range(2):
    #         obstacle_state_arr[1][i] = obstacle_state_arr[0][i]

    #     self.get_logger().info('The data from obstacle_info x, y, z: %f, %f, %f' % (msg.data[0], msg.data[1], msg.data[2]))
    #     # print('camera frame:', msg.data[0], msg.data[1], msg.data[2])

    #     # calculate obstacle's position in odom frame
    #     x = state_arr[0][0]
    #     y = state_arr[0][1]
    #     theta = state_arr[0][2]
    #     x_bar = msg.data[0]
    #     z_bar = msg.data[2]

    #     # record data
    #     obstacle_state_arr[0][0] = x+z_bar*np.cos(theta)+x_bar*np.sin(theta)
    #     obstacle_state_arr[0][1] = y+z_bar*np.sin(theta)-x_bar*np.cos(theta)

    #     self.get_logger().info('obstacle in odom: %f, %f' % (obstacle_state_arr[0][0], obstacle_state_arr[0][1]))
    #     print('velo of obstacle:', self.get_obstacle_velo(elapsed_time))

    # def get_obstacle_velo(self, elapsed_time):
    #     obstacle_x_velo = (obstacle_state_arr[0][0]-obstacle_state_arr[1][0])/elapsed_time
    #     obstacle_y_velo = (obstacle_state_arr[0][1]-obstacle_state_arr[1][1])/elapsed_time
    #     obstacle_velo = np.sqrt((obstacle_x_velo)**2 + (obstacle_y_velo)**2)
    #     return obstacle_velo