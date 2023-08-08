#!/usr/bin/env python3

import rclpy
import time
import threading
import numpy as np
import transforms3d.euler as euler
from rclpy.node import Node
import sys

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.append("/home/ubuntu/ros2_ws_4/src/my_robot_controller/my_robot_controller")
# import other function file
import controller as ctr
import create_BezierCurve as bcurve
import calculate_slope
import RRT_star as rrts
import move_next_position

# obstacle information
obstacle_state_arr = [[0., 0.], [0., 0.]]  # in odom frame
#(new)
initial_obstacle_position = [0., 0., 0.]
current_obstacle_position = initial_obstacle_position
obstacle_old_position = [0., 0.]
obstacle_velocity = 0.
obstacle_radius = 10.    # (R)
obstacle_x_velo = obstacle_velocity* np.cos(current_obstacle_position[2])
obstacle_y_velo = obstacle_velocity* np.sin(current_obstacle_position[2])
obstacle_temp_list = [[0., 0., 0.], [0., 0., 0.]]  # list for getting the velo. of obstacle

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

security_width = obstacle_radius + robot_radius # the additional width around the obstacle for safety 

last_callback_time = None

class odom_data_subscriber(Node):

    def __init__(self):
        super().__init__("odom_data_subscriber")

        # publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/arches/cmd_vel', 10)
        self.get_logger().info("start!@@@!")

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

        # subscriber for odom_obstacle_info
        self.subscription_obstacle_info = self.create_subscription(
            Float64MultiArray,
            "/odom_obstacle_info",
            self.odom_obstacle_info_callback,
            10
        )

    def odom_obstacle_info_callback(self, msg):
        global security_width, obstacle_temp_list, last_callback_time, dodge, iteration_since_see
        elapsed_time = 0                    # for getting time duration between each callback
        callback_start_time = time.time() 
        if last_callback_time is not None:
            elapsed_time = callback_start_time - last_callback_time
        last_callback_time = callback_start_time

        # store the old data
        for i in range(3):
            obstacle_temp_list[1][i] = obstacle_temp_list[0][i]  
            #  old one                    #  new one

        # recording data from /odom_obstacle_info as new data
        for j in range(3):
            obstacle_temp_list[0][j] = msg.data[j]

        # predict obstacle trajectory only if robot starts dodging and at particular # of iteration
        if dodge and iteration_since_see%5 == 0:
            rrts.RRTStar.obstacle_list = []   # clear the list every callback
            current_obstacle_position = [msg.data[0], msg.data[1], 0.] # the last element is orientation
            # print('data from odom_obstacle_info', current_obstacle_position)
            rrts.RRTStar.obstacle_list.append([current_obstacle_position[0],
                                    current_obstacle_position[1],
                                    security_width])
            # print('data in obstacle_list:', rrts.RRTStar.obstacle_list)

            # get obstacle velo
            if elapsed_time != 0:
                obstacle_x_velo = (obstacle_temp_list[0][0]-obstacle_temp_list[1][0])/elapsed_time
                obstacle_y_velo = (obstacle_temp_list[1][0]-obstacle_temp_list[1][1])/elapsed_time
                print('obstacle x velo:', obstacle_x_velo, 'obstacle y velo:', obstacle_y_velo)

            # do obstacle trajectory prediction
            predicted_length = 30
            for i in range(predicted_length):
                obstacle_list_x = current_obstacle_position[0]+obstacle_x_velo*i*elapsed_time
                obstacle_list_y = current_obstacle_position[1]+obstacle_x_velo*i*elapsed_time
                rrts.RRTStar.obstacle_list.append([obstacle_list_x, obstacle_list_y, security_width*(1-(i/predicted_length))])


    def odom_callback(self, msg):
        global iteration, state_arr, diff_arr, target, robot_velocity, robot_omega
        global obstacle_state_arr, nearby, see_obstacle, controller_index, dodge
        global iteration_since_see

        # store the old data
        for i in range(3):
            state_arr[1][i] = state_arr[0][i]  
            #  old one         #  new one

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
            # calculate the difference to final distination
            for j in range(3):
                diff_arr[j] = state_arr[0][j] - target[j]
            #### new controller
            robot_velocity, robot_omega = ctr.controller(diff_x=diff_arr[0], diff_y=diff_arr[1], diff_theta=diff_arr[2],
                                                            iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
                                                            dodge=dodge)
            # escape the callback function
            return
        
        if dodge:
            if iteration_since_see%5 == 0:
                # set RRT* starting point and goal
                rrts.RRTStar.start = (state_arr[0][0], state_arr[0][1])
                rrts.RRTStar.goal = (target[0], target[1])
                # RRT*
                path = rrts.RRTStar.do_RRTstar(start=rrts.RRTStar.start,
                                          goal=rrts.RRTStar.goal,
                                          obstacle_list=rrts.RRTStar.obstacle_list,    # obstacle list need the data from camera
                                          x_limit=rrts.RRTStar.x_limit,
                                          y_limit=rrts.RRTStar.y_limit,
                                          step_size=rrts.RRTStar.step_size,
                                          max_iterations=rrts.RRTStar.max_iterations,
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

                # orientation adjustment if needed
                if np.abs(diff_arr[2]) > np.pi/6:
                    self.turn(turning_index=2)

                #### new controller
                robot_velocity, robot_omega = ctr.controller(diff_x=diff_arr[0], diff_y=diff_arr[1], diff_theta=diff_arr[2],
                                                            iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
                                                            dodge=dodge)
            iteration_since_see += 1
                    
        # if the robot is close enough to distination
        if np.sqrt((state_arr[0][0]-target[0])**2 + 
               (state_arr[0][1]-target[1])**2) < 100.: #or smooth_data_index > len(smooth_data):
            dodge = 0
            for j in range(3):
                diff_arr[j] = state_arr[0][j] - target[j]
            #### new controller
            robot_velocity, robot_omega = ctr.controller(diff_x=diff_arr[0], diff_y=diff_arr[1], diff_theta=diff_arr[2],
                                                            iteration=iteration, initial_velocity=robot_velocity, initial_omega=robot_omega,
                                                            dodge=dodge)
            
        print("state: ", state_arr[0])
        print()

        # count iteration only if the robot starts moving
        if np.abs(state_arr[1][0] - state_arr[0][0]) > 0.001 or np.abs(state_arr[1][1] - state_arr[0][1]) > 0.001:
            iteration += 1

    def drive(self):
        # keep publishing the data to cmd_velocity to drive the robot
        while rclpy.ok():
            global robot_velocity, robot_omega, diff_arr
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

    def turn(self, msg=Twist(), turning_index=1):
        # publish "turn around" to cmd_velocity, "turning_index" indicate the type of turn
        # turning_index 1 => turn around
        #               2 => orientation adjustment
        global state_arr, diff_arr
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
            if diff_arr[2] > np.pi/6:
                while (np.abs(state_arr[0][2] - old_angle) > np.pi/6):
                    msg.linear.x = float(0)
                    msg.angular.z = float(-0.05)   # clockwise
                    self.publisher_.publish(msg)
                    time.sleep(0.3)
            elif diff_arr[2] < np.pi/6:
                while (np.abs(state_arr[0][2] - old_angle) > np.pi/6):
                    msg.linear.x = float(0)
                    msg.angular.z = float(0.05)   # counter-clockwise
                    self.publisher_.publish(msg)
                    time.sleep(0.3)


def main(args=None):
    global target, state_arr, diff_arr

    # user input
    target[0] = float(input("desired x: "))
    target[1] = float(input("desierd y: "))
    target[2] = float(input("desired theta: "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()

    driving_thread = threading.Thread(target = Odom_data_subscriber.drive)
    driving_thread.start()

    rclpy.spin(Odom_data_subscriber)

    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()
