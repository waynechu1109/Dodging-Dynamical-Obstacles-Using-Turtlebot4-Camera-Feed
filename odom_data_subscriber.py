import rclpy
import numpy as np
import transforms3d.euler as euler
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

# state_arr = np.zeros((50, 3))
desired_arr = np.zeros((1, 3))
state_arr = np.zeros((1, 3))
diff_arr = np.zeros((1, 3))
# counter = 1                        # record the index

class odom_data_subscriber(Node):

    def __init__(self):
        super().__init__("odom_data_subscriber")
        self.get_logger().info("start!456!")

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
        global counter, state_arr, diff_arr, desired_arr        

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

        diff_arr[0][0] = state_arr[0][0] - desired_arr[0][0]
        diff_arr[0][1] = state_arr[0][1] - desired_arr[0][1]
        diff_arr[0][2] = state_arr[0][2] - desired_arr[0][2]

        print("state: ", state_arr)
        print("diff: ", diff_arr)
        print()

        # counter += 1


def main(args=None):
    global desired_arr, state_arr, diff_arr

    # user input
    desired_arr[0][0] = float(input("desired x: "))
    desired_arr[0][1] = float(input("desierd y: "))
    desired_arr[0][2] = float(input("desired theta: "))

    rclpy.init(args=args)
    Odom_data_subscriber = odom_data_subscriber()
    rclpy.spin(Odom_data_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Odom_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


