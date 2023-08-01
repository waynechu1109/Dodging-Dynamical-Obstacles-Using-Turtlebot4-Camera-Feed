import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import tf2_py
import tf2_msgs.msg
import asyncio
import time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("start!@@!")
        
        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to the /odom topic
        self.tf_subscriber = self.create_subscription(
            Odometry,
            '/arches/odom',
            self.tf_callback,
            qos_profile)

    # def tf_callback(self, msg):
    #     # Wait for 2 seconds to give tf_buffer some time to fill up
    #     self.get_logger().info("Received TF data. Waiting for 2 seconds to process.")
    #     self.create_timer(2.0, lambda: self.process_tf_data(msg))

    def tf_callback(self, msg):
        # Process the TF data here
        # You can access the transformations using msg.transforms

        # Wait for 2 seconds to give tf_buffer some time to fill up
        self.get_logger().info("Received TF data. Waiting for 2 seconds to process.")
        time.sleep(2.0)    

        # for stamp in msg:
        #     source_frame = stamp.header.frame_id
        #     target_frame = stamp.child_frame_id
        #     # print('source:', source_frame)
        #     # print('child:', target_frame)

        #     # Example usage:
        #     if source_frame == 'odom' and target_frame == 'base_link':
        #         print('source:', source_frame)
        #         print('child:', target_frame)
        #         # Do something with this specific transform

        #         translation = stamp.transform.translation
        #         rotation = stamp.transform.rotation

        print('source: odom')
        print('child: base_link')


        print('translation x:', msg.pose.pose.position.x)
        print('translation y:', msg.pose.pose.position.y)
        print('translation z:', msg.pose.pose.position.z)

        print('rotation x:', msg.pose.pose.orientation.x)
        print('rotation y:', msg.pose.pose.orientation.y)
        print('rotation z:', msg.pose.pose.orientation.z)
        print('rotation w:', msg.pose.pose.orientation.w)

        # convert rotation data into matrix
        r = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rotation_matrix = r.as_matrix()
        

        # create new ones column
        zero_row = np.zeros((1, 3))
        transform_matrix = np.vstack((rotation_matrix, zero_row))
        
        # create new zero row
        ones_column = np.ones((4, 1))
        transform_matrix = np.hstack((transform_matrix, ones_column))

        transform_matrix[0][3] = msg.pose.pose.position.x
        transform_matrix[1][3] = msg.pose.pose.position.y
        transform_matrix[2][3] = msg.pose.pose.position.z


        print(transform_matrix)
        print()


def main(args=None):
    rclpy.init(args=args)

    tf_listener_node = TFListenerNode()

    rclpy.spin(tf_listener_node)
    tf_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


