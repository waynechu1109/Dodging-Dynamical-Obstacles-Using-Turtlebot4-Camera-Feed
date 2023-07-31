import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import tf2_py
import tf2_msgs.msg
import asyncio
import time
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("start!#!")
        
        # Subscribe to the /tf topic
        self.tf_subscriber = self.create_subscription(
            tf2_msgs.msg.TFMessage,
            '/arches/tf',
            self.tf_callback,
            10)

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

        for transform_stamped in msg.transforms:
            source_frame = transform_stamped.header.frame_id
            target_frame = transform_stamped.child_frame_id
            # print('source:', source_frame)
            # print('child:', target_frame)

            # Example usage:
            if source_frame == 'base_link' and target_frame == 'wheel_drop_right':
                print('source:', source_frame)
                print('child:', target_frame)
                # Do something with this specific transform

                translation = transform_stamped.transform.translation
                rotation = transform_stamped.transform.rotation

                print('translation x:', translation.x)
                print('translation y:', translation.y)
                print('translation z:', translation.z)

                print('rotation x:', rotation.x)
                print('rotation y:', rotation.y)
                print('rotation z:', rotation.z)
                print('rotation w:', rotation.w)

                # convert rotation data into matrix
                r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
                rotation_matrix = r.as_matrix()
                

                # create new ones column
                zero_row = np.zeros((1, 3))
                transform_matrix = np.vstack((rotation_matrix, zero_row))
                
                # create new zero row
                ones_column = np.ones((4, 1))
                transform_matrix = np.hstack((transform_matrix, ones_column))

                transform_matrix[0][3] = translation.x
                transform_matrix[1][3] = translation.y
                transform_matrix[2][3] = translation.z


                print(transform_matrix)
                print()


                # transform_matrix = self.get_transform_matrix(source_frame, target_frame)
                # if transform_matrix is not None:
                #     print('Transformation Matrix:')
                #     print(transform_matrix)

    # def get_transform_matrix(self, source_frame, target_frame):

    #     print('now getting transform matrix...')


    #     # tf_future = self.tf_buffer.wait_for_transform_async(
    #     #     target_frame,
    #     #     source_frame,
    #     #     time=rclpy.time.Time()
    #     # )

    #     # rclpy.spin_until_future_complete(self, tf_future)
    #     # print("Got it!")

    #     # try:
    #     #     transform_stamped = asyncio.run(self.tf_buffer.lookup_transform_async(
    #     #     target_frame,
    #     #     source_frame,
    #     #     rclpy.time.Time()
    #     #     ))
    #     #     translation = transform_stamped.transform.translation
    #     #     rotation = transform_stamped.transform.rotation
    #     #     transform_matrix = tf2_py.transformations.concatenate_matrices(
    #     #         tf2_py.transformations.translation_matrix((translation.x, translation.y, translation.z)),
    #     #         tf2_py.transformations.quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    #     #     )
    #     #     return transform_matrix
    #     # except tf2_ros.LookupException as e:
    #     #     self.get_logger().warn('TF Lookup Exception: %s' % e)
    #     #     return None
    #     # except tf2_ros.ExtrapolationException as e:
    #     #     self.get_logger().warn('TF Extrapolation Exception: %s' % e)
    #     #     return None

    #     # 'lookup_transform', 'lookup_transform_async', 'lookup_transform_core', 'lookup_transform_full', 'lookup_transform_full_async'

    #     print (self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time()))
    #     print('tf_beffer:', self.tf_buffer.lookup_transform, self.tf_buffer.lookup_transform_async, self.tf_buffer.lookup_transform_core, 
    #           self.tf_buffer.lookup_transform_full, self.tf_buffer.lookup_transform_full_async)
    #     quit()

    #     try:
    #         transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
    #         translation = transform_stamped.transform.translation
    #         rotation = transform_stamped.transform.rotation
    #         transform_matrix = tf2_py.transformations.concatenate_matrices(
    #             tf2_py.transformations.translation_matrix((translation.x, translation.y, translation.z)),
    #             tf2_py.transformations.quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    #         )
    #         return transform_matrix
    #     except tf2_ros.LookupException as e:
    #         self.get_logger().warn('TF Lookup Exception: %s' % e)
    #         return None
    #     except tf2_ros.ExtrapolationException as e:
    #         self.get_logger().warn('TF Extrapolation Exception: %s' % e)
    #         return None

def main(args=None):
    rclpy.init(args=args)

    tf_listener_node = TFListenerNode()

    rclpy.spin(tf_listener_node)
    tf_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


