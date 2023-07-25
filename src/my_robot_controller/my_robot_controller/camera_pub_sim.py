import rclpy
from rclpy.node import Node

from std_msgs.msg import String

x, y, z = 0., 0., 0.

class CamSimPublisher(Node):

    def __init__(self):
        super().__init__('CamSimPublisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global x, y, z
        msg = String()
        msg.data = "[" + "%f" %x + ", " + "%f" %y + ", " + "%f" %z + "]"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        x += 1       # x increasing


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CamSimPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
