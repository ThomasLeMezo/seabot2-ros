import rclpy
from rclpy.node import Node
import numpy as np

from gpsd_client.msg import GpsFix


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(GpsFix, '/driver/fix', 10)
        timer_period = 1./2.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = GpsFix()
        msg.status = 2
        msg.latitude = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*0.001 + 48.3897
        msg.longitude = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*0.001 + -4.48333
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()