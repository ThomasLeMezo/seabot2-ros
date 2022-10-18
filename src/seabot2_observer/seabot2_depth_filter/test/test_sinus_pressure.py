import rclpy
from rclpy.node import Node
import numpy as np

from pressure_ms5803_driver.msg import PressureSensorData


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PressureSensorData, '/driver/pressure_external', 10)
        timer_period = 1./5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PressureSensorData()
        msg.pressure = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)/10.0+2.0+np.random.normal(0,0.01)
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