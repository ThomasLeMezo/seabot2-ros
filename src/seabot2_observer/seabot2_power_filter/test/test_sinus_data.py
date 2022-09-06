import rclpy
from rclpy.node import Node
import numpy as np

from seabot2_power_driver.msg import PowerState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PowerState, '/driver/power', 10)
        timer_period = 1./5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PowerState()
        msg.battery_volt = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*1.0 + 16.0 +np.random.normal(0,0.1)
        msg.motor_current = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*0.5 + 1.0 +np.random.normal(0,0.1)
        for i in range(2):
            msg.esc_current[i] = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*0.5 + 1.0 + np.random.normal(0,0.1)
        for i in range(4):
            msg.cell_volt[i] = np.sin(0.5*self.get_clock().now().nanoseconds/1e9)*0.5 + 3.5 + np.random.normal(0,0.1)
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