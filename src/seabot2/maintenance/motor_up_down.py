from std_srvs.srv import Trigger
import os
import rclpy
from rclpy.node import Node

from seabot2_piston_driver.msg import PistonState, PistonSetPoint

class Seabot2Maintenance(Node):

    def __init__(self):
        super().__init__('seabot2maintenance')
        self.publisher_ = self.create_publisher(PistonSetPoint, '/driver/piston_set_point', 10)
        self.subscription = self.create_subscription(PistonState, '/driver/piston', self.listener_callback, 10)
        timer_period = 1./5.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sens = 1
        self.piston_max_value = 2.0e6

    def timer_callback(self):
        msg = PistonSetPoint()

        if self.sens == 1:
            msg.position = int(self.piston_max_value)
            msg.exit = False
        else:
            msg.position = 0
            msg.exit = True

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        if self.sens == 1 and msg.switch_top == True:
            self.sens = 0
        elif self.sens == 0 and (msg.switch_bottom == True or abs(msg.position) < 100.0):
            self.sens = 1

def main():
    rclpy.init()

    seabot2maintenance = Seabot2Maintenance()
    rclpy.spin(seabot2maintenance)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
