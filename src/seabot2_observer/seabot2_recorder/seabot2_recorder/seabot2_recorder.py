from std_srvs.srv import Trigger
import os
import rclpy
from rclpy.node import Node
import shlex, subprocess, time, signal

class Seabot2Recorder(Node):

    def __init__(self):
        super().__init__('seabot2_recorder')
        self.srv = self.create_service(Trigger, 'restart_bag', self.restart_bag_callback)

        # Ensure we are on the right directory
        home_path = os.path.expanduser('~')
        bag_path = home_path + "/log/"
        if not os.path.exists(bag_path):
            os.makedirs(bag_path)
        os.chdir(bag_path)

        # Start the bag
        self.proc = None
        self.start_bag()

    def start_bag(self):
        # Stop the bag if it is already running
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            self.proc.wait()

        # Start the bag
        command_line = 'ros2 bag record --all --storage mcap'
        args = shlex.split(command_line)
        self.proc = subprocess.Popen(args)

    def restart_bag_callback(self, request, response):
        self.get_logger().info('[seabot2_recorder] Restarting bag')
        self.start_bag()
        response.success = True
        return response

def main():
    rclpy.init()

    seabot2recorder = Seabot2Recorder()
    rclpy.spin(seabot2recorder)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
