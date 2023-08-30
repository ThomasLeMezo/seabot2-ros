import rclpy
from rclpy.node import Node
from digi.xbee.devices import *

from seabot2_safety.msg import SafetyStatus
from pressure_bme280_driver.msg import Bme280Data
from seabot2_power_driver.msg import PowerState
from gpsd_client.msg import GpsFix
from seabot2_lambert.msg import GnssPose
from seabot2_depth_filter.msg import DepthPose
from seabot2_mission.msg import Waypoint

class XbeeNode(Node):

    def __init__(self):
        super().__init__('xbee_node')
        self.init_interfaces()
        self.init_parameters()

        self.timer = self.create_timer(self.time_between_communication, self.timer_callback)

        # Connect to xbee
        # https://xbplib.readthedocs.io/en/latest/index.html
        self.xbee = XBeeDevice(self.serial_port, self.serial_baudrate)
        self.xbee.open()

        self.configure_xbee()

        self.safety_global_safety_valid = False
        self.safety_published_frequency = False
        self.safety_depth_limit = False
        self.safety_batteries_limit = False
        self.safety_depressurization = False
        self.safety_seafloor = False
        self.safety_piston = False
        self.safety_zero_depth = False

        self.internal_pressure = 0.0
        self.internal_temperature = 0.0
        self.internal_humidity = 0.0

        self.battery = 0.0

        self.valid_fix = False
        self.fix_latitude = 0.0
        self.fix_longitude = 0.0

        self.gnss_heading = 0.0
        self.gnss_speed = 0.0
        self.gnss_mean_east = 0.0
        self.gnss_mean_north = 0.0

        self.depth = 0.0

        self.current_waypoint = 0.0

    def __del__(self):
        self.xbee.close()

    def configure_xbee(self):
        self.xbee.read_device_info()
        self.xbee.set_node_id(self.xbee_node_id)

        # Set encryption enable
        self.xbee.set_parameter("EE",  b'1')
        # Set encryption key
        self.xbee.set_parameter("KY",  bytes(self.xbee_encryption_key, 'utf-8'))
        # Set network id
        self.xbee.set_parameter("ID",  self.xbee_network_id.to_bytes(2, 'big'))

        # Apply changes.
        self.xbee.apply_changes()
        # Write changes (to flash).
        self.xbee.write_changes()

        self.xbee.add_data_received_callback(self.data_received_callback)

    def my_data_received_callback(self, xbee_message):
        # ToDo : process the callback
        address = xbee_message.remote_device.get_64bit_addr()
        data = xbee_message.data.decode("utf8")
        print("Received data from %s: %s" % (address, data))

    def init_parameters(self):
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', '9600')
        self.declare_paramter('xbee_node_id', 'SeabotXX')
        self.declare_parameter('time_between_communication', '5') # in seconds
        self.declare_parameter('xbee_encryption_key', "ABCDEFGHIFKLMNOP") # 16 bytes
        self.declare_parameter('xbee_network_id', 0x42) # between 0x0 and 0x7FFF

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.time_between_communication = self.get_parameter('time_between_communication').get_parameter_value().integer_value
        self.xbee_node_id = self.get_parameter('xbee_node_id').get_parameter_value().string_value
        self.xbee_encryption_key = self.get_parameter('xbee_encryption_key').get_parameter_value().string_value
        self.xbee_network_id = self.get_parameter('xbee_network_id').get_parameter_value().integer_value

    def init_interfaces(self):
        self.subscription_safety_data = self.create_subscription(SafetyStatus, '/safety/safety', self.safety_callback, 10)
        self.subscription_internal_sensor_filter = self.create_subscription(Bme280Data, '/observer/pressure_internal', self.internal_sensor_callback, 10)
        self.subscription_power_data = self.create_subscription(PowerState, '/observer/power', self.power_callback, 10)
        self.subscription_gnss_data = self.create_subscription(GpsFix, '/driver/fix', self.gpsd_callback, 10)
        self.subscription_gnss_pose = self.create_subscription(GnssPose, '/observer/pose_mean', self.gnss_pose_callback, 10)
        self.subscription_depth = self.create_subscription(DepthPose, '/observer/depth', self.depth_callback, 10)
        self.subscription_mission = self.create_subscription(Waypoint, '/mission/waypoint', self.mission_callback, 10)

    def safety_callback(self, msg):
        self.safety_global_safety_valid = msg.global_safety_valid
        self.safety_published_frequency = msg.published_frequency
        self.safety_depth_limit = msg.depth_limit
        self.safety_batteries_limit = msg.batteries_limit
        self.safety_depressurization = msg.depressurization
        self.safety_seafloor = msg.seafloor
        self.safety_piston = msg.piston
        self.safety_zero_depth = msg.zero_depth

    def internal_sensor_callback(self, msg):
        self.internal_pressure = msg.pressure
        self.internal_temperature = msg.temperature
        self.internal_humidity = msg.humidity

    def gpsd_callback(self, msg):
        self.valid_fix = msg.mode > GpsFix.MODE_NO_FIX
        self.fix_latitude = msg.latitude
        self.fix_longitude = msg.longitude

    def power_callback(self, msg):
        self.battery = msg.battery_volt

    def gnss_pose_callback(self, msg):
        self.gnss_heading = msg.heading
        self.gnss_speed = msg.velocity
        self.gnss_mean_east = msg.east
        self.gnss_mean_north = msg.north

    def depth_callback(self, msg):
        self.depth = msg.depth

    def mission_callback(self, msg):
        self.current_waypoint = msg.waypoint_id
    def timer_callback(self):
        # Send data to xbee
        # ToDo : create the frame to be send
        self.xbee.send_data_broadcast("Hello XBee World!")

def main(args=None):
    rclpy.init(args=args)

    xbee_node = XbeeNode()
    rclpy.spin(xbee_node)

    xbee_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()