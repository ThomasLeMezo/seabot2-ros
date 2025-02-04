import rclpy
from rclpy.node import Node
from digi.xbee.devices import *
import socket
import math

from seabot2_msgs.msg import SafetyStatus2
from seabot2_msgs.msg import Bme280Data
from seabot2_msgs.msg import PowerState
from seabot2_msgs.msg import GpsFix
from seabot2_msgs.msg import GnssPose
from seabot2_msgs.msg import DepthPose
from seabot2_msgs.msg import MissionState
from seabot2_msgs.msg import XbeeMessageLogState
from seabot2_msgs.msg import XbeeMessageHeader

def serialize_data(data, val, nb_bit, start_bit, value_min=None, value_max=None, flag_debug=False):
    if flag_debug:
        print("------")
        print("val init =", val)
    if value_min is not None and value_max is not None:
        scale = ((1 << nb_bit) - 1) / (value_max - value_min)
        val = int(round((min(max(val,value_min), value_max) - value_min) * scale))
    else:
        value_min = 0.0
        scale = 1.0
    mask = ((1 << nb_bit) - 1) << start_bit
    data = data | (mask & (val << start_bit))

    if flag_debug:
        print("val_min =", value_min)
        print("scale = ", scale)
        print("val = ", val)
    return data, nb_bit + start_bit, val / scale + value_min

def deserialize_data(data, nb_bit, start_bit, value_min=None, value_max=None):
    mask = (1 << nb_bit) - 1
    # data is byte array, convert to int
    v = (data >> start_bit) & mask
    if value_min is not None and value_max is not None:
        scale = ((1 << nb_bit) - 1) / (value_max - value_min)
        v = v / scale + value_min
    return v, start_bit + nb_bit

class XbeeNode(Node):

    def __init__(self):
        super().__init__('xbee_node')

        # Interfaces
        self.subscription_mission = None
        self.subscription_depth = None
        self.subscription_mission_state = None
        self.subscription_gnss_pose = None
        self.subscription_gnss_data = None
        self.subscription_power_data = None
        self.subscription_internal_sensor_filter = None
        self.subscription_safety_data = None
        self.publisher_log_state = None

        # Get hostname of device
        self.hostname = socket.gethostname()
        # limit size
        self.hostname = self.hostname[:20] if len(self.hostname) >= 21 else self.hostname

        # Test if hostname starts with "seabot"
        self.hostname_is_seabot = True
        if not self.hostname.startswith("seabot"):
            self.hostname_is_seabot = False
        print("Hostname : ", self.hostname)

        # Parameters
        self.xbee_network_id = 0x42
        self.xbee_encryption_key = "ABCDEFGHIFKLMNOP"
        self.xbee_node_id = self.hostname
        self.time_between_communication = 5
        self.serial_baudrate = 9600
        self.tx_power = 4 # 0=2mW, 1=5mW, 2=10mW, 3=16mW, 4=32mW
        self.routing_node = 2

        self.serial_port = "/dev/ttyMAX0" if self.hostname_is_seabot else "/dev/ttyUSB0"

        # Initialization
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

        self.current_waypoint = 0
        self.mission_mode = 0

        self.CMD_MSG_TYPE = {"LOG_STATE": 0, "CMD_SLEEP": 1, "CMD_PARAMETERS": 2, "CMD_MISSION_NEW": 3,
                             "CMD_MISSION_KEEP": 4}
        self.last_cmd_received = 0

    def __del__(self):
        self.xbee.close()

    def configure_xbee(self):
        self.xbee.read_device_info()
        self.xbee.set_node_id(self.xbee_node_id)

        ## MAC/PHY
        # Set network id
        self.get_logger().info(f'Set network ID {self.xbee_network_id}')
        self.xbee.set_parameter('ID', self.xbee_network_id.to_bytes(2, 'big'))

        # Set encryption key
        self.get_logger().info(f'Set encryption key {self.xbee_encryption_key}')
        self.xbee.set_parameter('KY', bytearray(self.xbee_encryption_key, 'utf-8'))

        # Set encryption enable
        self.get_logger().info('Enable Encryption')
        self.xbee.set_parameter('EE', b'\x01')

        # TX Power Level
        self.get_logger().info(f'Set TX power level = {self.tx_power}')
        self.xbee.set_parameter('PL', self.tx_power.to_bytes(2, 'big'))

        ## Set Network parameters
        # CE Routing/Messaging Mode
        self.get_logger().info(f'Set Routing node ({self.routing_node})')
        self.xbee.set_parameter('CE', self.routing_node.to_bytes(2, 'big'))

        # Apply changes.
        self.get_logger().info('Apply changes & write')
        self.xbee.apply_changes()
        # Write changes (to flash).
        self.xbee.write_changes()

        self.xbee.add_data_received_callback(self.data_received_callback)

    def data_received_callback(self, xbee_message):
        # ToDo : process the callback
        node_address = xbee_message.remote_device.get_64bit_addr()
        node_name = xbee_message.remote_device.get_node_id()
        message_timestamp = xbee_message.timestamp
        message_is_broadcast = xbee_message.is_broadcast

        # Test if size of a message is correct
        if 1 < len(xbee_message.data) < 20:
            data = int.from_bytes(xbee_message.data, byteorder='little') # test if little or big endian
            message_type = self.deserialize_log_type(data)
            if message_type == self.CMD_MSG_TYPE["LOG_STATE"]:
                data_decoded = self.deserialize_log_state(data)
                if not self.hostname_is_seabot:
                    self.get_logger().info(f'Received data from {node_name} ({node_address})\n {data_decoded}')
                message_log_state = XbeeMessageLogState()
                message_log_state.xbee_header = XbeeMessageHeader()
                message_log_state.xbee_header.xbee_address = int.from_bytes(node_address, byteorder='big')
                message_log_state.xbee_header.node_name = node_name if node_name else ""
                message_log_state.xbee_header.is_broadcast = message_is_broadcast
                message_log_state.xbee_header.message_type = message_type
                message_log_state.safety_global_safety_valid = True if data_decoded["state"]["global_safety_valid"] == 1 else False
                message_log_state.safety_published_frequency = True if data_decoded["state"]["published_frequency"] == 1 else False
                message_log_state.safety_depth_limit = True if data_decoded["state"]["depth_limit"] == 1 else False
                message_log_state.safety_batteries_limit = True if data_decoded["state"]["batteries_limit"] == 1 else False
                message_log_state.safety_depressurization = True if data_decoded["state"]["depressurization"] == 1 else False
                message_log_state.safety_seafloor = True if data_decoded["state"]["seafloor"] == 1 else False
                message_log_state.safety_piston = True if data_decoded["state"]["piston"] == 1 else False
                message_log_state.safety_zero_depth = True if data_decoded["state"]["zero_depth"] == 1 else False
                message_log_state.fix_latitude = data_decoded["latitude"]
                message_log_state.fix_longitude = data_decoded["longitude"]
                message_log_state.gnss_speed = data_decoded["speed"]
                message_log_state.gnss_heading = data_decoded["heading"]
                message_log_state.battery = data_decoded["battery"]
                message_log_state.internal_pressure = data_decoded["pressure"]
                message_log_state.internal_temperature = data_decoded["temperature"]
                message_log_state.internal_humidity = int(data_decoded["humidity"])
                message_log_state.current_waypoint = data_decoded["waypoint"]
                message_log_state.last_cmd_received = data_decoded["last_cmd_received"]
                self.publisher_log_state.publish(message_log_state)
            else:
                self.get_logger().info(f'Received unknown message {node_name} ({node_address})')

    def init_parameters(self):
        self.declare_parameter('serial_port', self.serial_port)
        self.declare_parameter('serial_baudrate', self.serial_baudrate)
        self.declare_parameter('xbee_node_id', self.xbee_node_id)
        self.declare_parameter('time_between_communication', self.time_between_communication)  # in seconds
        self.declare_parameter('xbee_encryption_key', self.xbee_encryption_key)  # 16 bytes
        self.declare_parameter('xbee_network_id', self.xbee_network_id)  # between 0x0 and 0x7FFF
        self.declare_parameter('tx_power', self.tx_power)
        self.declare_parameter('routing_node', self.routing_node)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.time_between_communication = self.get_parameter(
            'time_between_communication').get_parameter_value().integer_value
        self.xbee_node_id = self.get_parameter('xbee_node_id').get_parameter_value().string_value
        self.xbee_encryption_key = self.get_parameter('xbee_encryption_key').get_parameter_value().string_value
        self.xbee_network_id = self.get_parameter('xbee_network_id').get_parameter_value().integer_value
        self.tx_power = self.get_parameter('tx_power').get_parameter_value().integer_value
        self.routing_node = self.get_parameter('routing_node').get_parameter_value().integer_value

    def init_interfaces(self):
        self.subscription_safety_data = self.create_subscription(SafetyStatus2,
                                                                 '/safety/safety', self.safety_callback, 10)
        self.subscription_internal_sensor_filter = self.create_subscription(Bme280Data, '/observer/pressure_internal',
                                                                            self.internal_sensor_callback, 10)
        self.subscription_power_data = self.create_subscription(PowerState, '/observer/power', self.power_callback, 10)
        self.subscription_gnss_data = self.create_subscription(GpsFix, '/driver/fix', self.gpsd_callback, 10)
        self.subscription_gnss_pose = self.create_subscription(GnssPose,
                                                               '/observer/pose_mean', self.gnss_pose_callback, 10)
        self.subscription_depth = self.create_subscription(DepthPose, '/observer/depth', self.depth_callback, 10)
        self.subscription_mission_state = self.create_subscription(MissionState, '/mission/mission_state',
                                                                    self.mission_callback, 10)

        self.publisher_log_state = self.create_publisher(XbeeMessageLogState, '/com/xbee/log_state', 10)

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
        self.fix_latitude = msg.latitude if not math.isnan(msg.latitude) else 0.
        self.fix_longitude = msg.longitude if not math.isnan(msg.longitude) else 0.

    def power_callback(self, msg):
        self.battery = msg.battery_volt

    def gnss_pose_callback(self, msg):
        self.gnss_heading = msg.heading if not math.isnan(msg.heading) else 0.
        self.gnss_speed = msg.velocity if not math.isnan(msg.velocity) else 0.
        self.gnss_mean_east = msg.east if not math.isnan(msg.east) else 0.
        self.gnss_mean_north = msg.north if not math.isnan(msg.north) else 0.

    def depth_callback(self, msg):
        self.depth = msg.depth

    def mission_callback(self, msg):
        self.current_waypoint = msg.waypoint_id
        self.mission_mode = msg.mode

    def timer_callback(self):
        # Send data to xbee
        if self.depth < 0.5 or self.mission_mode == 0:
            self.xbee.send_data_broadcast(self.serialize_log_state()[0])

    def serialize_log_state(self):
        bit_position = 0
        data = 0b0
        message_type = self.CMD_MSG_TYPE["LOG_STATE"]
        state = 0
        state |= (self.safety_global_safety_valid & 0x1) << 0
        state |= (self.safety_published_frequency & 0x1) << 1
        state |= (self.safety_depth_limit & 0x1) << 2
        state |= (self.safety_batteries_limit & 0x1) << 3
        state |= (self.safety_depressurization & 0x1) << 4
        state |= (self.safety_seafloor & 0x1) << 5
        state |= (self.safety_piston & 0x1) << 6
        state |= (self.safety_zero_depth & 0x1) << 7

        data, bit_position, _ = serialize_data(data, message_type, 4, bit_position)
        data, bit_position, _ = serialize_data(data, self.fix_latitude, 25, bit_position, value_min=-90.0,
                                               value_max=90.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.fix_longitude, 25, bit_position, value_min=-180.0,
                                               value_max=180.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.gnss_speed, 8, bit_position, value_min=0, value_max=5.0,
                                               flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.gnss_heading, 8, bit_position, value_min=0, value_max=359.0,
                                               flag_debug=False)

        data, bit_position, _ = serialize_data(data, state, 8, bit_position, flag_debug=False)

        data, bit_position, _ = serialize_data(data, self.battery, 8, bit_position, value_min=12.0, value_max=16.8,
                                               flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_pressure, 6, bit_position, value_min=680.0,
                                               value_max=800.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_temperature, 6, bit_position, value_min=8.0,
                                               value_max=50.0, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.internal_humidity, 6, bit_position, value_min=50.0,
                                               value_max=100.0, flag_debug=False)

        data, bit_position, _ = serialize_data(data, self.current_waypoint, 8, bit_position, flag_debug=False)
        data, bit_position, _ = serialize_data(data, self.last_cmd_received, 4, bit_position, flag_debug=False)

        return data.to_bytes(int(bit_position / 8), byteorder='little'), message_type

    def deserialize_log_type(self, data):
        bit_position = 0
        message_type, bit_position = deserialize_data(data, 4, bit_position)
        return message_type

    def deserialize_log_state(self, data):
        bit_position = 0
        data_dict = {}
        message_type, bit_position = deserialize_data(data, 4, bit_position)
        data_dict["message_type"] = message_type
        latitude, bit_position = deserialize_data(data, 25, bit_position, value_min=-90.0, value_max=90.0)
        data_dict["latitude"] = latitude
        longitude, bit_position = deserialize_data(data, 25, bit_position, value_min=-180.0, value_max=180.0)
        data_dict["longitude"] = longitude
        speed, bit_position = deserialize_data(data, 8, bit_position, value_min=0, value_max=5.0)
        data_dict["speed"] = speed
        heading, bit_position = deserialize_data(data, 8, bit_position, value_min=0, value_max=359.0)
        data_dict["heading"] = heading
        state, bit_position = deserialize_data(data, 8, bit_position)
        data_dict["state"] = {}
        data_dict["state"]["global_safety_valid"] = (state >> 0) & 0x1
        data_dict["state"]["published_frequency"] = (state >> 1) & 0x1
        data_dict["state"]["depth_limit"] = (state >> 2) & 0x1
        data_dict["state"]["batteries_limit"] = (state >> 3) & 0x1
        data_dict["state"]["depressurization"] = (state >> 4) & 0x1
        data_dict["state"]["seafloor"] = (state >> 5) & 0x1
        data_dict["state"]["piston"] = (state >> 6) & 0x1
        data_dict["state"]["zero_depth"] = (state >> 7) & 0x1
        battery, bit_position = deserialize_data(data, 8, bit_position, value_min=12.0, value_max=16.8)
        data_dict["battery"] = battery
        pressure, bit_position = deserialize_data(data, 6, bit_position, value_min=680.0, value_max=800.0)
        data_dict["pressure"] = pressure
        temperature, bit_position = deserialize_data(data, 6, bit_position, value_min=8.0, value_max=50.0)
        data_dict["temperature"] = temperature
        humidity, bit_position = deserialize_data(data, 6, bit_position, value_min=50.0, value_max=100.0)
        data_dict["humidity"] = humidity
        waypoint, bit_position = deserialize_data(data, 8, bit_position)
        data_dict["waypoint"] = waypoint
        last_cmd_received, bit_position = deserialize_data(data, 4, bit_position)
        data_dict["last_cmd_received"] = last_cmd_received
        return data_dict

def main(args=None):
    rclpy.init(args=args)

    xbee_node = XbeeNode()
    rclpy.spin(xbee_node)

    xbee_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
