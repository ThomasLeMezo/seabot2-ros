#include "bluerobotics_ping_driver/ping_node.h"

#include <ping-device-ping1d.h>
#include <ping-message-all.h>
#include <link/desktop/abstract-link.h>

#include <memory>

using namespace std::placeholders;

PingNode::PingNode()
        : Node("ping_node"){

    init_parameters();
    init_interfaces();

    init_driver();

    RCLCPP_INFO(this->get_logger(), "[Ping_node] Start Ok");
}

PingNode::~PingNode(){
    if(enable_ping_){
        device_->set_ping_enable(false); /// Diseable ping

        auto ping_msg = ping1d_continuous_stop();
        ping_msg.set_id(static_cast<uint16_t>(PingMessageId::PING1D_PROFILE));
        device_->writeMessage(ping_msg);
        device_->set_mode_auto(false);
    }
}

#include "link/desktop/serial-link.h"

void PingNode::init_driver(){
    RCLCPP_INFO(this->get_logger(), "[Ping_node] Init driver");
    port_ = std::make_shared<SerialLink>(uart_port_, uart_baudrate_);
    device_ = std::make_unique<Ping1d>(*port_);

    RCLCPP_INFO(this->get_logger(), "[Ping_node] Device Id = %ui", device_->device_id);
    RCLCPP_INFO(this->get_logger(), "[Ping_node device_type = %ui", device_->device_information.device_type);
    RCLCPP_INFO(this->get_logger(), "[Ping_node device_revision = %ui", device_->device_information.device_revision);
    RCLCPP_INFO(this->get_logger(), "[Ping_node firmware_version_major = %ui", device_->device_information.firmware_version_major);
    RCLCPP_INFO(this->get_logger(), "[Ping_node firmware_version_minor = %ui", device_->device_information.firmware_version_minor);
    RCLCPP_INFO(this->get_logger(), "[Ping_node firmware_version_patch = %ui", device_->device_information.firmware_version_patch);

    device_->set_mode_auto(false); /// Set mode
    device_->set_speed_of_sound(static_cast<int>(round(speed_of_sound_*1e3))); /// Set speed of sound
    device_->set_ping_interval(ping_interval_); /// Set interval
    device_->set_gain_setting(gain_setting_); /// Set gain

    device_->set_mode_auto(mode_auto_);
    device_->set_ping_enable(enable_ping_); /// Set ping enable

    if(enable_ping_) {
        /// Ask for continuous stream of data
        auto ping_msg = ping1d_continuous_start();
        ping_msg.set_id(static_cast<uint16_t>(PingMessageId::PING1D_PROFILE));
        device_->writeMessage(ping_msg);
    }

    RCLCPP_INFO(this->get_logger(), "[Ping_node] Device configured");
}

void PingNode::wait_message() const {
    if(enable_ping_) {
        if (const ping_message *ping_msg = device_->waitMessage(Ping1dId::PROFILE);
            ping_msg !=nullptr && ping_msg->msgData != nullptr) {
            //RCLCPP_INFO(this->get_logger(),"[Ping_mode] Msg received");
            const ping1d_profile profile_msg(*ping_msg);

            seabot2_msgs::msg::Profile msg;
            msg.header.stamp = this->now();
            msg.confidence = profile_msg.confidence();

            msg.distance = profile_msg.distance();
            msg.confidence = profile_msg.confidence();
            msg.transmit_duration = profile_msg.transmit_duration();
            msg.ping_number = profile_msg.ping_number();
            msg.scan_start = profile_msg.scan_start();
            msg.scan_length = profile_msg.scan_length();
            msg.gain_setting = profile_msg.gain_setting();
            msg.profile_data_length = profile_msg.profile_data_length();
            msg.profile_data = std::vector<uint8_t>(profile_msg.profile_data(), profile_msg.profile_data()+profile_msg.profile_data_length());

            publisher_profile_->publish(msg);
        }
    }
}

void PingNode::init_parameters() {

    this->declare_parameter<std::string>("serial_port", uart_port_);
    this->declare_parameter<int>("serial_baudrate", uart_baudrate_);
    this->declare_parameter<bool>("mode_auto", mode_auto_);
    this->declare_parameter<double>("speed_of_sound", speed_of_sound_);
    this->declare_parameter<int>("ping_interval", ping_interval_);
    this->declare_parameter<int>("gain_setting", gain_setting_);
    this->declare_parameter<bool>("enable_ping", enable_ping_);

    uart_port_ = this->get_parameter_or("serial_port", uart_port_);
    uart_baudrate_ = this->get_parameter_or("serial_baudrate", uart_baudrate_);
    mode_auto_ = this->get_parameter_or("mode_auto", mode_auto_);
    speed_of_sound_ = this->get_parameter_or("default_speed_of_sound", speed_of_sound_);
    ping_interval_ = this->get_parameter_or("ping_interval", ping_interval_);
    gain_setting_ = this->get_parameter_or("gain_setting", gain_setting_);
    enable_ping_ = this->get_parameter_or("enable_ping", enable_ping_);
}

void PingNode::sound_speed_callback(const seabot2_msgs::msg::Density &msg){
    if(msg.sound_speed != speed_of_sound_){
        speed_of_sound_ = msg.sound_speed;
        device_->set_speed_of_sound(static_cast<int>(round(speed_of_sound_*1e3))); /// Set speed of sound
    }
}

void PingNode::ping_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response){

    enable_ping_ = request->data;
    device_->set_ping_enable(enable_ping_); /// Set ping enable
    if(enable_ping_) {
        auto ping_msg = ping1d_continuous_start();
        ping_msg.set_id(static_cast<uint16_t>(PingMessageId::PING1D_PROFILE));
        device_->writeMessage(ping_msg);
    }
    else{
        auto ping_msg = ping1d_continuous_stop();
        ping_msg.set_id(static_cast<uint16_t>(PingMessageId::PING1D_PROFILE));
        device_->writeMessage(ping_msg);
    }
}

void PingNode::init_interfaces() {
    publisher_profile_ = this->create_publisher<seabot2_msgs::msg::Profile>("profile", 1);

    service_ping_enable_ = this->create_service<std_srvs::srv::SetBool>("ping_enable",
                                                                       std::bind(&PingNode::ping_enable_callback, this,
                                                                                 std::placeholders::_1,
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));
    subscriber_density_ = this->create_subscription<seabot2_msgs::msg::Density>(
            "/observer/density", 10, std::bind(&PingNode::sound_speed_callback, this, std::placeholders::_1));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PingNode>();
    while (rclcpp::ok()){
        node->wait_message();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
