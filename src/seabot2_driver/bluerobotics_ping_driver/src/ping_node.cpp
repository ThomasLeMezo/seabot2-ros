#include "bluerobotics_ping_driver/ping_node.h"

#include <ping-device-ping1d.h>
#include <ping-message-all.h>
#include <link/desktop/abstract-link.h>
#include <bluerobotics_ping_driver/msg/profile.hpp>

#include <memory>

using namespace placeholders;

PingNode::PingNode()
        : Node("ping_node"){

    init_parameters();
    init_topics();
    init_services();

    init_driver();

    timer_ = this->create_wall_timer(
            500ms, std::bind(&PingNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Ping_node] Start Ok");
}

void PingNode::init_driver(){
    port_ = AbstractLink::openUrl(uart_port_);
    device_ = std::make_unique<Ping1d>(*port_.get());
    device_->initialize(100);
    RCLCPP_INFO(this->get_logger(), "[Ping_node] Device Type = %ui", device_->device_information.device_type);
    RCLCPP_INFO(this->get_logger(), "[Ping_node] Device Id = %ui", device_->device_id);

    device_->set_mode_auto(true);
    device_->set_speed_of_sound(1550000); /// Set speed of sound
    device_->set_mode_auto(false); /// Set mode
    device_->set_ping_interval(200); /// Set interval
    device_->set_gain_setting(1); /// Set gain
    device_->set_ping_enable(1); /// Set ping enable
}

void PingNode::timer_callback() {
    ping_message *ping_msg = device_->waitMessage(Ping1dId::PROFILE);
    if(ping_msg->msgData != nullptr){
        ping1d_profile profile_msg(*ping_msg);

        bluerobotics_ping_driver::msg::Profile msg;
        msg.confidence = profile_msg.confidence();

        /// ToDo : full msg

    }
}

void PingNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

}

void PingNode::init_topics() {

}

void PingNode::init_services(){

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingNode>());
    rclcpp::shutdown();
    return 0;
}
