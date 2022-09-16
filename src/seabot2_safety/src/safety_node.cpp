#include "seabot2_safety/safety_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

SafetyNode::SafetyNode()
        : Node("safety_node"){

    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&SafetyNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Safety_node] Start Ok");
}

void SafetyNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

}

void SafetyNode::init_interfaces() {

//    service_safety_reload_ = this->create_service<std_srvs::srv::Trigger>("safety_reload",
//                                                                            std::bind(&SafetyNode::service_safety_reload_callback, this, _1, _2, _3));
//
//    service_safety_enable_ = this->create_service<std_srvs::srv::SetBool>("safety_enable",
//                                                                           std::bind(&SafetyNode::service_safety_enable_callback, this, _1, _2, _3));
//
//    publisher_waypoint_ = this->create_publisher<seabot2_safety::msg::Waypoint>("waypoint", 10);
//
//    client_light_ = this->create_client<seabot2_light_driver::srv::Light>("/driver/light");
}

void SafetyNode::timer_callback() {
//    seabot2_safety::msg::Waypoint wp_msg;
//    bool is_new_waypoint = safety_.compute_command(wp_msg);
//
//    if(!safety_enable_) /// Check if safety was disabled by service
//        wp_msg.safety_enable = false;
//
//    publisher_waypoint_->publish(wp_msg);
//
//    if(is_new_waypoint)
//        call_light();

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}