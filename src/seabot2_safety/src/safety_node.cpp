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

void SafetyNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    depth_ = msg.depth;
    velocity_ = msg.velocity;
    depth_last_received_ = msg.header.stamp;
}

void SafetyNode::internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg){
    internal_humidity_ = msg.humidity;
    internal_pressure_ = msg.pressure;
    internal_temperature_ = msg.temperature;
    internal_last_received_ = this->now();
}

void SafetyNode::init_interfaces() {

    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&SafetyNode::depth_callback, this, _1));

    subscriber_internal_sensor_filter_ = this->create_subscription<pressure_bme280_driver::msg::Bme280Data>(
            "/observer/sensor_internal", 10, std::bind(&SafetyNode::internal_sensor_callback, this, _1));

}

bool SafetyNode::test_depth(){
    bool is_valid = true;

    if(this->now()-depth_last_received_>depth_no_data_warning)
        is_valid = false;

}

bool SafetyNode::test_internal_data() {

}

bool SafetyNode::test_zero_pressure() {

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

 /// ToDo :
 /// * Zero pressure
 /// * Leak detection
 /// * Over pressure => overdepth
 /// * batteries
 /// * Piston issue (?) / Seafloor
 /// * Flash for surface
 /// * CPU data
 /// * Crash of driver node (piston, depth sensor)

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}