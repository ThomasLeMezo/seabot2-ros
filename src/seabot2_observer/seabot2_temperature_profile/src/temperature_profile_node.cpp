#include "seabot2_temperature_profile/temperature_profile_node.hpp"

using namespace std::placeholders;

TemperatureProfileNode::TemperatureProfileNode()
        : Node("temperature_profile_node"), t_(){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Temperature_profile_node] Start Ok");
}

void TemperatureProfileNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void TemperatureProfileNode::depth_callback(const seabot2_msgs::msg::DepthPose &msg){
    depth_ = msg.depth;
}
void TemperatureProfileNode::temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg){
    t_.update_temperature(msg.temperature, depth_);
    t_.compute_profile();

    seabot2_msgs::msg::TemperatureProfile temperature_profile_msg;
    temperature_profile_msg.header.stamp = this->now();
    temperature_profile_msg.profile_slope = t_.profile_slope_;
    temperature_profile_msg.profile_intercept = t_.profile_intercept_;
    publisher_temperature_profile_->publish(temperature_profile_msg);
}

void TemperatureProfileNode::init_interfaces() {
    publisher_temperature_profile_ = this->create_publisher<seabot2_msgs::msg::TemperatureProfile>("temperature_profile", 10);

    subscriber_depth_data_ = this->create_subscription<seabot2_msgs::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&TemperatureProfileNode::depth_callback, this, _1));

    subscriber_temperature_ = this->create_subscription<seabot2_msgs::msg::TemperatureSensorData>(
            "/driver/temperature", 10, std::bind(&TemperatureProfileNode::temperature_callback, this, _1));
}

int main(const int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureProfileNode>());
    rclcpp::shutdown();
    return 0;
}