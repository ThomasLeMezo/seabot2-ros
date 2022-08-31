#include "seabot2_piston_driver/piston_node.h"

using std::placeholders::_1;

PistonNode::PistonNode()
        : Node("piston_node"), piston_(this){

    init_parameters();
    init_topics();

    timer_ = this->create_wall_timer(
            200ms, std::bind(&PistonNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Thruster_node] Start Ok");
}

void PistonNode::timer_callback() {
    if(piston_.get_all_data()==EXIT_SUCCESS){
        seabot2_piston_driver::msg::PistonState state_msg;
        state_msg.header.stamp = this->now();

        state_msg.position = piston_.position_;
        state_msg.position_set_point = piston_.position_set_point_;
        state_msg.switch_top = piston_.switch_top_;
        state_msg.switch_bottom = piston_.switch_bottom_;
        state_msg.enable = piston_.enable_;
        state_msg.motor_sens = piston_.motor_sens_;
        state_msg.state = piston_.state_;
        state_msg.motor_speed_set_point = piston_.motor_set_point_;
        state_msg.motor_speed = piston_.motor_cmd_;
        state_msg.battery_voltage = piston_.battery_voltage_;
        state_msg.motor_current = piston_.motor_current_;

        publisher_piston_state_->publish(state_msg);
    }
}

void PistonNode::init_parameters() {
    this->declare_parameter<int>("dt", dt_.count());
    dt_ = std::chrono::microseconds(this->get_parameter_or("dt", dt_.count()));

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", piston_.getI2CPeriph());
    this->declare_parameter<bool>("primary_i2c_address", piston_.getI2CAddr());

    piston_.setI2CPeriph(this->get_parameter_or("i2c_periph", piston_.getI2CPeriph()));
    piston_.setI2CAddr(this->get_parameter_or("i2c_address", piston_.getI2CAddr()));
}

void PistonNode::topic_position_set_point_callback(const std_msgs::msg::Int32 &msg){
    piston_.set_position(msg.data);
}

void PistonNode::init_topics() {
    publisher_piston_state_ = this->create_publisher<seabot2_piston_driver::msg::PistonState>("state", 1);

    subscription_position_set_point_ = this->create_subscription<std_msgs::msg::Int32>(
            "cmd_engine", 10, std::bind(&PistonNode::topic_position_set_point_callback, this, _1));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PistonNode>());
    rclcpp::shutdown();
    return 0;
}
