#include "seabot2_thruster_driver/thruster_node.h"

using std::placeholders::_1;

ThrusterNode::ThrusterNode()
        : Node("thruster_node"), thruster_(this){

    init_parameters();
    init_topics();

    timer_ = this->create_wall_timer(
            200ms, std::bind(&ThrusterNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "[Thruster_node] Start Ok");
}

void ThrusterNode::timer_callback() {
//    if(pressure_sensor_.measure()){ /// Process the measurement
//        pressure_ms5803_driver::msg::PressureSensorData msg;
//        msg.temperature = pressure_sensor_.get_temperature();
//        msg.pressure = pressure_sensor_.get_pression();
//        msg.header.stamp = this->get_clock()->now();
//
//        publisher_sensor_->publish(msg);
//    }
}

void ThrusterNode::init_parameters() {
    this->declare_parameter<int>("dt", dt_.count());
    this->declare_parameter<float>("coeff_cmd_to_pwm", coeff_cmd_to_pwm_);
    this->declare_parameter<int>("delay_stop", delay_stop_.count());
    this->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
    this->declare_parameter<double>("max_linear_velocity_", max_linear_velocity_);
    this->declare_parameter<double>("max_velocity_pwm", max_velocity_pwm_);
    this->declare_parameter<bool>("allow_backward", allow_backward_);
    this->declare_parameter<bool>("reverse_thruster_order", reverse_thruster_order_);
    this->declare_parameter<bool>("reverse_left", reverse_left_);
    this->declare_parameter<bool>("reverse_right", reverse_right_);

    dt_ = std::chrono::microseconds(this->get_parameter_or("dt", dt_.count()));
    this->get_parameter("coeff_cmd_to_pwm", coeff_cmd_to_pwm_);
    delay_stop_ = std::chrono::microseconds(this->get_parameter_or("delay_stop", delay_stop_.count()));
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("max_linear_velocity_", max_linear_velocity_);
    this->get_parameter("max_velocity_pwm", max_velocity_pwm_);
    this->get_parameter("allow_backward", allow_backward_);
    this->get_parameter("reverse_thruster_order", reverse_thruster_order_);
    this->get_parameter("reverse_left", reverse_left_);
    this->get_parameter("reverse_right", reverse_right_);

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", thruster_.getI2CPeriph());
    this->declare_parameter<bool>("primary_i2c_address", thruster_.getI2CAddr());

    thruster_.setI2CPeriph(this->get_parameter_or("i2c_periph", thruster_.getI2CPeriph()));
    thruster_.setI2CAddr(this->get_parameter_or("i2c_address", thruster_.getI2CAddr()));
}

void ThrusterNode::init_topics() {
    publisher_engine_ = this->create_publisher<seabot2_thruster_driver::msg::Engine>("engine", 1);

    subscription_velocity_ = this->create_subscription<seabot2_thruster_driver::msg::Velocity>(
            "topic", 10, std::bind(&ThrusterNode::topic_velocity_callback, this, _1));
    subscription_manual_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "topic", 10, std::bind(&ThrusterNode::topic_manual_velocity_callback, this, _1));
}

void ThrusterNode::topic_velocity_callback(const seabot2_thruster_driver::msg::Velocity &msg) {
    velocity_linear = msg.linear;
    velocity_angular_ = msg.angular;
    velocity_time_last = this->get_clock()->now();
}

void ThrusterNode::topic_manual_velocity_callback(const geometry_msgs::msg::Twist &msg) {
    manual_velocity_linear_ = msg.linear.x;
    manual_velocity_angular_ = msg.angular.z;
    manual_velocity_time_last_ = this->get_clock()->now();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
