#include "seabot2_power_driver/power_node.h"

using namespace placeholders;

PowerNode::PowerNode()
        : Node("power_node"), power_(this){

    init_parameters();
    init_topics();
    init_services();

    power_.i2c_open();

    timer_ = this->create_wall_timer(
            500ms, std::bind(&PowerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Power_node] Start Ok");
}

void PowerNode::timer_callback() {
    if(power_.get_all_data()==EXIT_SUCCESS){
        seabot2_power_driver::msg::PowerState state_msg;
        state_msg.batteries_volt = power_.batteries_volt_;
        state_msg.esc_current = power_.esc_current_;
        state_msg.motor_current = power_.motor_current_;

        publisher_power_state_->publish(state_msg);
    }
}

void PowerNode::init_parameters() {
    this->declare_parameter<int>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", power_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", power_.getI2CAddr());

    power_.setI2CPeriph(this->get_parameter_or("i2c_periph", power_.getI2CPeriph()));
    power_.setI2CAddr(this->get_parameter_or("i2c_address", power_.getI2CAddr()));
}

void PowerNode::init_topics() {
    publisher_power_state_ = this->create_publisher<seabot2_power_driver::msg::PowerState>("power", 1);
}

void PowerNode::init_services(){

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerNode>());
    rclcpp::shutdown();
    return 0;
}
