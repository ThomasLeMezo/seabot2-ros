#include "seabot2_light_driver/light_node.h"

using namespace placeholders;

LightNode::LightNode()
        : Node("light_node"), light_(this){

    init_parameters();
    init_topics();
    init_services();

    timer_ = this->create_wall_timer(
            500ms, std::bind(&LightNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Light_node] Start Ok");
}

void LightNode::timer_callback() {
    if(light_is_on_ && (this->now()>time_turn_off_light_)){
        if(light_.set_light_enable(false)==EXIT_SUCCESS)
            light_is_on_ = false;
    }
}

void LightNode::init_parameters() {
    this->declare_parameter<int>("dt", dt_.count());
    dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", dt_.count()));

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", light_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", light_.getI2CAddr());

    light_.setI2CPeriph(this->get_parameter_or("i2c_periph", light_.getI2CPeriph()));
    light_.setI2CAddr(this->get_parameter_or("i2c_address", light_.getI2CAddr()));

    /// Light
    this->declare_parameter<int>("flash_duration_", light_.flash_duration_);
    this->declare_parameter<int>("flash_pause_between_flash_", light_.flash_pause_between_flash_);
    this->declare_parameter<int>("flash_pause_end_", light_.flash_pause_end_);

    light_.flash_duration_ = this->get_parameter_or("flash_duration", light_.flash_duration_);
    light_.flash_pause_between_flash_ = this->get_parameter_or("flash_pause_between_flash", light_.flash_pause_between_flash_);
    light_.flash_pause_end_ = this->get_parameter_or("flash_pause_end", light_.flash_pause_end_);
}

void LightNode::init_topics() {

}

void LightNode::service_light_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<seabot2_light_driver::srv::Light::Request> request,
                                       std::shared_ptr<seabot2_light_driver::srv::Light::Response> response){
    time_turn_off_light_ = this->now() + rclcpp::Duration::from_seconds(request->duration);

    light_.set_flash_number(request->number_of_flash);
    if(light_.set_light_enable(true)==EXIT_SUCCESS)
        light_is_on_ = true;

}

void LightNode::init_services(){
    service_light_ = this->create_service<seabot2_light_driver::srv::Light>("light",
                                                                            std::bind(&LightNode::service_light_callback, this, _1, _2, _3));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightNode>());
    rclcpp::shutdown();
    return 0;
}
