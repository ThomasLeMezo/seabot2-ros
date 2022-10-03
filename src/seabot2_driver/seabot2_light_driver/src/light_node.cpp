#include "seabot2_light_driver/light_node.h"

using namespace placeholders;

LightNode::LightNode()
        : Node("light_node"), light_(this){

    init_parameters();
    init_interfaces();

    light_.i2c_open();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&LightNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Light_node] Start Ok");
}

void LightNode::timer_callback() {
    if(light_is_on_){
        if(special_flash_ && (this->now()>time_turn_off_light_)){
            special_flash_ = false;
            light_.set_flash_number(nb_surface_flash_);
        }
        if(!is_surface_) {
            if (light_.set_light_enable(false) == EXIT_SUCCESS)
                light_is_on_ = false;
        }
    }
    else{
        if(is_surface_){
            if (light_.set_light_enable(true) == EXIT_SUCCESS)
                light_is_on_ = true;
        }
    }
}

void LightNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", light_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", light_.getI2CAddr());

    light_.setI2CPeriph(this->get_parameter_or("i2c_periph", light_.getI2CPeriph()));
    light_.setI2CAddr(this->get_parameter_or("i2c_address", light_.getI2CAddr()));

    /// Light
    this->declare_parameter<int>("flash_duration", light_.flash_duration_);
    this->declare_parameter<int>("flash_pause_between_flash", light_.flash_pause_between_flash_);
    this->declare_parameter<int>("flash_pause_end", light_.flash_pause_end_);
    this->declare_parameter<double>("depth_flash_enable", depth_flash_enable_);

    light_.flash_duration_ = this->get_parameter_or("flash_duration", light_.flash_duration_);
    light_.flash_pause_between_flash_ = this->get_parameter_or("flash_pause_between_flash", light_.flash_pause_between_flash_);
    light_.flash_pause_end_ = this->get_parameter_or("flash_pause_end", light_.flash_pause_end_);
    depth_flash_enable_ = this->get_parameter_or("depth_flash_enable", depth_flash_enable_);
}

void LightNode::service_light_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<seabot2_light_driver::srv::Light::Request> request,
                                       std::shared_ptr<seabot2_light_driver::srv::Light::Response> response){
    if(request->flash_enable) {
        RCLCPP_INFO(this->get_logger(), "[Node_light] Received flash order");
        time_turn_off_light_ = this->now() + rclcpp::Duration::from_seconds(request->duration);
        special_flash_ = true;

        light_.set_flash_number(request->number_of_flash);
        if (light_.set_light_enable(true) == EXIT_SUCCESS)
            light_is_on_ = true;
    }
    else{
        if (light_.set_light_enable(false) == EXIT_SUCCESS)
            light_is_on_ = false;

    }
}

void LightNode::service_flash_surface_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                              std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    is_surface_ = request->data;
}

void LightNode::init_interfaces(){
    service_light_ = this->create_service<seabot2_light_driver::srv::Light>("light",
                                                                            std::bind(&LightNode::service_light_callback, this, _1, _2, _3));

    service_flash_surface_ = this->create_service<std_srvs::srv::SetBool>("surface",
                                                                            std::bind(&LightNode::service_flash_surface_callback, this, _1, _2, _3));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightNode>());
    rclcpp::shutdown();
    return 0;
}
