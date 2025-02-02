#include "seabot2_density/density_node.hpp"

using namespace placeholders;

DensityNode::DensityNode()
        : Node("density_node"), ts(){

    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&DensityNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Density_node] Start Ok");
}

void DensityNode::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));

    this->declare_parameter<double>("physics_salinity", salinity_);
    salinity_ = this->get_parameter_or("physics_salinity", salinity_);
}
//
void DensityNode::temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg){
    temperature_ = msg.temperature;
}

void DensityNode::pressure_callback(const seabot2_msgs::msg::DepthPose &msg){
    sea_pressure_ = msg.pressure;
}

void DensityNode::init_interfaces() {
    publisher_density_ = this->create_publisher<seabot2_msgs::msg::Density>("density", 1);

    subscriber_depth_data_ = this->create_subscription<seabot2_msgs::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&DensityNode::pressure_callback, this, _1));
    subscriber_temperature_data_ = this->create_subscription<seabot2_msgs::msg::TemperatureSensorData>(
            "/observer/temperature", 10, std::bind(&DensityNode::temperature_callback, this, _1));
}

void DensityNode::timer_callback() {
///    %  SA  =  Absolute Salinity                                        [ g/kg ]
///   %  t   =  in-situ temperature (ITS-90)                            [ deg C ]
///    %  p   =  sea pressure                                             [ dbar ]
/// ( i.e. absolute pressure - 10.1325 dbar )

    water_density_ = ts.gsw_rho_t_exact(salinity_, temperature_, sea_pressure_*10.0);
    water_sound_speed_ = ts.gsw_sound_speed_t_exact(salinity_, temperature_, sea_pressure_*10.0);
    seabot2_msgs::msg::Density msg;
    msg.density = water_density_;
    msg.sound_speed = water_sound_speed_;
    msg.header.stamp = this->now();

    publisher_density_->publish(msg);
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DensityNode>());
    rclcpp::shutdown();
    return 0;
}