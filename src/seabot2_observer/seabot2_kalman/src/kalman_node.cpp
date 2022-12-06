#include "seabot2_kalman/kalman_node.hpp"
#include <algorithm>    // std::sort

using namespace std::placeholders;

KalmanNode::KalmanNode()
        : Node("kalman_node"), k_(){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Kalman_node] Start Ok");
}

void KalmanNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<double>("physics_rho", k_.physics_rho_);
    this->declare_parameter<double>("physics_g", k_.physics_g_);
    this->declare_parameter<double>("robot_mass", k_.robot_mass_);
    this->declare_parameter<double>("robot_diameter", k_.robot_diameter_);
    this->declare_parameter<double>("screw_thread", k_.screw_thread_);
    this->declare_parameter<double>("tick_per_turn", k_.tick_per_turn_);
    this->declare_parameter<double>("piston_diameter", k_.piston_diameter_);
    this->declare_parameter<double>("piston_max_tick_value", k_.piston_max_tick_);

    this->declare_parameter<double>("enable_kalman_depth", k_.enable_kalman_depth_);
    this->declare_parameter<double>("piston_volume_eq_init", k_.piston_volume_eq_init_);
    this->declare_parameter<double>("init_chi", k_.init_chi_);
    this->declare_parameter<double>("init_chi2", k_.init_chi2_);
    this->declare_parameter<double>("init_cz", k_.init_cz_);
    this->declare_parameter<double>("init_volume_air", k_.init_volume_air_);

    this->declare_parameter<double>("gamma_alpha_velocity", k_.gamma_alpha_velocity_);
    this->declare_parameter<double>("gamma_alpha_depth", k_.gamma_alpha_depth_);
    this->declare_parameter<double>("gamma_alpha_offset", k_.gamma_alpha_offset_);
    this->declare_parameter<double>("gamma_alpha_chi", k_.gamma_alpha_chi_);
    this->declare_parameter<double>("gamma_alpha_chi2", k_.gamma_alpha_chi2_);
    this->declare_parameter<double>("gamma_alpha_cz", k_.gamma_alpha_cz_);
    this->declare_parameter<double>("gamma_alpha_volume_air", k_.gamma_alpha_volume_air_);

    this->declare_parameter<double>("gamma_init_velocity", k_.gamma_init_velocity_);
    this->declare_parameter<double>("gamma_init_depth", k_.gamma_init_depth_);
    this->declare_parameter<double>("gamma_init_offset", k_.gamma_init_offset_);
    this->declare_parameter<double>("gamma_init_chi", k_.gamma_init_chi_);
    this->declare_parameter<double>("gamma_init_chi2", k_.gamma_init_chi2_);
    this->declare_parameter<double>("gamma_init_cz", k_.gamma_init_cz_);
    this->declare_parameter<double>("gamma_init_volume_air", k_.gamma_init_volume_air_);

    this->declare_parameter<double>("gamma_beta_depth", k_.gamma_beta_depth_);

    this->declare_parameter<bool>("enable_volume_air", k_.enable_volume_air_);

    k_.physics_rho_ = this->get_parameter_or("physics_rho", k_.physics_rho_);
    k_.physics_g_ = this->get_parameter_or("physics_g", k_.physics_g_);
    k_.robot_mass_ = this->get_parameter_or("physics_mass", k_.robot_mass_);
    k_.robot_diameter_ = this->get_parameter_or("robot_diameter", k_.robot_diameter_);
    k_.screw_thread_ = this->get_parameter_or("screw_thread", k_.screw_thread_);
    k_.tick_per_turn_ = this->get_parameter_or("tick_per_turn", k_.tick_per_turn_);
    k_.piston_diameter_ = this->get_parameter_or("piston_diameter", k_.piston_diameter_);
    k_.piston_max_tick_ = this->get_parameter_or("piston_max_tick", k_.piston_max_tick_);

    k_.enable_kalman_depth_ = this->get_parameter_or("enable_kalman_depth", k_.enable_kalman_depth_);
    k_.piston_volume_eq_init_ = this->get_parameter_or("piston_volume_eq_init", k_.piston_volume_eq_init_);
    k_.init_chi_ = this->get_parameter_or("init_chi", k_.init_chi_);
    k_.init_chi2_ = this->get_parameter_or("init_chi2", k_.init_chi2_);
    k_.init_volume_air_ = this->get_parameter_or("init_volume_air", k_.init_volume_air_);
    k_.init_cz_ = this->get_parameter_or("init_cz", k_.init_cz_);

    k_.gamma_alpha_velocity_ = this->get_parameter_or("gamma_alpha_velocity", k_.gamma_alpha_velocity_);
    k_.gamma_alpha_depth_ = this->get_parameter_or("gamma_alpha_depth", k_.gamma_alpha_depth_);
    k_.gamma_alpha_offset_ = this->get_parameter_or("gamma_alpha_offset", k_.gamma_alpha_offset_);
    k_.gamma_alpha_chi_ = this->get_parameter_or("gamma_alpha_chi", k_.gamma_alpha_chi_);
    k_.gamma_alpha_chi2_ = this->get_parameter_or("gamma_alpha_chi2", k_.gamma_alpha_chi2_);
    k_.gamma_alpha_cz_ = this->get_parameter_or("gamma_alpha_cz", k_.gamma_alpha_cz_);
    k_.gamma_alpha_volume_air_ = this->get_parameter_or("gamma_alpha_volume_air", k_.gamma_alpha_volume_air_);

    k_.gamma_init_velocity_ = this->get_parameter_or("gamma_init_velocity", k_.gamma_init_velocity_);
    k_.gamma_init_depth_ = this->get_parameter_or("gamma_init_depth", k_.gamma_init_depth_);

    k_.gamma_init_offset_ = this->get_parameter_or("gamma_init_offset", k_.gamma_init_offset_);
    k_.gamma_init_chi_ = this->get_parameter_or("gamma_init_chi", k_.gamma_init_chi_);
    k_.gamma_init_chi2_ = this->get_parameter_or("gamma_init_chi2", k_.gamma_init_chi2_);
    k_.gamma_init_cz_ = this->get_parameter_or("gamma_init_cz", k_.gamma_init_cz_);
    k_.gamma_init_volume_air_ = this->get_parameter_or("gamma_init_volume_air", k_.gamma_init_volume_air_);

    k_.gamma_beta_depth_ = this->get_parameter_or("gamma_beta_depth", k_.gamma_beta_depth_);

    k_.enable_volume_air_ = this->get_parameter_or("enable_volume_air", k_.enable_volume_air_);

    RCLCPP_INFO(this->get_logger(), "[Kalman_node] physics_rho = %f", k_.physics_rho_);
    RCLCPP_INFO(this->get_logger(), "[Kalman_node] gamma_init_volume_air_ = %f", k_.gamma_init_volume_air_);

    k_.init_parameters(this->now());
}

void KalmanNode::state_callback(const seabot2_piston_driver::msg::PistonState &msg){
    k_.set_new_piston_data(msg.position, msg.position_set_point, msg.header.stamp);
}

void KalmanNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    k_.update_pressure(msg.pressure);
    k_.set_new_depth_data(msg.depth, msg.velocity, msg.header.stamp);
    publish_data();
}

void KalmanNode::density_callback(const seabot2_density::msg::Density &msg){
    k_.update_density(msg.density);
}

void KalmanNode::temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg){
    k_.update_temperature(msg.temperature);
}

void KalmanNode::init_interfaces() {
    publisher_kalman_ = this->create_publisher<seabot2_kalman::msg::KalmanState>("kalman", 10);

    subscriber_state_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/piston", 10, std::bind(&KalmanNode::state_callback, this, _1));
    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&KalmanNode::depth_callback, this, _1));

    subscriber_density_ = this->create_subscription<seabot2_density::msg::Density>(
            "/observer/density", 10, std::bind(&KalmanNode::density_callback, this, _1));

    subscriber_temperature_ = this->create_subscription<temperature_tsys01_driver::msg::TemperatureSensorData>(
            "/driver/temperature", 10, std::bind(&KalmanNode::temperature_callback, this, _1));
}


void KalmanNode::publish_data() {
    seabot2_kalman::msg::KalmanState msg;

    const Matrix<double,NB_STATES, 1> x = k_.x_forcast_;
    msg.velocity = x(0);
    msg.depth = x(1);
    msg.offset = x(2);
    msg.chi = x(3);
    msg.chi2 = x(4);
    msg.cz = x(5);
    msg.volume_air = x(6);
    if(k_.pressure_>0.)
        msg.offset_total = x(2)+x(6)*k_.temperature_/k_.pressure_+x(3)*x(1) + x(4)*pow(x(1),2);
    msg.header.stamp = k_.time_last_predict_;

    msg.variance[0] = k_.gamma_forcast_(0,0);
    msg.variance[1] = k_.gamma_forcast_(1,1);
    msg.variance[2] = k_.gamma_forcast_(2,2);
    msg.variance[3] = k_.gamma_forcast_(3,3);
    msg.variance[4] = k_.gamma_forcast_(4,4);
    msg.variance[5] = k_.gamma_forcast_(5,5);
    msg.variance[6] = k_.gamma_forcast_(6,6);

    publisher_kalman_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanNode>());
    rclcpp::shutdown();
    return 0;
}