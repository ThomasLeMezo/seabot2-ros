#include "seabot2_depth_control/depth_control_node.hpp"
#include <algorithm>    // std::sort
#include <cmath>

using namespace std::placeholders;

DepthControlNode::DepthControlNode()
        : Node("depth_control_node"), dc_(this->now()){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&DepthControlNode::timer_callback, this), callback_group_);

    RCLCPP_INFO(this->get_logger(), "[Depth_control_node] Start Ok");
}

void DepthControlNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", static_cast<int>(loop_dt_.count()));
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<double>("physics_rho", dc_.physics_rho_);
    this->declare_parameter<double>("physics_g", dc_.physics_g_);
    this->declare_parameter<double>("robot_mass", dc_.robot_mass_);
    this->declare_parameter<double>("robot_diameter", dc_.robot_diameter_);
    this->declare_parameter<double>("screw_thread", dc_.screw_thread_);
    this->declare_parameter<double>("tick_per_turn", dc_.tick_per_turn_);
    this->declare_parameter<double>("motor_max_rpm_", dc_.motor_max_rpm_);
    this->declare_parameter<double>("piston_diameter", dc_.piston_diameter_);
    this->declare_parameter<double>("piston_max_tick_value", dc_.piston_max_tick_value_);
    this->declare_parameter<double>("root_regulation", dc_.root_regulation_);
    this->declare_parameter<double>("limit_depth_regulation", dc_.limit_depth_control_);
    this->declare_parameter<double>("flow_piston_sink", dc_.flow_piston_sink_);
    this->declare_parameter<double>("piston_hysteresis", dc_.piston_hysteresis_);
    this->declare_parameter<double>("piston_max_velocity", dc_.flow_max_);
    this->declare_parameter<bool>("hold_depth_enable", dc_.hold_depth_enable_);
    this->declare_parameter<double>("hold_depth_value_enter", dc_.hold_depth_value_enter_);
    this->declare_parameter<double>("hold_depth_value_exit", dc_.hold_depth_value_exit_);
    this->declare_parameter<double>("hold_velocity_enter", dc_.hold_velocity_enter_);
    this->declare_parameter<double>("hold_velocity_exit", dc_.hold_velocity_exit_);
    this->declare_parameter<double>("delta_velocity_lb", dc_.delta_velocity_lb_);
    this->declare_parameter<double>("delta_velocity_ub", dc_.delta_velocity_ub_);
    this->declare_parameter<double>("delta_position_lb", dc_.delta_position_lb_);
    this->declare_parameter<double>("delta_position_ub", dc_.delta_position_ub_);
    this->declare_parameter<bool>("control_filter", dc_.control_filter_);
    this->declare_parameter<double>("piston_flow_security_percentage", dc_.piston_flow_security_percentage_);
    this->declare_parameter<double>("cf_estimation", dc_.Cf_);
    this->declare_parameter<bool>("debug", dc_.debug_);

    dc_.physics_rho_ = this->get_parameter_or("physics_rho", dc_.physics_rho_);
    dc_.physics_g_ = this->get_parameter_or("physics_g", dc_.physics_g_);
    dc_.robot_mass_ = this->get_parameter_or("physics_mass", dc_.robot_mass_);
    dc_.robot_diameter_ = this->get_parameter_or("robot_diameter", dc_.robot_diameter_);
    dc_.screw_thread_ = this->get_parameter_or("screw_thread", dc_.screw_thread_);
    dc_.tick_per_turn_ = this->get_parameter_or("tick_per_turn", dc_.tick_per_turn_);
    dc_.motor_max_rpm_ = this->get_parameter_or("motor_max_rpm_", dc_.motor_max_rpm_);
    dc_.piston_diameter_ = this->get_parameter_or("piston_diameter", dc_.piston_diameter_);
    dc_.piston_max_tick_value_ = this->get_parameter_or("piston_max_tick_value", dc_.piston_max_tick_value_);
    dc_.root_regulation_ = this->get_parameter_or("root_regulation", dc_.root_regulation_);
    dc_.limit_depth_control_ = this->get_parameter_or("limit_depth_control", dc_.limit_depth_control_);
    dc_.flow_piston_sink_ = this->get_parameter_or("flow_piston_sink", dc_.flow_piston_sink_);
    dc_.piston_hysteresis_ = this->get_parameter_or("piston_hysteresis", dc_.piston_hysteresis_);
    dc_.flow_max_ = this->get_parameter_or("piston_max_velocity", dc_.flow_max_);
    dc_.hold_depth_enable_ = this->get_parameter_or("hold_depth_enable", dc_.hold_depth_enable_);
    dc_.hold_depth_value_enter_ = this->get_parameter_or("hold_depth_value_enter", dc_.hold_depth_value_enter_);
    dc_.hold_depth_value_exit_ = this->get_parameter_or("hold_depth_value_exit", dc_.hold_depth_value_exit_);
    dc_.hold_velocity_enter_ = this->get_parameter_or("hold_velocity_enter", dc_.hold_velocity_enter_);
    dc_.hold_velocity_exit_ = this->get_parameter_or("hold_velocity_exit", dc_.hold_velocity_exit_);
    dc_.delta_velocity_lb_ = this->get_parameter_or("delta_velocity_lb", dc_.delta_velocity_lb_);
    dc_.delta_velocity_ub_ = this->get_parameter_or("delta_velocity_ub", dc_.delta_velocity_ub_);
    dc_.delta_position_lb_ = this->get_parameter_or("delta_position_lb", dc_.delta_position_lb_);
    dc_.delta_position_ub_ = this->get_parameter_or("delta_position_ub", dc_.delta_position_ub_);
    dc_.control_filter_ = this->get_parameter_or("control_filter", dc_.control_filter_);
    dc_.piston_flow_security_percentage_ = this->get_parameter_or("piston_flow_security_percentage", dc_.piston_flow_security_percentage_);
    dc_.Cf_ = this->get_parameter_or("cf_estimation", dc_.Cf_);
    dc_.debug_ = this->get_parameter_or("debug", dc_.debug_);

    dc_.update_coeff();
}

void DepthControlNode::kalman_callback(const seabot2_kalman::msg::KalmanState &msg) {
    dc_.update_state(msg.velocity,
                     msg.depth,
                     msg.chi,
                     msg.chi2,
                     msg.cz,
                     msg.offset_total,
                     msg.header.stamp);
}

void DepthControlNode::piston_callback(const seabot2_piston_driver::msg::PistonState &msg){
    dc_.update_piston(msg.position,
                      msg.switch_top,
                      msg.switch_bottom,
                      msg.state,
                      msg.header.stamp);
}

void DepthControlNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    dc_.update_depth(msg.depth, msg.pressure);
}

void DepthControlNode::safety_callback(const seabot2_safety::msg::SafetyStatus &msg){
    dc_.update_safety(!msg.global_safety_valid, msg.limit_depth);
}

void DepthControlNode::waypoint_callback(const seabot2_mission::msg::Waypoint &msg){
    dc_.update_waypoint(msg.depth,
                        msg.limit_velocity,
                        msg.header.stamp,
                        msg.mission_enable);

    // Debug
    seabot2_depth_control::msg::AlphaDebug msg_alpha_debug;
    msg_alpha_debug.approach_velocity = static_cast<float>(dc_.approach_velocity_);
    publisher_alpha_debug_->publish(msg_alpha_debug);
}

void DepthControlNode::init_interfaces() {
    subscriber_kalman_data_ = this->create_subscription<seabot2_kalman::msg::KalmanState>(
            "/observer/kalman", 10, std::bind(&DepthControlNode::kalman_callback, this, _1));
    subscriber_state_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/piston", 10, std::bind(&DepthControlNode::piston_callback, this, _1));
    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&DepthControlNode::depth_callback, this, _1));
    subscriber_temperature_data_ = this->create_subscription<temperature_tsys01_driver::msg::TemperatureSensorData>(
            "/observer/temperature", 10, std::bind(&DepthControlNode::temperature_callback, this, _1));
    subscriber_mission_data_ = this->create_subscription<seabot2_mission::msg::Waypoint>(
            "/mission/waypoint", 10, std::bind(&DepthControlNode::waypoint_callback, this, _1));
    subscriber_safety_data_ = this->create_subscription<seabot2_safety::msg::SafetyStatus>(
            "/safety/safety", 10, std::bind(&DepthControlNode::safety_callback, this, _1));
    subscriber_density_ = this->create_subscription<seabot2_density::msg::Density>(
            "/observer/density", 10, std::bind(&DepthControlNode::density_callback, this, _1));

    publisher_piston_ = this->create_publisher<seabot2_piston_driver::msg::PistonSetPoint>("/driver/piston_set_point", 10);
    publisher_debug_ = this->create_publisher<seabot2_depth_control::msg::DepthControlDebug>("depth_control_debug", 10);
    publisher_alpha_debug_ = this->create_publisher<seabot2_depth_control::msg::AlphaDebug>("alpha_debug", 10);

    service_alpha_computation_ = this->create_service<seabot2_mission::srv::AlphaMission>("alpha_mission",
                                                                           bind(&DepthControlNode::alpha_mission_pre_computation, this, _1, _2, _3));
}

void DepthControlNode::alpha_mission_pre_computation(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<seabot2_mission::srv::AlphaMission::Request> request,
                                      std::shared_ptr<seabot2_mission::srv::AlphaMission::Response> response){
    RCLCPP_INFO(this->get_logger(), "[Depth_control_node] Received velocity computation request");

    velocity_limits_requests_.clear();
    velocity_limits_requests_ = std::vector<float>(request->velocity_limits);
    velocity_limits_computations_ = true;
}

void DepthControlNode::density_callback(const seabot2_density::msg::Density &msg){
    dc_.update_density(msg.density);
}

void DepthControlNode::temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg){
    dc_.update_temperature(msg.temperature);
}

void DepthControlNode::timer_callback() { /// ToDo bug to be check
    if(velocity_limits_computations_){
        velocity_limits_computations_ = false;
        timer_->cancel();
        dc_.regulation_state_ = DepthControl::STATE_COMPUTE_ALPHA;
        publish_message();
        for(auto velocity: velocity_limits_requests_) {
            //publish_message();
            auto result = dc_.alpha_solver_.compute_alpha(velocity);
            RCLCPP_INFO(this->get_logger(), "[Depth_control_node] Velocity was computed for beta = %f, alpha = %f", velocity,
                        result);
        }
        dc_.regulation_state_ = DepthControl::STATE_SURFACE;
        timer_->reset();
    }

    dc_.state_machine_step(loop_dt_, this->now());
    publish_message();

}

void DepthControlNode::publish_message(){
    /// Piston message
    seabot2_piston_driver::msg::PistonSetPoint msg_piston;
    msg_piston.position = round(dc_.piston_set_point_);
    msg_piston.exit = dc_.is_exit_;
    publisher_piston_->publish(msg_piston);

    /// Debug message
    seabot2_depth_control::msg::DepthControlDebug debug_msg_;
    debug_msg_.mode = dc_.regulation_state_;
    debug_msg_.u = dc_.u_debug_;
    debug_msg_.dy = dc_.dy_debug_;
    debug_msg_.y = dc_.y_debug_;
    debug_msg_.piston_set_point = static_cast<float>(dc_.piston_set_point_);
    publisher_debug_->publish(debug_msg_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DepthControlNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}