#include "seabot2_depth_control/depth_control_node.hpp"
#include <algorithm>    // std::sort

using namespace std::placeholders;

DepthControlNode::DepthControlNode()
        : Node("depth_control_node"){

    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&DepthControlNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Depth_control_node] Start Ok");
}

void DepthControlNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<double>("physics_rho", physics_rho_);
    this->declare_parameter<double>("physics_g", physics_g_);
    this->declare_parameter<double>("physics_mass", physics_mass_);
    this->declare_parameter<double>("robot_diameter", robot_diameter_);
    this->declare_parameter<double>("screw_thread", screw_thread_);
    this->declare_parameter<double>("tick_per_turn", tick_per_turn_);
    this->declare_parameter<double>("piston_diameter", piston_diameter_);
    this->declare_parameter<double>("piston_max_value", piston_max_value_);
    this->declare_parameter<double>("root_regulation", root_regulation_);
    this->declare_parameter<double>("limit_depth_regulation", limit_depth_regulation_);
    this->declare_parameter<double>("flow_piston_sink", flow_piston_sink_);
    this->declare_parameter<double>("piston_hysteresis", piston_hysteresis_);
    this->declare_parameter<double>("piston_max_velocity", piston_max_velocity_);
    this->declare_parameter<bool>("hold_depth_enable", hold_depth_enable_);
    this->declare_parameter<double>("hold_depth_value_enter", hold_depth_value_enter_);
    this->declare_parameter<double>("hold_depth_value_exit", hold_depth_value_exit_);
    this->declare_parameter<double>("hold_velocity_enter", hold_velocity_enter_);
    this->declare_parameter<double>("delta_velocity_lb", delta_velocity_lb_);
    this->declare_parameter<double>("delta_velocity_ub", delta_velocity_ub_);
    this->declare_parameter<double>("delta_position_lb", delta_position_lb_);
    this->declare_parameter<double>("delta_position_ub", delta_position_ub_);


    physics_rho_ = this->get_parameter_or("physics_rho", physics_rho_);
    physics_g_ = this->get_parameter_or("physics_g", physics_g_);
    physics_mass_ = this->get_parameter_or("physics_mass", physics_mass_);
    robot_diameter_ = this->get_parameter_or("robot_diameter", robot_diameter_);
    screw_thread_ = this->get_parameter_or("screw_thread", screw_thread_);
    tick_per_turn_ = this->get_parameter_or("tick_per_turn", tick_per_turn_);
    piston_diameter_ = this->get_parameter_or("piston_diameter", piston_diameter_);
    piston_max_value_ = this->get_parameter_or("piston_max_value", piston_max_value_);
    root_regulation_ = this->get_parameter_or("root_regulation", root_regulation_);
    limit_depth_regulation_ = this->get_parameter_or("limit_depth_regulation", limit_depth_regulation_);
    flow_piston_sink_ = this->get_parameter_or("flow_piston_sink", flow_piston_sink_);
    piston_hysteresis_ = this->get_parameter_or("piston_hysteresis", piston_hysteresis_);
    piston_max_velocity_ = this->get_parameter_or("piston_max_velocity", piston_max_velocity_);
    hold_depth_enable_ = this->get_parameter_or("hold_depth_enable", hold_depth_enable_);
    hold_depth_value_enter_ = this->get_parameter_or("hold_depth_value_enter", hold_depth_value_enter_);
    hold_depth_value_exit_ = this->get_parameter_or("hold_depth_value_exit", hold_depth_value_exit_);
    hold_velocity_enter_ = this->get_parameter_or("hold_velocity_enter", hold_velocity_enter_);
    delta_velocity_lb_ = this->get_parameter_or("delta_velocity_lb", delta_velocity_lb_);
    delta_velocity_ub_ = this->get_parameter_or("delta_velocity_ub", delta_velocity_ub_);
    delta_position_lb_ = this->get_parameter_or("delta_position_lb", delta_position_lb_);
    delta_position_ub_ = this->get_parameter_or("delta_position_ub", delta_position_ub_);

    /// Computed parameters
    Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    coeff_A_ = physics_g_*physics_rho_/physics_mass_;
    coeff_B_ = 0.5*physics_rho_*Cf_/physics_mass_;
}

void DepthControlNode::kalmann_callback(const seabot2_kalmann::msg::KalmannState &msg) {
    x(0) = msg.velocity;
    x(1) = msg.depth;
    x(3) = msg.offset;
    x(4) = msg.chi;
    x(5) = msg.chi2;
    x(6) = msg.cz;
    time_last_kalmann_callback_ = this->now();
}

void DepthControlNode::state_callback(const seabot2_piston_driver::msg::PistonState &msg){
    piston_position_ = msg.position;
    piston_switch_top_ = msg.switch_top;
    piston_switch_bottom_ = msg.switch_bottom;
}

void DepthControlNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    depth_fusion_ = msg.depth;
}

void DepthControlNode::waypoint_callback(const seabot2_mission::msg::Waypoint &msg){
    if(msg.mission_enable)
        depth_set_point_ = msg.depth;
    else
        depth_set_point_ = 0.0;

    limit_velocity_ = msg.limit_velocity;
    approach_velocity_ = msg.approach_velocity;
}

void DepthControlNode::depth_control_emergency(const std::shared_ptr<rmw_request_id_t> request_header,
                                               const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    emergency_ = request->data;
    response->success = true;
}

void DepthControlNode::init_interfaces() {

    subscriber_kalman_data_ = this->create_subscription<seabot2_kalmann::msg::KalmannState>(
            "/observation/kalmann", 10, std::bind(&DepthControlNode::kalmann_callback, this, _1));
    subscriber_state_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/state", 10, std::bind(&DepthControlNode::state_callback, this, _1));
    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observation/depth", 10, std::bind(&DepthControlNode::depth_callback, this, _1));
    subscriber_mission_data_ = this->create_subscription<seabot2_mission::msg::Waypoint>(
            "/mission/waypoint", 10, std::bind(&DepthControlNode::waypoint_callback, this, _1));

    publisher_piston_ = this->create_publisher<std_msgs::msg::Int32>("/driver/piston_set_point", 10);
    publisher_debug_ = this->create_publisher<seabot2_depth_control::msg::DepthControlDebug>("depth_control_debug", 10);

    service_emergency_ = this->create_service<std_srvs::srv::SetBool>("depth_control_emergency",
                                                                      std::bind(&DepthControlNode::depth_control_emergency, this, _1, _2, _3));
}

double DepthControlNode::compute_u(const Matrix<double, NB_STATES, 1> &x, double set_point, double limit_velocity, double approach_velocity){
    const double x1 = x(0);
    const double x2 = x(1);
    const double x3 = x(2);
    const double x4 = x(3);
    const double x5 = x(4);
    const double x6 = x(5);
    const double x7 = x(6);
    const double A = coeff_A_;
    const double B = coeff_B_;
    const double beta = limit_velocity/M_PI_2;
    const double alpha = approach_velocity;
    const double gamma = beta/alpha;

    double e = (set_point-x2)/alpha;
    double y = x1-gamma*atan(e);
    double dx1 = -A*(x3+x4-(x5*x2+x6*pow(x2,2)))-B*x7*abs(x1)*x1;
    double D = 1+pow(e,2);
    double dy = dx1 + gamma*x1/D;

    debug_msg_.y = y;
    debug_msg_.dy = dy;

    return (-2.*root_regulation_*dy+pow(root_regulation_,2)*y+ gamma*(dx1*D+2.*e*pow(x1,2)/pow(alpha,2))/(pow(D,2))-2.*B*x7*abs(x1)*dx1)/A+x1*(x5+2.*x6*x2);
}

double DepthControlNode::optimize_u(std::array<double, 4> &u_tab){
    sort(u_tab.begin(), u_tab.end());
    if(u_tab[0]<0.0 && u_tab[u_tab.size()-1]>0.0) // Case one positive, one negative => do not move
        return 0.0;
    else{ // Else choose the control that minimizes u
        sort(u_tab.begin(), u_tab.end(), [](int i, int j) { return abs(i) < abs(j); });
        return u_tab[0];
    }
}

void DepthControlNode::timer_callback() {

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthControlNode>());
    rclcpp::shutdown();
    return 0;
}