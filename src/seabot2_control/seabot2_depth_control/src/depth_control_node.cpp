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
    this->declare_parameter<double>("robot_mass", robot_mass_);
    this->declare_parameter<double>("robot_diameter", robot_diameter_);
    this->declare_parameter<double>("screw_thread", screw_thread_);
    this->declare_parameter<double>("tick_per_turn", tick_per_turn_);
    this->declare_parameter<double>("motor_max_rpm_", motor_max_rpm_);
    this->declare_parameter<double>("piston_diameter", piston_diameter_);
    this->declare_parameter<double>("piston_max_tick_value", piston_max_tick_value_);
    this->declare_parameter<double>("root_regulation", root_regulation_);
    this->declare_parameter<double>("limit_depth_regulation", limit_depth_control_);
    this->declare_parameter<double>("flow_piston_sink", flow_piston_sink_);
    this->declare_parameter<double>("piston_hysteresis", piston_hysteresis_);
    this->declare_parameter<double>("piston_max_velocity", flow_max_);
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
    robot_mass_ = this->get_parameter_or("physics_mass", robot_mass_);
    robot_diameter_ = this->get_parameter_or("robot_diameter", robot_diameter_);
    screw_thread_ = this->get_parameter_or("screw_thread", screw_thread_);
    tick_per_turn_ = this->get_parameter_or("tick_per_turn", tick_per_turn_);
    motor_max_rpm_ = this->get_parameter_or("motor_max_rpm_", motor_max_rpm_);
    piston_diameter_ = this->get_parameter_or("piston_diameter", piston_diameter_);
    piston_max_tick_value_ = this->get_parameter_or("piston_max_tick_value", piston_max_tick_value_);
    root_regulation_ = this->get_parameter_or("root_regulation", root_regulation_);
    limit_depth_control_ = this->get_parameter_or("limit_depth_control", limit_depth_control_);
    flow_piston_sink_ = this->get_parameter_or("flow_piston_sink", flow_piston_sink_);
    piston_hysteresis_ = this->get_parameter_or("piston_hysteresis", piston_hysteresis_);
    flow_max_ = this->get_parameter_or("piston_max_velocity", flow_max_);
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
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * Cf_ / (2.0 * robot_mass_);
    flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_;
}

void DepthControlNode::kalman_callback(const seabot2_kalman::msg::KalmanState &msg) {
    x(0) = msg.velocity;
    x(1) = msg.depth;
    x(3) = msg.offset;
    x(4) = msg.chi;
    x(5) = msg.chi2;
    x(6) = msg.cz;
    time_last_kalman_callback_ = this->now();
}

void DepthControlNode::state_callback(const seabot2_piston_driver::msg::PistonState &msg){
    piston_position_ = msg.position;
    piston_switch_top_ = msg.switch_top;
    piston_switch_bottom_ = msg.switch_bottom;
    piston_state_ = msg.state;
    x(2) = -piston_position_*tick_to_volume_;
    time_last_piston_callback_ = this->now();
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

    subscriber_kalman_data_ = this->create_subscription<seabot2_kalman::msg::KalmanState>(
            "/observer/kalman", 10, std::bind(&DepthControlNode::kalman_callback, this, _1));
    subscriber_state_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/state", 10, std::bind(&DepthControlNode::state_callback, this, _1));
    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&DepthControlNode::depth_callback, this, _1));
    subscriber_mission_data_ = this->create_subscription<seabot2_mission::msg::Waypoint>(
            "/mission/waypoint", 10, std::bind(&DepthControlNode::waypoint_callback, this, _1));

    publisher_piston_ = this->create_publisher<std_msgs::msg::Int32>("/driver/piston_set_point", 10);
    publisher_debug_ = this->create_publisher<seabot2_depth_control::msg::DepthControlDebug>("depth_control_debug", 10);

    service_emergency_ = this->create_service<std_srvs::srv::SetBool>("depth_control_emergency",
                                                                      std::bind(&DepthControlNode::depth_control_emergency, this, _1, _2, _3));
}

double DepthControlNode::compute_u(const Matrix<double, NB_STATES, 1> &x, double set_point, double limit_velocity, double approach_velocity){
    const double x1 = x(0); /// dz
    const double x2 = x(1); /// z
    const double x3 = x(2); /// piston volume
    const double x4 = x(3); /// offset
    const double x5 = x(4); /// chi1
    const double x6 = x(5); /// chi2
    const double x7 = x(6); /// Cf
    const double A = coeff_A_;
    const double B = coeff_B_;
    const double beta = limit_velocity;
    const double alpha = approach_velocity;

    double e = alpha*(set_point-x2);
    double de = -alpha*x1;
    double T = 1.0 - pow(tanh(e), 2);
    double dde = -alpha*beta*de*T;
    double dT = -2.*de*tanh(e)*T;
    double dx1 = -A*(x3+x4-(x5*x2+x6*pow(x2,2)))-B*x7*abs(x1)*x1;

    double y = x1-beta*tanh(e);
    double dy = dx1 - beta*de*T;
    double s = root_regulation_;

    debug_msg_.y = y;
    debug_msg_.dy = dy;

    double u = (1./A)* (-2.*s*dy+pow(s,2)*y-beta*(dde*T+de*dT)-2*B*x7*abs(x1)*dx1)+x1*(x5+2.*x6*x2);
    return u;
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

    double u = 0.;
    double dt = (std::chrono::duration<double>(loop_dt_)).count();

    /// Analyze specific cases
    if(emergency_)
        regulation_state_ = STATE_SURFACE;
    if(piston_state_!=PISTON_STATE_OK)
        regulation_state_ = STATE_PISTON_ISSUE;

    switch(regulation_state_){
        case STATE_SURFACE:
            /// Wait at surface until the waypoint is under the limit depth of regulation
            if(depth_set_point_ >= limit_depth_control_ && !emergency_)
                regulation_state_ = STATE_SINK;
            u = 0.;
            piston_set_point_ = 0;
            break;
        case STATE_SINK:
            if(depth_set_point_<limit_depth_control_) /// Case where set point is surface
                regulation_state_ = STATE_SURFACE;
            else if(depth_fusion_<limit_depth_control_){ /// Case where float is between surface and limit_depth_control
                u = flow_piston_sink_;

                /// Compute the position of the piston to be at equilibrium
                double position_eq = x(3) / tick_to_volume_; /// Assuming no compressibility effect

                /// First move to position_eq and the slowly decrease piston volume by flow_piston_sink_
                if(position_eq - piston_position_ > 2.*piston_reach_position_dead_zone_) { /// position reached
                    piston_set_point_ = position_eq;
                }
                else{
                    /// continue to sink until limit_depth_control_ is reached
                    piston_set_point_ += -u*dt/(tick_to_volume_); /// (m3/s * s) / (m3/tick)
                }
            }
            else { /// When limit_depth_control_ is reached
                regulation_state_ = STATE_CONTROL;
                piston_set_point_ = piston_position_;
            }
            break;
        case STATE_CONTROL:
            if(depth_set_point_<limit_depth_control_)
                regulation_state_ = STATE_SURFACE;
            else if(depth_fusion_>=limit_depth_control_){
                /// Test if data is too old
                if((this->now()-time_last_kalman_callback_)<safety_time_no_data_
                   && (this->now()-time_last_piston_callback_)<safety_time_no_data_){

                    /// Compute several commands according to velocity acceptable bounds
                    array<double, 4> u_tab;
                    u_tab[0] = compute_u(x, depth_set_point_, limit_velocity_+delta_velocity_lb_, approach_velocity_);
                    u_tab[1] = compute_u(x, depth_set_point_, limit_velocity_+delta_velocity_ub_, approach_velocity_);
                    u_tab[2] = compute_u(x, depth_set_point_+delta_position_lb_, limit_velocity_, approach_velocity_);
                    u_tab[3] = compute_u(x, depth_set_point_+delta_position_ub_, limit_velocity_, approach_velocity_);

                    /// Find best command
                    u=optimize_u(u_tab);

                    /// Mechanical limits (in = v_min, out = v_max)
                    if((piston_switch_top_ && u<0) || (piston_switch_bottom_ && u>0))
                        u = 0.0;

                    /// Limitation of u according to engine capabilities
                    u = std::clamp(u, -flow_max_, flow_max_);

                    /// Check next formula in the case where piston_set_point-piston_position > piston_max_velocity/frequency
                    /// Previous form do not allow movement under 1 tick
                    /// piston_set_point = piston_position - u/(tick_to_volume*control_loop_frequency);
                    piston_set_point_ -= u*dt/tick_to_volume_;

                    if(hold_depth_enable_ && abs(depth_set_point_-x(1))<hold_depth_value_enter_ && abs(x(0))<hold_velocity_enter_)
                        regulation_state_ = STATE_HOLD_DEPTH;
                }
                else{
                    /// Did not received state => surface
                    u=flow_piston_sink_;
                    piston_set_point_ += u*dt/tick_to_volume_;
                    RCLCPP_WARN(this->get_logger(), "[Depth_control_node] Timing issue with kalmann or depth fusion");
                }
            }
            else
                regulation_state_ = STATE_SINK;

            break;
        case STATE_HOLD_DEPTH:
            u=0.0;
            if(abs(depth_set_point_-x(1))>=hold_depth_value_exit_)
                regulation_state_ = STATE_CONTROL;
            break;

        case STATE_PISTON_ISSUE:
            u = 0.0;
            piston_set_point_ = 0;

            if(!emergency_ && piston_state_ == PISTON_STATE_OK)
                regulation_state_ = STATE_SURFACE;
            break;
        default:
            break;
    }

    /// Publish data to piston & debug

    piston_set_point_ = std::clamp(piston_set_point_, 0., piston_max_tick_value_);

//    if(abs(piston_set_point_old_ - piston_set_point_)>piston_hysteresis_){
//        piston_set_point_old_ = piston_set_point_;

        std_msgs::msg::Int32  msg_piston;

        msg_piston.data = round(piston_set_point_);
        publisher_piston_->publish(msg_piston);
//    }

    /// Limit debug messages to changes
    //if(debug_msg_.u != u || debug_msg_.piston_set_point != piston_set_point_ || debug_msg_.mode != regulation_state_) {
        debug_msg_.mode = regulation_state_;
        debug_msg_.u = u;
        debug_msg_.piston_set_point = piston_set_point_;
        publisher_debug_->publish(debug_msg_);
    //}
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthControlNode>());
    rclcpp::shutdown();
    return 0;
}