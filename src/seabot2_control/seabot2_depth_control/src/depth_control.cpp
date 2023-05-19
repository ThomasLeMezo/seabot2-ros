//
// Created by lemezoth on 18/05/23.
//

#include "seabot2_depth_control/depth_control.h"

DepthControl::DepthControl(const rclcpp::Time &start_time)
        : alpha_solver_(){

    time_last_kalman_callback_ = start_time;
    time_last_piston_callback_ = start_time;
    last_waypoint_time_ = start_time;
}

void DepthControl::update_state(const double &velocity,
                                const double &depth,
                                const double &offset,
                                const double &chi,
                                const double &chi2,
                                const double &cz,
                                const double &volume_air,
                                const double &offset_total,
                                const rclcpp::Time &time_update) {
    x(0) = velocity;
    x(1) = depth;
//    x(2) -> piston volume given by piston callback
    x(3) = offset;
    x(4) = chi;
    x(5) = chi2;
    x(6) = cz;
    x(7) = volume_air;
    offset_total_ = offset_total;
    time_last_kalman_callback_ = time_update;
}

void DepthControl::update_piston(const int &position,
                                 const bool &switch_top,
                                 const bool &switch_bottom,
                                 const int &state,
                                 const rclcpp::Time &time_update) {
    piston_position_ = position;
    piston_switch_top_ = switch_top;
    piston_switch_bottom_ = switch_bottom;
    piston_state_ = state;
    x(2) = -piston_position_*tick_to_volume_;
    time_last_piston_callback_ = time_update;
}

void DepthControl::update_depth(const double &depth,
                                const double &pressure){
    depth_fusion_ = depth;
    pressure_ = pressure*1e5;
}

void DepthControl::update_safety(const bool &emergency,
                                 const float &limit_depth){
    emergency_ = emergency;
    limit_depth_ = limit_depth;
}

void DepthControl::update_waypoint(const float &depth,
                                   const double &limit_velocity,
                                   const rclcpp::Time &time_update,
                                   const bool &mission_enable){
    if(mission_enable)
        depth_set_point_ = std::min(depth, limit_depth_);
    else
        depth_set_point_ = 0.0;

    limit_velocity_ = limit_velocity;
    last_waypoint_time_ = time_update;

    // Update approach velocity
    if(mission_enable)
        approach_velocity_ = alpha_solver_.compute_alpha(limit_velocity_);
    else
        approach_velocity_ = 1.0;
}

void DepthControl::update_density(const float &density){
    physics_rho_ = density;
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * Cf_ / (2.0 * robot_mass_);
}

void DepthControl::update_coeff(){
    /// Computed parameters
    S_ = M_PI*pow(robot_diameter_/2.0, 2);
    tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    coeff_A_ = physics_g_ * physics_rho_ / (2.0 * robot_mass_);
    coeff_B_ = 0.5 * physics_rho_ * S_ / (2.0 * robot_mass_);
    flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_;

    /// Ensuring a safety margin on the piston flow of piston_flow_security_percentage
    alpha_solver_.update_coeff(Cf_, coeff_A_, coeff_B_, flow_max_*piston_flow_security_percentage_);
}

void DepthControl::update_temperature(const float &temperature){
    temperature_ = temperature + 273.15;
}

double DepthControl::compute_u(double set_point, double limit_velocity){
    const double x1 = x(0); /// dz
    const double x2 = x(1); /// z
    const double x3 = x(2); /// piston volume
//    const double x4 = x(3); /// offset
    const double x5 = x(4); /// chi1
    const double x6 = x(5); /// chi2
    const double x7 = x(6); /// Cf
//    const double x8 = x(7); /// V_air
    const double A = coeff_A_;
    const double B = coeff_B_;
    const double beta = limit_velocity;
    const double alpha = approach_velocity_;

    double e = alpha*(set_point-x2);
    double de = -alpha*x1;
    double T = 1.0 - pow(tanh(e), 2);
    double dde = -alpha*beta*de*T;
    double dT = -2.*de*tanh(e)*T;
    double dx1 = -A*(x3+offset_total_)-B*x7*abs(x1)*x1;

    double y = x1-beta*tanh(e);
    double dy = dx1 - beta*de*T;
    double s = root_regulation_;

    y_debug_= y;
    dy_debug_ = dy;

    double u = (1./A)* (-2.*s*dy+pow(s,2)*y-beta*(dde*T+de*dT)-2*B*x7*abs(x1)*dx1)+x1*(x5+2.*x6*x2);
    return u;
}

double DepthControl::optimize_u(std::array<double, 4> &u_tab){
    sort(u_tab.begin(), u_tab.end());
    if(u_tab[0]<0.0 && u_tab[u_tab.size()-1]>0.0) // Case one positive, one negative => do not move
        return 0.0;
    else{ // Else choose the control that minimizes u
        sort(u_tab.begin(), u_tab.end(), [](int i, int j) { return abs(i) < abs(j); });
        return u_tab[0];
    }
}

void DepthControl::state_machine_step(const rclcpp::Duration &dt, const rclcpp::Time &current_time) {

    double u = 0.;

    /// Analyze specific cases
    if(emergency_)
        regulation_state_ = STATE_SURFACE;

    if((current_time-last_waypoint_time_)>last_waypoint_max_delay_ && !debug_)
        depth_set_point_ = 0.0;

    switch(regulation_state_){
        case STATE_SURFACE:
            /// Wait at surface until the waypoint is under the limit depth of regulation
            if(depth_set_point_ >= limit_depth_control_ && !emergency_)
                regulation_state_ = STATE_SINK;
            u = 0.;
            piston_set_point_ = 0;
            is_exit_ = true;
            break;
        case STATE_SINK:
            is_exit_ = false;
            if(depth_set_point_<limit_depth_control_) /// Case where set point is surface
                regulation_state_ = STATE_SURFACE;
            else if(depth_fusion_<limit_depth_control_){ /// Case where float is between surface and limit_depth_control
                u = flow_piston_sink_;

                /// Compute the position of the piston to be at equilibrium
                double position_eq = offset_total_ / tick_to_volume_; /// Assuming no compressibility effect

                /// First move to position_eq and the slowly decrease piston volume by flow_piston_sink_
                if(position_eq - piston_position_ > 2.*piston_reach_position_dead_zone_) { /// position reached
                    piston_set_point_ = position_eq;
                }
                else{
                    /// continue to sink until limit_depth_control_ is reached
                    piston_set_point_ += -u*dt.seconds()/(tick_to_volume_); /// (m3/s * s) / (m3/tick)
                }
            }
            else { /// When limit_depth_control_ is reached
                regulation_state_ = STATE_CONTROL;
                piston_set_point_ = piston_position_;
            }
            break;
        case STATE_CONTROL:
            is_exit_ = false;
            if(depth_set_point_<limit_depth_control_)
                regulation_state_ = STATE_SURFACE;
            else if(depth_fusion_>=limit_depth_control_){
                /// Test if data is too old
                if((current_time-time_last_kalman_callback_)<safety_time_no_data_
                   && (current_time-time_last_piston_callback_)<safety_time_no_data_){

                    if(control_filter_) {
                        /// Compute several commands according to velocity acceptable bounds
                        array<double, 4> u_tab{};
                        u_tab[0] = compute_u(depth_set_point_, limit_velocity_ + delta_velocity_lb_);
                        u_tab[1] = compute_u(depth_set_point_, limit_velocity_ + delta_velocity_ub_);
                        u_tab[2] = compute_u(depth_set_point_ + delta_position_lb_, limit_velocity_);
                        u_tab[3] = compute_u(depth_set_point_ + delta_position_ub_, limit_velocity_);

                        /// Find best command
                        u = optimize_u(u_tab);
                    }
                    else
                        u = compute_u(depth_set_point_, limit_velocity_);

                    /// Mechanical limits (in = v_min, out = v_max)
                    if((piston_switch_top_ && u<0) || (piston_switch_bottom_ && u>0)){
                        u = 0.0;
                        piston_set_point_ = piston_position_;
                    }

                    /// Limitation of u according to engine capabilities
                    u = std::clamp(u, -flow_max_, flow_max_);

                    /// Check next formula in the case where piston_set_point-piston_position > piston_max_velocity/frequency
                    /// Previous form do not allow movement under 1 tick
                    /// piston_set_point = piston_position - u/(tick_to_volume*control_loop_frequency);
                    piston_set_point_ -= u*dt.seconds()/tick_to_volume_;

                    if(hold_depth_enable_ && abs(depth_set_point_-x(1))<hold_depth_value_enter_ && abs(x(0))<hold_velocity_enter_)
                        regulation_state_ = STATE_HOLD_DEPTH;
                }
                else{
                    /// Did not received state => surface
                    u=flow_piston_sink_;
                    piston_set_point_ += u*dt.seconds()/tick_to_volume_;
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "[Depth_control] Timing issue with kalmann or depth fusion");
                }
            }
            else
                regulation_state_ = STATE_SINK;

            break;
        case STATE_HOLD_DEPTH:
            is_exit_ = false;
            u=0.0;
            if(abs(depth_set_point_-x(1))>=hold_depth_value_exit_)
                regulation_state_ = STATE_CONTROL;
            break;

        case STATE_PISTON_ISSUE:
            is_exit_ = true;
            u = 0.0;
            piston_set_point_ = 0;

            if(!emergency_ && piston_state_ == (int)PISTON_STATE_OK)
                regulation_state_ = STATE_SURFACE;
            break;
        default:
            break;
    }

    piston_set_point_ = std::clamp(piston_set_point_, 0., piston_max_tick_value_);
    u_debug_ = u;
}
