//
// Created by lemezoth on 18/05/23.
//

#include "seabot2_depth_control/depth_control.h"

DepthControl::DepthControl(const rclcpp::Time &start_time)
        : alpha_solver_(){
    set_start_time(start_time);
    update_coeff();
}

void DepthControl::set_start_time(const rclcpp::Time &start_time){
    time_last_kalman_callback_ = start_time;
    time_last_piston_callback_ = start_time;
    last_waypoint_time_ = start_time;
}

void DepthControl::update_state(const double &velocity,
                                const double &depth,
                                const double &chi,
                                const double &chi2,
                                const double &cz,
                                const double &offset_total,
                                const rclcpp::Time &time_update) {
    kalman_velocity_ = velocity;
    kalman_depth_ = depth;
    kalman_chi_ = chi;
    kalman_chi2_= chi2;
    kalman_cz_= cz;
    kalman_total_offset_ = offset_total;
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
    piston_volume_ = -piston_position_*tick_to_volume_;
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
                                   const bool &enable_control){
    depth_set_point_ = std::min(depth, limit_depth_);

    limit_velocity_ = limit_velocity;
    last_waypoint_time_ = time_update;

    // Update approach velocity
    if(enable_control) {
        if(limit_velocity_last_ != limit_velocity_) {
            approach_velocity_ = alpha_solver_.compute_alpha(limit_velocity_);
            limit_velocity_last_ = limit_velocity_;
        }
    }
    else
        approach_velocity_ = 1.0;
}

void DepthControl::update_density(const float &density){
    physics_rho_ = density;
    update_AB();
}

void DepthControl::update_AB(){
    coeff_A_ = physics_g_ * physics_rho_ / (robot_added_mass_ + robot_mass_);
    // coeff_B_ = 0.5 * physics_rho_ * S_ / (2.0 * robot_mass_);
    coeff_B_ = 1.0 / (robot_added_mass_ + robot_mass_);
}

double DepthControl::fz_computation(const double velocity) const {
    // Compute the drag coefficient
    // Inside the [-0.3, 0.3] range, the drag coefficient is computed using the cd_function provided by CFD analysis
    // Outside this range, the drag coefficient is assumed to be constant

    if (velocity <= fz_model_boundary_velocity_positive_ && velocity >= fz_model_boundary_velocity_negative_) {
        return 119.9 * pow(velocity, 5)
                   + 1.0928 * pow(velocity, 4)
                   - 29.224 * pow(velocity, 3)
                   - 0.0388 * pow(velocity, 2)
                   - 0.4588 * velocity;
    }
    else{
        if(velocity > fz_model_boundary_velocity_positive_)
            return velocity * fz_derivative_at_model_boundary_positive_ + fz_offset_at_model_boundary_positive_;
        else
            return velocity * fz_derivative_at_model_boundary_negative_ + fz_offset_at_model_boundary_negative_;
    }
}

double DepthControl::fz_derivative_computation(const double velocity) const {
    const double velocity_clamped = std::clamp(velocity, fz_model_boundary_velocity_negative_, fz_model_boundary_velocity_positive_);
    return 5.0 * 119.9 * pow(velocity_clamped, 4)
           + 4.0 * 1.0928 * pow(velocity_clamped, 3)
           - 3.0 * 29.224 * pow(velocity_clamped, 2)
           - 2.0 * 0.0388 * velocity_clamped
           - 0.4588;
}

void DepthControl::update_coeff(){
    /// Computed parameters
    // S_ = M_PI*pow(robot_diameter_/2.0, 2);
    update_AB();
    flow_max_ = (motor_max_rpm_ / 60.) * screw_thread_ * pow(piston_diameter_/2.0, 2)*M_PI;

    /// Ensuring a safety margin on the piston flow of piston_flow_security_percentage
//    alpha_solver_.update_coeff(Cf_, coeff_A_, coeff_B_, flow_max_*piston_flow_security_percentage_);
}

void DepthControl::update_temperature(const float &temperature){
    temperature_ = temperature + degree_to_kelvin_;
}

double DepthControl::compute_u(const double set_point, const double limit_velocity){
    const double x1 = kalman_velocity_;
    const double x2 = kalman_depth_;
    const double x3 = piston_volume_;
    const double x5 = kalman_chi_;
    const double x6 = kalman_chi2_;
    const double x7 = kalman_cz_;

    const double A = coeff_A_;
    const double B = coeff_B_;
    const double beta = limit_velocity;
    const double alpha = approach_velocity_;

    const double e = alpha*(set_point-x2);
    const double de = -alpha*x1;
    const double T = 1.0 - pow(tanh(e), 2);
    const double dde = -alpha*beta*de*T;
    const double dT = -2.*de*tanh(e)*T;
    // double dx1 = -A*(x3+kalman_total_offset_)-B*x7*abs(x1)*x1;
    const double dx1 = -A*(x3+kalman_total_offset_)+B*x7*fz_computation(x1);

    const double y = x1-beta*tanh(e);
    const double dy = dx1 - beta*de*T;
    const double s = root_regulation_;

    y_debug_= y;
    dy_debug_ = dy;

    // double u = (1./A)* (-2.*s*dy+pow(s,2)*y-beta*(dde*T+de*dT)-2*B*x7*abs(x1)*dx1)+x1*(x5+2.*x6*x2);
    const double u = (1./A)* (-2.*s*dy+pow(s,2)*y-beta*(dde*T+de*dT)+B*x7*fz_derivative_computation(x1)*dx1)+x1*(x5+2.*x6*x2);
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
                double position_eq = kalman_total_offset_ / tick_to_volume_; /// Assuming no compressibility effect

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

                    if(hold_depth_enable_
                    && abs(depth_set_point_-kalman_depth_)<hold_depth_value_enter_
                    && abs(kalman_velocity_)<hold_velocity_enter_) {
                        if(!hold_depth_validation_){
                            hold_depth_validation_ = true;
                            hold_depth_validation_time_ = current_time;
                        }
                        else if((current_time-hold_depth_validation_time_)>=hold_depth_validation_duration_)
                            regulation_state_ = STATE_HOLD_DEPTH;
                    }
                    else
                        hold_depth_validation_ = false;
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
            if(abs(depth_set_point_-kalman_depth_)>=hold_depth_value_exit_)
                regulation_state_ = STATE_CONTROL;
            break;

        case STATE_PISTON_ISSUE:
            is_exit_ = true;
            u = 0.0;
            piston_set_point_ = 0;

            if(!emergency_ && piston_state_ == (int)PISTON_STATE_OK)
                regulation_state_ = STATE_SURFACE;
            break;

        case STATE_COMPUTE_ALPHA:
            u = 0.;
            piston_set_point_ = 0;
            is_exit_ = true;
            break;
        default:
            break;
    }

    piston_set_point_ = std::clamp(piston_set_point_, 0., piston_max_tick_value_);
    u_debug_ = u;
}
