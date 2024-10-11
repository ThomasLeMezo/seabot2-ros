//
// Created by lemezoth on 08/10/23.
//

#include "seabot2_mission/waypoint.hpp"

void WaypointDepth::process(const rclcpp::Time &time) {
    mission_->get_depth_control_set_point().depth = depth;
    mission_->get_depth_control_set_point().limit_velocity = velocity;
    mission_->get_depth_control_set_point().enable_control = true;
    mission_->get_depth_control_set_point().header.stamp = time;
}

void WaypointGNSSProfile::process(const rclcpp::Time &time) {
    mission_->get_depth_control_set_point().depth = 0.0;
    mission_->get_depth_control_set_point().limit_velocity = 0.1;
    mission_->get_depth_control_set_point().enable_control = true;
    mission_->get_depth_control_set_point().header.stamp = time;
}

void WaypointSeafloorLanding::process(const rclcpp::Time &time) {
    // Wait until the seabot2 reached 0.5m above the seafloor (for instance) and save piston volume value
    // Then shutdown kalman and depth control, and set the piston to minimum volume
    // When the waypoint is near ending, set back the piston to the previous volume
    // and then restart kalman and depth control
}

void WaypointTemperatureKeeping::process(const rclcpp::Time &time) {
    // Control the depth according to a temperature set point
    // Filter the temperature value and estimate the temperature gradient
    // Use the gradient to control the depth

//    double depth_set_point = mission_->get_temp_slope()*this->temperature + mission_->get_temp_intercept();

    double depth_set_point;
    if(mission_->get_temperature() > this->temperature_)
        depth_set_point = 100.0;
    else
        depth_set_point = 2.0;

    error_temperature_ = abs(this->temperature_ - mission_->get_temperature());
    double velocity = fmin(mission_->temperature_keeping_k_*error_temperature_, this->velocity);

    mission_->get_depth_control_set_point().depth = depth_set_point;
    mission_->get_depth_control_set_point().limit_velocity = velocity;
    mission_->get_depth_control_set_point().enable_control = true;
    mission_->get_depth_control_set_point().header.stamp = time;
}

void WaypointTemperatureProfile::process(const rclcpp::Time &time) {
    // Make a profile between two depth and two temperature
    // Max and min depth are limit boundaries
    // max and min temperature give the threshold to change the velocity

    if(first_init_){
        time_last_transition_ = time;
        first_init_ = false;

        mission_->get_depth_control_set_point().limit_velocity = this->velocity;
        mission_->get_depth_control_set_point().enable_control = true;
        mission_->get_depth_control_set_point().depth = depth_max_;
        state_ = PROFILE_GO_DOWN;
    }
    mission_->get_depth_control_set_point().header.stamp = time;

    switch (state_) {
        case PROFILE_GO_DOWN:
            if(mission_->get_temperature()<temperature_low_
                || mission_->get_depth()>depth_max_
                || (time-time_last_transition_)>max_delay_){
                state_ = PROFILE_GO_UP;
                mission_->get_depth_control_set_point().depth = depth_min_;
                time_last_transition_ = time;
            }
            break;
        case PROFILE_GO_UP:
            if(mission_->get_temperature()>temperature_high_
               || mission_->get_depth()<depth_min_
               || (time-time_last_transition_)>max_delay_){
                state_ = PROFILE_GO_DOWN;
                mission_->get_depth_control_set_point().depth = depth_max_;
                time_last_transition_ = time;
            }
            break;
    }
}

void WaypointStop::process(const rclcpp::Time &time) {
    mission_->get_depth_control_set_point().enable_control = false;
    mission_->get_depth_control_set_point().header.stamp = time;
}