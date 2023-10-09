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

    double depth_set_point = mission_->get_temp_slope()*this->temperature + mission_->get_temp_intercept();

    if(this->temperature-mission_->get_temperature()<3){
        depth_set_point_accumulator_ += 0.2;
    }
    else{
        depth_set_point_accumulator_ = 0;
    }

    depth_set_point += depth_set_point_accumulator_;
    depth_set_point = fmax(depth_set_point, 2.0);

    mission_->get_depth_control_set_point().depth = depth_set_point;
    mission_->get_depth_control_set_point().limit_velocity = velocity;
    mission_->get_depth_control_set_point().enable_control = true;
    mission_->get_depth_control_set_point().header.stamp = time;
}

void WaypointTemperatureProfile::process(const rclcpp::Time &time) {
    // Make a profile between two depth and two temperature
    // Max and min depth are limit boundaries
    // max and min temperature give the threshold to change the velocity
}
