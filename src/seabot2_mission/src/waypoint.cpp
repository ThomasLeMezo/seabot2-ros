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

}

void WaypointSeafloorLanding::process(const rclcpp::Time &time) {

}

void WaypointTemperatureKeeping::process(const rclcpp::Time &time) {

}

void WaypointTemperatureProfile::process(const rclcpp::Time &time) {

}
