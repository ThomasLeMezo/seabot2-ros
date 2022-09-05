//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_DEPTH_POSE_NODE_HPP
#define BUILD_DEPTH_POSE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <deque>

using namespace std::chrono_literals;
using namespace std;

class DepthPoseNode : public rclcpp::Node {
public:
    DepthPoseNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

    /// Variable
    double rho_ = 1025.0;
    double g_ = 9.81;
    double velocity_limit_ = 0.5;

    double zero_depth_ = 1.0;

    /// Pressure to Depth
    deque<double> pressure_deque_;
    size_t filter_window_size_ = 5;
    int filter_median_remove_side_samples_ = 1;

    /// Velocity
    deque<pair<double, rclcpp::Time>> depth_memory_;
    size_t filter_velocity_window_size_ = 6;
    size_t velocity_dt_gap_sample_ = 5;
    size_t filter_velocity_median_remove_side_samples_ = 1;

// Zero depth
    deque<double> pressure_zero_depth_deque_;
    size_t zero_depth_window_size_ = 150;

    bool new_data_ = false;

    /// Interfaces
    rclcpp::Subscription<pressure_ms5803_driver::msg::PressureSensorData>::SharedPtr subscriber_pressure_data_;
    rclcpp::Publisher<seabot2_depth_filter::msg::DepthPose>::SharedPtr publisher_depth_data_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_zero_depth_ ;

    /// Functions
    void timer_callback();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     * Callback of the external pressure
     * @param msg
     */
    void pressure_callback(const pressure_ms5803_driver::msg::PressureSensorData &msg);

    /**
     * Service callback
     * @param request_header
     * @param request
     * @param response
     */
    void service_zero_pressure_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:

};
#endif //BUILD_DEPTH_POSE_NODE_HPP
