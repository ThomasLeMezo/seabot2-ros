//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_SAFETY_NODE_HPP
#define BUILD_SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"

using namespace std::chrono_literals;
using namespace std;

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    /// Variable
    double internal_humidity_ = 100.0;
    double internal_pressure_ = 1050.;
    double internal_temperature_ = 100.0;
    rclcpp::Time internal_last_received_ = this->now();
    rclcpp::Duration internal_no_data_warning = 2s;

    double depth_ = 0.0;
    double velocity_ = 1.0;
    rclcpp::Time depth_last_received_ = this->now();
    rclcpp::Duration depth_no_data_warning = 2s;

    /// Interfaces
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;

//    rclcpp::Publisher<seabot2_mission::msg::Waypoint>::SharedPtr publisher_waypoint_;
//    rclcpp::Client<seabot2_light_driver::srv::Light>::SharedPtr client_light_;
//
//    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_mission_reload_ ;
//    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_mission_enable_ ;

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     * Timer callback
     */
    void timer_callback();

    /**
     * Depth Callback
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     * Internal sensor callback
     * @param msg
     */
    void internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg);

    /**
     *
     * @return
     */
    bool test_depth();

    /**
     *
     * @return
     */
    bool test_zero_pressure();

    /**
     *
     * @return
     */
    bool test_internal_data();

};
#endif //BUILD_SAFETY_NODE_HPP
