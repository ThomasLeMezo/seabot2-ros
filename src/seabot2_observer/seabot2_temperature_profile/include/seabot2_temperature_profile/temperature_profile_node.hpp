#ifndef BUILD_TEMPERATURE_PROFILE_NODE_HPP
#define BUILD_TEMPERATURE_PROFILE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "seabot2_depth_filter/msg/pressure_sensor_data.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_temperature_profile/msg/temperature_profile.hpp"

#include <eigen3/Eigen/Dense>

#include "seabot2_temperature_profile/temperature_profile.h"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

class TemperatureProfile;

class TemperatureProfileNode : public rclcpp::Node {
public:
    TemperatureProfileNode();

private:

    /// Variable
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 200ms; /// loop dt

    TemperatureProfile t_;

    double depth_=0.0;

    /// Interfaces
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_;

    rclcpp::Publisher<seabot2_temperature_profile::msg::TemperatureProfile>::SharedPtr publisher_temperature_profile_;

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     *
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg);

};
#endif //BUILD_TEMPERATURE_PROFILE_NODE_HPP
