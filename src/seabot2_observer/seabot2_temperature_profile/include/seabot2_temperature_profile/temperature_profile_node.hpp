#ifndef BUILD_TEMPERATURE_PROFILE_NODE_HPP
#define BUILD_TEMPERATURE_PROFILE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_temperature_profile/temperature_profile.h"

#include "seabot2_msgs/msg/depth_pose.hpp"
#include "seabot2_msgs/msg/temperature_sensor_data.hpp"
#include "seabot2_msgs/msg/temperature_profile.hpp"

using namespace std::chrono_literals;
using namespace std;

class TemperatureProfile;

class TemperatureProfileNode final : public rclcpp::Node {
public:
    TemperatureProfileNode();

private:

    /// Variable
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 200ms; /// loop dt

    TemperatureProfile t_;

    double depth_ = 0.0;

    /// Interfaces
    rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_;

    rclcpp::Publisher<seabot2_msgs::msg::TemperatureProfile>::SharedPtr publisher_temperature_profile_;

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
    void depth_callback(const seabot2_msgs::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg);

};
#endif //BUILD_TEMPERATURE_PROFILE_NODE_HPP
