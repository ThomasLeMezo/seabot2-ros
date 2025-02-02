#ifndef BUILD_KALMANN_NODE_HPP
#define BUILD_KALMANN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_kalman/kalman.h"

#include "seabot2_msgs/msg/piston_state.hpp"
#include "seabot2_msgs/msg/depth_pose.hpp"
#include "seabot2_msgs/msg/kalman_state.hpp"
#include "seabot2_msgs/msg/density.hpp"
#include "seabot2_msgs/msg/pressure_sensor_data.hpp"
#include "seabot2_msgs/msg/temperature_sensor_data.hpp"

#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

class Kalman;

class KalmanNode final : public rclcpp::Node {
public:
    KalmanNode();

private:

    /// Variable
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 200ms; /// loop dt

    Kalman k_;

    /// Interfaces

    rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_msgs::msg::PistonState>::SharedPtr subscriber_state_data_;
    rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;
    rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_;

    rclcpp::Publisher<seabot2_msgs::msg::KalmanState>::SharedPtr publisher_kalman_;

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
    void state_callback(const seabot2_msgs::msg::PistonState &msg);

    /**
     *
     * @param msg
     */
    void depth_callback(const seabot2_msgs::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void density_callback(const seabot2_msgs::msg::Density &msg);

    /**
     *
     * @param msg
     */
    void pressure_callback(const seabot2_msgs::msg::PressureSensorData &msg);

    /**
     *
     * @param msg
     */
    void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg);


    /**
     *
     */
    void publish_data();

};
#endif //BUILD_KALMANN_NODE_HPP
