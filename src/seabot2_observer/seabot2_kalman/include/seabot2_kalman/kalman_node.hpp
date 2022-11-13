#ifndef BUILD_KALMANN_NODE_HPP
#define BUILD_KALMANN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_density/msg/density.hpp"
#include "std_msgs/msg/int32.hpp"
#include <eigen3/Eigen/Dense>
#include "seabot2_kalman/kalman.h"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

class Kalman;

class KalmanNode : public rclcpp::Node {
public:
    KalmanNode();

private:

    /// Variable
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 200ms; /// loop dt

    std::unique_ptr<Kalman> k_;

    /// Interfaces

    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_state_data_;
    rclcpp::Subscription<seabot2_density::msg::Density>::SharedPtr subscriber_density_;

    rclcpp::Publisher<seabot2_kalman::msg::KalmanState>::SharedPtr publisher_kalman_;

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
    void state_callback(const seabot2_piston_driver::msg::PistonState &msg);

    /**
     *
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void density_callback(const seabot2_density::msg::Density &msg);

    /**
     *
     */
    void publish_data();

};
#endif //BUILD_KALMANN_NODE_HPP
