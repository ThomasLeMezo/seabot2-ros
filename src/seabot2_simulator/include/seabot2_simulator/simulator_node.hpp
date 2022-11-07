#ifndef BUILD_SIMULATOR_NODE_HPP
#define BUILD_SIMULATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "simulator.h"

using namespace std::chrono_literals;
using namespace std;

class SimulatorNode : public rclcpp::Node {
public:
    SimulatorNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    /// Variable
    Simulator s_;

    /// Interfaces
//    rclcpp::Publisher<seabot2_safety::msg::SafetyStatus>::SharedPtr publisher_safety_;
//
//    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;

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

};
#endif //BUILD_SIMULATOR_NODE_HPP
