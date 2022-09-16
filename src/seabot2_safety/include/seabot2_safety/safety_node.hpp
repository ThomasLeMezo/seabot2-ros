//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_SAFETY_NODE_HPP
#define BUILD_SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"

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

    /// Interfaces
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


};
#endif //BUILD_SAFETY_NODE_HPP
