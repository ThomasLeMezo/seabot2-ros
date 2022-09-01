#ifndef BUILD_POWER_NODE_H
#define BUILD_POWER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_power_driver/power.h"
#include "seabot2_power_driver/msg/power_state.hpp"

using namespace std::chrono_literals;
using namespace std;

class PowerNode : public rclcpp::Node {
public:
    PowerNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 500ms; /// loop dt

    /// I2C configuration
    Power power_;

    /// Topics / Services
    rclcpp::Publisher<seabot2_power_driver::msg::PowerState>::SharedPtr publisher_power_state_;

    /// Variables
    rclcpp::Time time_turn_off_light_ = this->now();
    bool light_is_on_ = false;

    /// Functions
    void timer_callback();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init topics to this node (publishers & subscribers)
     */
    void init_topics();

    /**
     * Init services od this node
     */
    void init_services();

};

#endif //BUILD_POWER_NODE_H
