#ifndef BUILD_PISTON_NODE_H
#define BUILD_PISTON_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "seabot2_piston_driver/piston.h"
#include <memory>
#include "seabot2_msgs/msg/piston_state.hpp"
#include "seabot2_msgs/msg/piston_set_point.hpp"

using namespace std::chrono_literals;
using namespace std;

class PistonNode final : public rclcpp::Node {
public:
    PistonNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt

    bool is_detected_issue_reset_ = false;
    rclcpp::Time time_detected_issue_reset_ = this->now();
    std::chrono::milliseconds delay_detected_issue_reset_ = 5s;

    /// I2C configuration
    Piston piston_;

    /// Piston
    std::chrono::seconds delay_no_data_ = 30s;
    rclcpp::Time time_last_cmd_received_ = this->now();
    int last_cmd_ = -1;

    int cpt_piston_error_ = 0;
    const int cpt_piston_error_max_reset_ = 100;

    std::chrono::seconds delay_reset_piston_ = 10s;

    /// Topics
    rclcpp::Publisher<seabot2_msgs::msg::PistonState>::SharedPtr publisher_piston_state_;
    rclcpp::Subscription<seabot2_msgs::msg::PistonSetPoint>::SharedPtr subscription_position_set_point_;

    /// Functions
    void timer_callback();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    /**
     * Callback for set point position
     * @param msg
     */
    void topic_position_set_point_callback(const seabot2_msgs::msg::PistonSetPoint &msg);

};

#endif //BUILD_PISTON_NODE_H
