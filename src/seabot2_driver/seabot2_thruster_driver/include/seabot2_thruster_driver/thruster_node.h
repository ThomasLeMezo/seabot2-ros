#ifndef BUILD_THRUSTER_NODE_H
#define BUILD_THRUSTER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_thruster_driver/msg/engine.hpp"
#include "seabot2_thruster_driver/msg/velocity.hpp"
#include "seabot2_thruster_driver/thruster.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace std;

class ThrusterNode : public rclcpp::Node {
public:
    ThrusterNode();

private:
    /// Values
    bool state_enable_ = true;
    float velocity_linear = 0.0;
    float velocity_angular_ = 0.0;
    rclcpp::Time velocity_time_last;
    float manual_velocity_linear_ = 0.0;
    float manual_velocity_angular_ = 0.0;
    rclcpp::Time manual_velocity_time_last_;

    bool stop_sent_ = false;
    bool send_cmd_ = true;

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::microseconds dt_ = 100ms; /// loop dt

    /// Thrusters regulation
    float coeff_cmd_to_pwm_ = 9.0;
    std::chrono::microseconds delay_stop_ = 500ms;
    double max_angular_velocity_ = 1.0;
    double max_linear_velocity_ = 1.0;
    double max_velocity_pwm_ = 100.0; /// velocity of pwm command (per seconds)

    /// Thrusters configuration
    bool allow_backward_ = false;
    bool reverse_thruster_order_ = false;
    bool reverse_left_ = false;
    bool reverse_right_ = false;

    /// I2C configuration
    Thruster thruster_;

    /// Topics
    rclcpp::Publisher<seabot2_thruster_driver::msg::Engine>::SharedPtr publisher_engine_;
    rclcpp::Subscription<seabot2_thruster_driver::msg::Velocity>::SharedPtr subscription_velocity_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_manual_velocity_;

    /// Functions
    void timer_callback();

    void topic_velocity_callback(const seabot2_thruster_driver::msg::Velocity &msg);
    void topic_manual_velocity_callback(const geometry_msgs::msg::Twist &msg);

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init topics to this node (publishers & subscribers)
     */
    void init_topics();
};

#endif //BUILD_THRUSTER_NODE_H
