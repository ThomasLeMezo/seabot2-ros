#ifndef BUILD_SCREEN_NODE_H
#define BUILD_SCREEN_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_screen_driver/screen.h"
#include "pressure_bme280_driver/msg/bme280_data.hpp"

using namespace std::chrono_literals;
using namespace std;

class ScreenNode : public rclcpp::Node {
public:
    ScreenNode();

private:

    /// I2C configuration
    Screen screen_;

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::microseconds  loop_dt_ = 2s; // loop dt

    /// Variable
    std::array<unsigned char, 4> ip_ = {0, 0, 0, 0};
    double pressure_ = 0;
    double temperature_ = 0;
    double hygro_ = 0;
    double voltage_ = 0;
    string robot_name_;
    string mission_name_;
    unsigned int wp_id_ = 0;
    unsigned int wp_max_ = 0;
    rclcpp::Time time_next_wp;
    Screen::Robot_Status status_ = Screen::Robot_Status::ERROR;

    double depth_ = 0.0;
    double depth_no_update_ = 1.0;

    /// Topics
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_sensor_internal_;

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

private:
    /**
     * Get the hostname of the robot
     */
    void get_hostname();

    /**
     * Get the ip of the robot
     */
    void get_ip();

    /**
     * Callback for internal pressure messages
     * @param msg
     */
    void topic_internal_pressure_callback(const pressure_bme280_driver::msg::Bme280Data &msg);
};

#endif //BUILD_SCREEN_NODE_H
