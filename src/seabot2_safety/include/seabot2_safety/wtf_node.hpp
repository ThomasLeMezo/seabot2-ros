#ifndef BUILD_WTF_NODE_HPP
#define BUILD_WTF_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include "seabot2_power_driver/msg/power_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seabot2_safety/msg/safety_status.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "seabot2_mission/msg/waypoint.hpp"
#include <ncurses.h>

using namespace std::chrono_literals;
using namespace std;

class WtfNode : public rclcpp::Node {
public:
    WtfNode();
    ~WtfNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    /// Variable
    WINDOW *windows_robot_;
    WINDOW *windows_safety_;
    WINDOW *windows_internal_pressure_;
    WINDOW *windows_depth_;
    WINDOW *windows_power_;
    WINDOW *windows_piston_;
    WINDOW *windows_mission_;

    /// Variable

    seabot2_safety::msg::SafetyStatus msg_safety_;
    seabot2_depth_filter::msg::DepthPose msg_depth_data_;
    pressure_bme280_driver::msg::Bme280Data msg_internal_sensor_filter_;
    seabot2_power_driver::msg::PowerState msg_power_data_;
    seabot2_piston_driver::msg::PistonState msg_piston_data_;
    seabot2_mission::msg::Waypoint msg_waypoint_;

    rclcpp::Time time_last_safety_ = this->now();
    rclcpp::Time time_last_depth_data_ = this->now();
    rclcpp::Time time_last_internal_sensor_filter_ = this->now();
    rclcpp::Time time_last_power_data_ = this->now();
    rclcpp::Time time_last_piston_data_ = this->now();
    rclcpp::Time time_last_waypoint_ = this->now();

    bool msg_first_received_safety_ = false;
    bool msg_first_received_depth_data_ = false;
    bool msg_first_received_internal_sensor_filter_ = false;
    bool msg_first_received_power_data_ = false;
    bool msg_first_received_piston_data_ = false;
    bool msg_first_received_waypoint_ = false;

    /// Interfaces
    rclcpp::Subscription<seabot2_safety::msg::SafetyStatus>::SharedPtr subscriber_safety_;
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
    rclcpp::Subscription<seabot2_power_driver::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_piston_data_;
    rclcpp::Subscription<seabot2_mission::msg::Waypoint>::SharedPtr subscriber_mission_;

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

    /**
     * Depth Callback
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     * Internal sensor callback
     * @param msg
     */
    void internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg);

    /**
     * Power callback
     * @param msg
     */
    void power_callback(const seabot2_power_driver::msg::PowerState &msg);

    /**
     *
     * @param msg
     */
    void piston_callback(const seabot2_piston_driver::msg::PistonState &msg);

    /**
     *
     * @param msg
     */
    void safety_callback(const seabot2_safety::msg::SafetyStatus &msg);

    /**
     *
     * @param msg
     */
    void waypoint_callback(const seabot2_mission::msg::Waypoint &msg);

    /**
     *
     */
    void update_safety_windows();

    /**
     *
     */
    void update_mission_windows();

    /**
     *
     */
    void update_internal_pressure_windows();

    /**
     *
     */
    void update_power();

    /**
     *
     */
    void update_depth();

    /**
     *
     */
    void update_piston();

    /**
     *
     */
    void update_robot();

};
#endif //BUILD_WTF_NODE_HPP
