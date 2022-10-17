//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_SAFETY_NODE_HPP
#define BUILD_SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include "seabot2_power_driver/msg/power_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seabot2_safety/msg/safety_status.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"

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
    bool global_safety_ok_ = true;
    bool safety_published_frequency_= false;
    bool safety_depth_limit_= false;
    bool safety_batteries_limit_= false;
    bool safety_depressurization_= false;
    bool safety_seafloor_= false;
    bool safety_piston_= false;
    bool safety_zero_depth_= false;
    double cpu_= 0.;
    double ram_= 0.;

    double internal_humidity_ = 100.0;
    double internal_pressure_ = 1050.;
    double internal_temperature_ = 100.0;
    rclcpp::Time internal_last_received_ = this->now();
    std::chrono::milliseconds internal_no_data_warning_ = 2s;
    double internal_humidity_limit_ = 70.0;
    double internal_pressure_limit_ = 800.0;

    double depth_ = 0.0;
    double velocity_ = 1.0;
    rclcpp::Time depth_last_received_ = this->now();
    std::chrono::milliseconds depth_no_data_warning_ = 2s;
    double depth_limit_max_ = 110.0;

    double battery_volt_ = 0.0;
    rclcpp::Time battery_last_received_ = this->now();
    std::chrono::milliseconds battery_no_data_warning_ = 10s;
    double battery_volt_limit_ = 12.0;

    double depth_flash_surface_ = 0.5;
    bool flash_surface_enable_ = false;

    double piston_position_ = 0.0;
    rclcpp::Time piston_last_received_ = this->now();
    std::chrono::milliseconds piston_no_data_warning_ = 1s;
    double limit_piston_position_reset_depth_ = 100.0;

    double max_depth_reset_zero_ = 1.0; // Should take into account atmospheric pressure variations
    double max_velocity_reset_zero_ = 0.1;
    enum ZERO_DEPTH_STATUS {IDLE, WAIT_RESET};
    ZERO_DEPTH_STATUS reset_depth_status_ = ZERO_DEPTH_STATUS::IDLE;
    rclcpp::Time depth_reset_time_wait_ = this->now();
    std::chrono::milliseconds depth_reset_delay_wait_ = 10s;
    bool is_zero_depth_once_ = false;


    /// Interfaces
    rclcpp::Publisher<seabot2_safety::msg::SafetyStatus>::SharedPtr publisher_safety_;

    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
    rclcpp::Subscription<seabot2_power_driver::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_piston_data_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_zero_pressure_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_flash_surface_;

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
     * @return
     */
    bool test_depth();

    /**
     *
     * @return
     */
    bool test_zero_pressure();

    /**
     *
     * @return
     */
    bool test_battery();

    /**
     *
     * @return
     */
    bool test_piston();

    /**
     *
     * @return
     */
    bool test_internal_data();

    /**
     * Detect if surface is reached
     */
    void flash_surface();

    /**
     *
     * @param is_surface
     * @return
     */
    int call_service_flash_surface(const bool &is_surface);

    /**
     *
     * @return
     */
    int call_service_zero_depth();

    /**
     *
     */
    void get_ram_cpu();

};
#endif //BUILD_SAFETY_NODE_HPP
