#ifndef BUILD_SAFETY_NODE_HPP
#define BUILD_SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_msgs/msg/depth_pose.hpp"
#include "seabot2_msgs/msg/bme280_data.hpp"
#include "seabot2_msgs/msg/power_state.hpp"
#include "seabot2_msgs/msg/safety_status2.hpp"
#include "seabot2_msgs/msg/piston_state.hpp"
#include "seabot2_msgs/msg/profile.hpp"
#include "seabot2_msgs/msg/gps_fix.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std;

class SafetyNode final : public rclcpp::Node {
public:
    SafetyNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    std::chrono::seconds delay_between_warning_msg_ = 2s;

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
    double hdd_empty_space_ = 0.; // in GB
    double hdd_empty_space_limit_ = 0.5; // in GB

    double internal_humidity_ = 100.0;
    double internal_pressure_ = 1050.;
    double internal_temperature_ = 100.0;
    rclcpp::Time internal_last_received_ = this->now();
    std::chrono::milliseconds internal_no_data_warning_ = 2s;
    double internal_humidity_limit_ = 70.0;
    double internal_pressure_limit_ = 800.0;
    rclcpp::Time internal_last_rclcpp_warning_ = this->now();

    double depth_ = 0.0;
    double velocity_ = 1.0;
    rclcpp::Time depth_last_received_ = this->now();
    std::chrono::milliseconds depth_no_data_warning_ = 2s;
    rclcpp::Time depth_limit_live_last_update_ = this->now();
    std::chrono::milliseconds depth_limit_live_last_update_warning_ = 20s;
    double depth_limit_max_ = 110.0;

    double battery_volt_ = 0.0;
    rclcpp::Time battery_last_received_ = this->now();
    std::chrono::milliseconds battery_no_data_warning_ = 10s;
    double battery_volt_limit_ = 12.5;
    int power_state_ = 0;
    enum  class POWER_STATE_STATUS {IDLE=0, MEASURE_VOLTAGE=1, WIRE_DETECTION=2, POWER_ON=3, WAIT_TO_SLEEP=4, POWER_SLEEP=5};

    double depth_flash_surface_ = 0.5;
    bool flash_surface_enable_ = false;

    double depth_chrip_enable_ = 1.0;
    bool chirp_enable_ = true;
    bool chirp_is_enable_ = false;

    double piston_position_ = 0.0;
    double piston_set_point_ = 0.0;
    bool piston_switch_top_ = false;
    int piston_state_ = 0;
    rclcpp::Time piston_last_received_ = this->now();
    std::chrono::milliseconds piston_no_data_warning_ = 1s;
    double limit_piston_position_reset_depth_ = 100.0;
    uint16_t piston_motor_speed_= 2000;
    uint16_t piston_motor_speed_stop_ = 2000;
    rclcpp::Time piston_error_velocity_time_ = this->now();
    bool piston_error_velocity_detected_ = false;
    std::chrono::milliseconds piston_error_velocity_delay_ = 10s;
    double piston_error_threshold_set_point_ = 1e5;
    double piston_error_threshold_position_ = 100;
    double piston_last_position_ = 0.0;

    double max_depth_reset_zero_ = 1.0; // Should take into account atmospheric pressure variations
    double max_velocity_reset_zero_ = 0.04;
    enum ZERO_DEPTH_STATUS {IDLE, WAIT_RESET};
    ZERO_DEPTH_STATUS reset_depth_status_ = ZERO_DEPTH_STATUS::IDLE;
    rclcpp::Time depth_reset_time_wait_ = this->now();
    std::chrono::milliseconds depth_reset_delay_wait_ = 10s;
    bool is_zero_depth_once_ = false;

    double ping_altitude_ = 0.0;
    int ping_confidence_ = 0;
    int ping_confidence_threshold_ = 90;
    rclcpp::Time ping_last_time_received_ = this->now();
    double robot_height_ping_ = 1.1;
    double safety_distance_from_seabed_ = 2.0;
    double bathy_ = 0.0;
    double depth_allowed_max_ = 100.0;
    double depth_allowed_min_ = 2.0;
    double depth_limit_live_ = 100.0;
    double limit_depth_filter_coeff_ = 0.9;
    std::chrono::milliseconds ping_no_data_warning_ = 10s;

    bool seabed_test_detected_ = false;
    rclcpp::Time seabed_test_first_detected_ = this->now();
    std::chrono::milliseconds seabed_delay_detection_ = 30s;

    bool gnss_fix_once_ = false;
    int gnss_mode_ = seabot2_msgs::msg::GpsFix::MODE_NOT_SEEN;

    bool gnss_fix_once_enable_ = true;

    bool enable_limit_depth_ = true;
    bool enable_flash_underwater_ = false;

    /// Interfaces
    rclcpp::Publisher<seabot2_msgs::msg::SafetyStatus2>::SharedPtr publisher_safety_;

    rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_msgs::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
    rclcpp::Subscription<seabot2_msgs::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Subscription<seabot2_msgs::msg::PistonState>::SharedPtr subscriber_piston_data_;
    rclcpp::Subscription<seabot2_msgs::msg::Profile>::SharedPtr subscriber_profile_data_;
    rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_data_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_zero_pressure_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_flash_surface_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_chirp_enable_;

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
    void depth_callback(const seabot2_msgs::msg::DepthPose &msg);

    /**
     * Internal sensor callback
     * @param msg
     */
    void internal_sensor_callback(const seabot2_msgs::msg::Bme280Data &msg);

    /**
     * Power callback
     * @param msg
     */
    void power_callback(const seabot2_msgs::msg::PowerState &msg);

    /**
     *
     * @param msg
     */
    void piston_callback(const seabot2_msgs::msg::PistonState &msg);

    /**
     * Profile callback
     * @param msg
     */
    void profile_callback(const seabot2_msgs::msg::Profile &msg);

    /**
     * Gnss callback
     * @param msg
     */
    void gpsd_callback(const seabot2_msgs::msg::GpsFix &msg);

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
     *  Test max depth
     */
    void test_depth_max();

    /**
     * Test if seabed is reached
     */
    bool test_seabed_reached();

    /**
     * Test if gnss fix is ok
     * @return
     */
    bool test_gnss_fix();

    bool test_hdd_available_space();

    /**
     * Detect if surface is reached
     */
    void flash_surface();

    /**
     * Enable chirp
     */
    void enable_chirp();

    /**
     *
     * @param is_surface
     * @return
     */
    int call_service_flash_surface(const bool &is_surface) const;

    /**
     * call_service_chirp_enable
     * @param enable
     * @return
     */
    int call_service_chirp_enable(const bool &enable) const;

    /**
     *
     * @return
     */
    int call_service_zero_depth() const;

    /**
     *
     */
    void get_ram_cpu();

    /**
     * Get the hard drive empty space
     */
    void get_hard_drive_empty_space();

};
#endif //BUILD_SAFETY_NODE_HPP
