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
#include "seabot2_depth_control/msg/depth_control_debug.hpp"
#include "gpsd_client/msg/gps_fix.hpp"
#include <ncurses.h>
#include "bluerobotics_ping_driver/msg/profile.hpp"
#include "seabot2_density/msg/density.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"

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

    string hostname_ = "Seabot";

    const array<std::string, 4> gpsd_mode_string_ = {"NOT_SEEN",
                                                     "NO_FIX",
                                                     "2D",
                                                     "3D"};
    const array<std::string, 6> piston_state_string_ = {"SEARCH_SWITCH_BOTTOM",
                                                        "RELEASE_SWITCH_BOTTOM",
                                                        "BACK_SWITCH_BOTTOM",
                                                        "REGULATION",
                                                        "EXIT",
                                                        "BATT_LOW"};
    const array<std::string, 6> power_state_string_ = {"IDLE",
                                                       "MEASURE_VOLTAGE",
                                                       "POWER_ON",
                                                       "WAIT_TO_SLEEP",
                                                       "SLEEP"};

    const array<std::string, 8> depth_control_string_ = {"SURFACE",
                                                        "SINK",
                                                        "CONTROL",
                                                        "EMERGENCY",
                                                        "PISTON_ISSUE",
                                                        "HOLD_DEPTH"};

    /// Variable
    WINDOW *windows_robot_;
    WINDOW *windows_safety_;
    WINDOW *windows_internal_pressure_;
    WINDOW *windows_depth_;
    WINDOW *windows_power_;
    WINDOW *windows_depth_control_;
    WINDOW *windows_piston_;
    WINDOW *windows_mission_;
    WINDOW *windows_gnss_;
    WINDOW *windows_sensors_;

    int windows_max_y_{}, windows_max_x_{};
    int windows_default_y_=4;
    int windows_current_y_ = windows_default_y_, windows_current_x_{};
    int windows_width_max_{};

    /// Variable

    seabot2_safety::msg::SafetyStatus msg_safety_;
    seabot2_depth_filter::msg::DepthPose msg_depth_data_;
    pressure_bme280_driver::msg::Bme280Data msg_internal_sensor_filter_;
    seabot2_power_driver::msg::PowerState msg_power_data_;
    seabot2_piston_driver::msg::PistonState msg_piston_data_;
    seabot2_mission::msg::Waypoint msg_waypoint_;
    seabot2_depth_control::msg::DepthControlDebug msg_depth_control_;
    gpsd_client::msg::GpsFix msg_gnss_;
    bluerobotics_ping_driver::msg::Profile msg_profile_;
    seabot2_density::msg::Density msg_density_;
    temperature_tsys01_driver::msg::TemperatureSensorData msg_temperature_sensor_data_;

    rclcpp::Time time_last_safety_ = this->now();
    rclcpp::Time time_last_depth_data_ = this->now();
    rclcpp::Time time_last_internal_sensor_filter_ = this->now();
    rclcpp::Time time_last_power_data_ = this->now();
    rclcpp::Time time_last_piston_data_ = this->now();
    rclcpp::Time time_last_waypoint_ = this->now();
    rclcpp::Time time_last_depth_control_ = this->now();
    rclcpp::Time time_last_gnss_ = this->now();
    rclcpp::Time time_last_profile_ = this->now();
    rclcpp::Time time_last_density_ = this->now();
    rclcpp::Time time_last_temperature_sensor_data_ = this->now();

    bool msg_first_received_safety_ = false;
    bool msg_first_received_depth_data_ = false;
    bool msg_first_received_internal_sensor_filter_ = false;
    bool msg_first_received_power_data_ = false;
    bool msg_first_received_piston_data_ = false;
    bool msg_first_received_waypoint_ = false;
    bool msg_first_received_depth_control_ = false;
    bool msg_first_received_gnss_ = false;
    bool msg_first_received_profile_ = false;
    bool msg_first_received_density_ = false;
    bool msg_first_received_temperature_sensor_data_ = false;

    /// Interfaces
    rclcpp::Subscription<seabot2_safety::msg::SafetyStatus>::SharedPtr subscriber_safety_;
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
    rclcpp::Subscription<seabot2_power_driver::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_piston_data_;
    rclcpp::Subscription<seabot2_mission::msg::Waypoint>::SharedPtr subscriber_mission_;
    rclcpp::Subscription<seabot2_depth_control::msg::DepthControlDebug>::SharedPtr subscriber_control_debug_;
    rclcpp::Subscription<gpsd_client::msg::GpsFix>::SharedPtr subscriber_gnss_;
    rclcpp::Subscription<bluerobotics_ping_driver::msg::Profile>::SharedPtr subscriber_profile_;
    rclcpp::Subscription<seabot2_density::msg::Density>::SharedPtr subscriber_density_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_sensor_data_;

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
     *  Piston callback
     * @param msg
     */
    void piston_callback(const seabot2_piston_driver::msg::PistonState &msg);

    /**
     *  Safety callback
     * @param msg
     */
    void safety_callback(const seabot2_safety::msg::SafetyStatus &msg);

    /**
     *  Waypoint callback
     * @param msg
     */
    void waypoint_callback(const seabot2_mission::msg::Waypoint &msg);

    /**
     *  Control debug callback
     * @param msg
     */
    void depth_control_callback(const seabot2_depth_control::msg::DepthControlDebug &msg);

    /**
     * Gps callback
     * @param msg
     */
    void gnss_callback(const gpsd_client::msg::GpsFix &msg);

    /**
     * Profile callback
     * @param msg
     */
    void profile_callback(const bluerobotics_ping_driver::msg::Profile &msg);

    /**
     * Density callback
     * @param msg
     */
    void density_callback(const seabot2_density::msg::Density &msg);

    /**
     * Temperature sensor data callback
     * @param msg
     */
    void temperature_sensor_data_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg);

    /**
     *  Update safety windows
     */
    void update_safety_windows();

    /**
     * Update mission windows
     */
    void update_mission_windows();

    /**
     *  Update internal pressure windows
     */
    void update_internal_pressure_windows();

    /**
     * Update power windows
     */
    void update_power();

    /**
     *  Update depth windows
     */
    void update_depth();

    /**
     * Update piston windows
     */
    void update_piston();

    /**
     * Update robot info windows
     */
    void update_robot();

    /**
     * Update depth windows
     */
    void update_depth_control();

    /**
     * Update gnss windows
     */
    void update_gnss();

    /**
     * Update sensor windows
     */
    void update_sensors();

    /**
     *  Update all windows
     */
    static std::string set_color_valid(WINDOW *w, bool valid, const std::string& text="");

    /**
     *
     * @param valid
     * @return
     */
    static std::string get_bool_text(bool valid);

    /**
     *  Create new sub window
     */
    WINDOW * create_new_sub_window(int height, int width, const string &name);

    /**
     * Create windows
     */
    void create_windows();

};
#endif //BUILD_WTF_NODE_HPP
