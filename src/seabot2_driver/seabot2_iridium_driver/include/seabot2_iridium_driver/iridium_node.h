//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_IRIDIUM_NODE_H
#define BUILD_IRIDIUM_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_iridium_driver/sbd/sbd.h"
#include "seabot2_iridium_driver/sbd/log_data.h"
#include "seabot2_iridium_driver/sbd/mission_xml.h"

#include "seabot2_msgs/msg/depth_pose.hpp"
#include "seabot2_msgs/msg/bme280_data.hpp"
#include "seabot2_msgs/msg/power_state.hpp"
#include "seabot2_msgs/msg/safety_status.hpp"
#include "seabot2_msgs/msg/gps_fix.hpp"
#include "seabot2_msgs/msg/mission_state.hpp"
#include "seabot2_msgs/msg/gnss_pose.hpp"
#include "seabot2_msgs/msg/iridium_session.hpp"
#include "seabot2_msgs/msg/iridium_status.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using namespace std;

class IridiumNode : public rclcpp::Node {
public:
    IridiumNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_read_, timer_write_;
    std::chrono::milliseconds loop_serial_read_ = 10ms; /// loop dt
    std::chrono::milliseconds loop_serial_write_ = 1000ms; /// loop dt

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    SBD sbd_;
    LogData log_state_;

    bool enable_iridium_sending_ = true;
    bool valid_fix_ = false;
    bool is_surface_ = false;
    std::string mission_path_ = "./";
    std::string mission_file_name_ = "mission.xml";

    rclcpp::Time last_time_communication_ = rclcpp::Time(0., RCL_ROS_TIME);
    std::chrono::seconds time_between_communication_ = 180s;
    std::chrono::milliseconds  surface_wait_time_ = 5000ms;
    double surface_depth_limit_ = 0.5;
    bool surface_is_valid_ = false;
    rclcpp::Time surface_time_detected_ = rclcpp::Time(0., RCL_ROS_TIME);
    rclcpp::Time time_last_log_version_ = rclcpp::Time(0., RCL_ROS_TIME);
    std::chrono::seconds delay_last_log_version_ = 30s;

    double fix_latitude_ = 0.;
    double fix_longitude_ = 0.;
    rclcpp::Time time_last_gnss_ = rclcpp::Time(0., RCL_ROS_TIME);
    std::chrono::seconds delay_last_gnss_ = 10s;
    bool enable_iridium_gnss_ = false;

    std::string serial_port_name_ = "/dev/iridium";
    unsigned int serial_baud_rate_ = 19200;
    bool sbd_debug_ = false;

    /// Topics
    rclcpp::Subscription<seabot2_msgs::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
    rclcpp::Subscription<seabot2_msgs::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Subscription<seabot2_msgs::msg::SafetyStatus>::SharedPtr subscriber_safety_data_;
    rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_data_;
    rclcpp::Subscription<seabot2_msgs::msg::GnssPose>::SharedPtr subscriber_gnss_pose_;
    rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_;
    rclcpp::Subscription<seabot2_msgs::msg::MissionState >::SharedPtr subscriber_mission;

    rclcpp::Publisher<seabot2_msgs::msg::IridiumSession>::SharedPtr publisher_iridium_session_;
    rclcpp::Publisher<seabot2_msgs::msg::IridiumStatus>::SharedPtr publisher_iridium_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_iridium_log_;

    /// Functions
    /**
     *  Callback for timer read
     */
    void timer_read_callback();

    /**
     *  Callback for timer write
     */
    void timer_write_callback();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

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
     * Safety callback
     * @param msg
     */
    void safety_callback(const seabot2_msgs::msg::SafetyStatus &msg);

    /**
     * Mission callback
     * @param msg
     */
    void mission_callback(const seabot2_msgs::msg::MissionState &msg);

    /**
     * Depth callback
     * @param msg
     */
    void depth_callback(const seabot2_msgs::msg::DepthPose &msg);

    /**
     * Gnss callback
     * @param msg
     */
    void gpsd_callback(const seabot2_msgs::msg::GpsFix &msg);

    /**
     * Gnss pose callback
     * @param msg
     */
    void gnss_pose_callback(const seabot2_msgs::msg::GnssPose &msg);

    /**
     * Decode data from iridium
     * @param data_raw
     */
    void call_decode(const string &data_raw);

    /**
     * Process data to iridium
     */
    void process();

};

#endif //BUILD_IRIDIUM_NODE_H
