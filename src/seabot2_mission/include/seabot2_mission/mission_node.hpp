//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_MISSION_NODE_HPP
#define BUILD_MISSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_mission/mission.hpp"
#include "seabot2_mission/msg/mission_state.hpp"
#include "seabot2_mission/msg/depth_control_set_point.hpp"
#include "seabot2_light_driver/srv/light.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "seabot2_mission/srv/alpha_mission.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_temperature_profile/msg/temperature_profile.hpp"
#include "seabot2_mission/msg/temperature_keeping_debug.hpp"

using namespace std::chrono_literals;
using namespace std;

class MissionNode : public rclcpp::Node {
public:
    MissionNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    /// Variable
    /// Mission
    Mission mission_;
    string mission_file_name_ = "mission.xml";
    string mission_path_ = "./";
    bool mission_enable_ = true;
    double flash_next_waypoint_time_ = 5.0;
    int flash_number_ = 2;

    /// Controller
    double limit_velocity_default_ = 0.02;

    /// Interfaces
    rclcpp::Publisher<seabot2_mission::msg::MissionState>::SharedPtr publisher_mission_state_;
    rclcpp::Publisher<seabot2_mission::msg::DepthControlSetPoint>::SharedPtr publisher_depth_control_set_point_;
    rclcpp::Publisher<seabot2_mission::msg::TemperatureKeepingDebug>::SharedPtr publisher_temperature_keeping_debug_;

    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_data_;

    rclcpp::Client<seabot2_light_driver::srv::Light>::SharedPtr client_light_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_log_parameters_;
    rclcpp::Client<seabot2_mission::srv::AlphaMission>::SharedPtr client_alpha_mission_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_bag_recorder_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_mission_reload_ ;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_mission_enable_ ;

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
     *
     * @param request_header
     * @param request
     * @param response
     */
    void service_mission_reload_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     *
     * @param request_header
     * @param request
     * @param response
     */
    void service_mission_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * Call light service
     */
    void call_light();

    /**
     * Call log params
     */
    void call_log_params();

    /**
     * Load mission
     */
    int load_mission();

    /**
     * Call alpha mission
     * @param velocity_list
     */
    void call_velocity_computation(std::vector<float> &velocity_list);

    /**
     * Call restart bag
     */
    void call_restart_bag();

    /**
     * Kalman callback
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose::SharedPtr msg);

    /**
     * Kalman callback
     * @param msg
     */
    void temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData::SharedPtr msg);

    /**
     * Temp profile callback
     * @param msg
     */
//    void temperature_profile(const seabot2_temperature_profile::msg::TemperatureProfile ::SharedPtr msg);

};
#endif //BUILD_MISSION_NODE_HPP
