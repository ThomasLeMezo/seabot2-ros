//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_MISSION_NODE_HPP
#define BUILD_MISSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_mission/mission.hpp"
#include "seabot2_mission/msg/waypoint.hpp"
#include "seabot2_light_driver/srv/light.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using namespace std;

class MissionNode : public rclcpp::Node {
public:
    MissionNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

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
    double approach_velocity_default_ = 1.0;

    /// Interfaces
    rclcpp::Publisher<seabot2_mission::msg::Waypoint>::SharedPtr publisher_waypoint_;
    rclcpp::Client<seabot2_light_driver::srv::Light>::SharedPtr client_light_;

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

};
#endif //BUILD_MISSION_NODE_HPP
