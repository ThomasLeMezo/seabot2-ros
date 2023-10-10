//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_DEPTH_CONTROL_NODE_HPP
#define BUILD_DEPTH_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_mission/msg/depth_control_set_point.hpp"
#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "seabot2_depth_control/msg/depth_control_debug.hpp"
#include "seabot2_piston_driver/msg/piston_set_point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Dense>
#include "seabot2_safety/msg/safety_status.hpp"
#include "seabot2_density/msg/density.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_depth_control/alpha_solver.h"
#include "seabot2_depth_control/msg/alpha_debug.hpp"
#include "seabot2_mission/srv/alpha_mission.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "seabot2_depth_control/depth_control.h"

#define NB_STATES 8
#define PISTON_STATE_OK 2

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

class DepthControlNode : public rclcpp::Node {
public:
    DepthControlNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 200ms; /// loop dt

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    DepthControl dc_;

    std::vector<float> velocity_limits_requests_;
    bool velocity_limits_computations_ = false;

    bool enable_control_ = true; // Allow publish set point to piston

    std::vector<double> solver_velocity_, solver_alpha_;

    /// Interfaces
    rclcpp::Subscription<seabot2_kalman::msg::KalmanState>::SharedPtr subscriber_kalman_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_state_data_;
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_mission::msg::DepthControlSetPoint>::SharedPtr subscriber_mission_data_;
    rclcpp::Subscription<seabot2_safety::msg::SafetyStatus>::SharedPtr subscriber_safety_data_;
    rclcpp::Subscription<seabot2_density::msg::Density>::SharedPtr subscriber_density_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_data_;

    rclcpp::Publisher<seabot2_piston_driver::msg::PistonSetPoint>::SharedPtr publisher_piston_;
    rclcpp::Publisher<seabot2_depth_control::msg::DepthControlDebug>::SharedPtr publisher_debug_;
    rclcpp::Publisher<seabot2_depth_control::msg::AlphaDebug>::SharedPtr publisher_alpha_debug_;

    rclcpp::Service<seabot2_mission::srv::AlphaMission>::SharedPtr service_alpha_computation_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_alpha_generation_;

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
     * @param msg
     */
    void kalman_callback(const seabot2_kalman::msg::KalmanState &msg);

    /**
     *
     * @param msg
     */
    void density_callback(const seabot2_density::msg::Density &msg);

    /**
     *
     * @param msg
     */
    void piston_callback(const seabot2_piston_driver::msg::PistonState &msg);

    /**
     *
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void safety_callback(const seabot2_safety::msg::SafetyStatus &msg);

    /**
     *
     * @param msg
     */
    void depth_set_point_callback(const seabot2_mission::msg::DepthControlSetPoint &msg);

    /**
     *
     * @param msg
     */
    void temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg);

    /**
     * Compute the alpha values
     * @param request_header
     * @param request
     * @param response
     */
    void alpha_mission_pre_computation(const std::shared_ptr<rmw_request_id_t> request_header,
                                                         const std::shared_ptr<seabot2_mission::srv::AlphaMission::Request> request,
                                                         std::shared_ptr<seabot2_mission::srv::AlphaMission::Response> response);

    /**
     *
     * @param request_header
     * @param request
     * @param response
     */
    void alpha_generation(const std::shared_ptr<rmw_request_id_t> request_header,
                                            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /**
     * Publish messages
     */
    void publish_message();

    /**
     * Generate velocities
     */
    void generate_velocity_pairs();
};
#endif //BUILD_DEPTH_CONTROL_NODE_HPP
