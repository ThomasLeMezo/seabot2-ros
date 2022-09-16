//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_DEPTH_CONTROL_NODE_HPP
#define BUILD_DEPTH_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_mission/msg/waypoint.hpp"
#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include "seabot2_depth_control/msg/depth_control_debug.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Dense>

#define NB_STATES 7
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

    /// Variable
    bool emergency_ = false;

    /// Physical characteristics
    double physics_rho_ =  1025.0;
    double physics_g_ =  9.81;
    double robot_mass_ =  12.0;
    double robot_diameter_ =  0.125;
    double screw_thread_ =  1.e-3;
    double tick_per_turn_ =  2048*4;
    double piston_diameter_ =  0.045;
    double piston_max_tick_value_ =  1146880;

    double Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    double tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    double coeff_A_ = physics_g_ * physics_rho_ / robot_mass_;
    double coeff_B_ = 0.5 * physics_rho_ * Cf_ / robot_mass_;

    /// Compute regulation constant
    double root_regulation_ = -1.0;
    double limit_depth_control_ = 0.5; /// m
    double flow_piston_sink_ = -0.5e-6; /// m3/s

    double piston_reach_position_dead_zone_ = 50.;
    double piston_hysteresis_ = 0.6;
    double motor_max_rpm_ = 38.0;
    double flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_; /// in m3/sec (0.6

    /// Hold depth parameters
    bool hold_depth_enable_ = false;
    double hold_depth_value_enter_ = 0.05; /// m
    double hold_depth_value_exit_ = 0.1; /// m
    double hold_velocity_enter_ = 0.01; /// m/s

    double delta_velocity_lb_ = 0.;
    double delta_velocity_ub_ = 0.;
    double delta_position_lb_ = 0.;
    double delta_position_ub_ = 0.;

    rclcpp::Duration safety_time_no_data_ = 5s;

    long piston_position_ = 0;
    bool piston_switch_top_ = false;
    bool piston_switch_bottom_ = false;
    int piston_state_ = 0;
    double piston_set_point_ = 0.;
    double piston_set_point_old_ = 0.;

    /// Callback data
    rclcpp::Time time_last_kalman_callback_ = this->now();
    rclcpp::Time time_last_piston_callback_ = this->now();
    // [Velocity; Depth; Volume; Offset, chi, chi2, Cz]
    Matrix<double, NB_STATES, 1> x = Matrix<double, NB_STATES, 1>::Zero();
    double depth_fusion_ = 0.0;
    double depth_set_point_ = 0.0;
    double limit_velocity_ = 0.0;
    double approach_velocity_ = 1.0;

    /// State machine
    enum STATE_MACHINE {STATE_IDLE, STATE_SURFACE, STATE_SINK, STATE_CONTROL, STATE_STATIONARY, STATE_EMERGENCY, STATE_PISTON_ISSUE, STATE_HOLD_DEPTH};
    STATE_MACHINE regulation_state_ = STATE_SURFACE;

    /// Debug
    seabot2_depth_control::msg::DepthControlDebug debug_msg_;

    /// Interfaces
    rclcpp::Subscription<seabot2_kalman::msg::KalmanState>::SharedPtr subscriber_kalman_data_;
    rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_state_data_;
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<seabot2_mission::msg::Waypoint>::SharedPtr subscriber_mission_data_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_piston_;
    rclcpp::Publisher<seabot2_depth_control::msg::DepthControlDebug>::SharedPtr publisher_debug_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_emergency_;

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
    void state_callback(const seabot2_piston_driver::msg::PistonState &msg);

    /**
     *
     * @param msg
     */
    void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

    /**
     *
     * @param msg
     */
    void waypoint_callback(const seabot2_mission::msg::Waypoint &msg);

    void depth_control_emergency(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * Compute output
     * @param x
     * @param set_point
     * @param limit_velocity
     * @param approach_velocity
     * @return
     */
    double compute_u(const Matrix<double, NB_STATES, 1> &x, double set_point, double limit_velocity, double approach_velocity=1.0);

    /**
     *
     * @param u_tab
     * @return
     */
    double optimize_u(std::array<double, 4> &u_tab);
};
#endif //BUILD_DEPTH_CONTROL_NODE_HPP
