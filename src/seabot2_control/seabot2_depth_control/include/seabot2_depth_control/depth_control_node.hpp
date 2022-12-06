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
#include "seabot2_depth_control/msg/depth_control_debug.hpp"
#include "seabot2_piston_driver/msg/piston_set_point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <eigen3/Eigen/Dense>
#include "seabot2_safety/msg/safety_status.hpp"
#include "seabot2_density/msg/density.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"

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

    /// Variable
    bool emergency_ = true;

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
    double flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_; /// in m3/sec

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
    bool is_exit_ = true;

    /// Callback data
    rclcpp::Time time_last_kalman_callback_ = this->now();
    rclcpp::Time time_last_piston_callback_ = this->now();
    // [Velocity; Depth; Volume; Offset, chi, chi2, Cz]
    Matrix<double, NB_STATES, 1> x = Matrix<double, NB_STATES, 1>::Zero();
    double depth_fusion_ = 0.0;
    double depth_set_point_ = 0.0;
    double limit_velocity_ = 0.0;
    double approach_velocity_ = 1.0;
    bool control_filter_ = false;
    double temperature_ = 288.15;
    double pressure_ = 101325;

    std::chrono::milliseconds last_waypoint_max_delay_ = 5s;
    rclcpp::Time last_waypoint_time_ = this->now();

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
    rclcpp::Subscription<seabot2_safety::msg::SafetyStatus>::SharedPtr subscriber_safety_data_;
    rclcpp::Subscription<seabot2_density::msg::Density>::SharedPtr subscriber_density_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_data_;

    rclcpp::Publisher<seabot2_piston_driver::msg::PistonSetPoint>::SharedPtr publisher_piston_;
    rclcpp::Publisher<seabot2_depth_control::msg::DepthControlDebug>::SharedPtr publisher_debug_;

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
    void waypoint_callback(const seabot2_mission::msg::Waypoint &msg);

    /**
     *
     * @param msg
     */
    void temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg);

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
