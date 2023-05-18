//
// Created by lemezoth on 18/05/23.
//

#ifndef BUILD_DEPTH_CONTROL_H
#define BUILD_DEPTH_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "seabot2_depth_control/alpha_solver.h"

#define NB_STATES 8
#define PISTON_STATE_OK 2

class DepthControl {

public:
    DepthControl(const rclcpp::Time &start_time);

    bool debug_ = false;

    /// Variable
    AlphaSolver alpha_solver_;

    bool emergency_ = true;
    float limit_depth_ = 100.0;

    /// Physical characteristics
    double physics_rho_ =  1025.0;
    double physics_g_ =  9.81;
    double robot_mass_ =  12.0;
    double robot_diameter_ =  0.125;
    double screw_thread_ =  1.e-3;
    double tick_per_turn_ =  2048*4;
    double piston_diameter_ =  0.045;
    double piston_max_tick_value_ =  1146880;

    double S_ = M_PI*pow(robot_diameter_/2.0, 2);
    double Cf_ = 4.0;
    double tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    double coeff_A_ = physics_g_ * physics_rho_ / robot_mass_;
    double coeff_B_ = 0.5 * physics_rho_ * S_ / robot_mass_;

    /// Compute regulation constant
    double root_regulation_ = -1.0;
    double limit_depth_control_ = 0.5; /// m
    double flow_piston_sink_ = -0.5e-6; /// m3/s

    double piston_reach_position_dead_zone_ = 50.;
    double piston_hysteresis_ = 0.6;
    double motor_max_rpm_ = 38.0;
    double flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_; /// in m3/sec

    double piston_flow_security_percentage_ = 0.25;

    /// Hold depth parameters
    bool hold_depth_enable_ = false;
    double hold_depth_value_enter_ = 0.0; /// m
    double hold_depth_value_exit_ = 0.0; /// m
    double hold_velocity_enter_ = 0.0; /// m/s
    double hold_velocity_exit_ = 0.0;

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
    bool is_exit_ = true;

    bool enable_flow_max_ = true;

    /// Callback data
    rclcpp::Time time_last_kalman_callback_{};
    rclcpp::Time time_last_piston_callback_{};
    // [Velocity; Depth; Volume; Offset, chi, chi2, Cz]
    Eigen::Matrix<double, NB_STATES, 1> x = Eigen::Matrix<double, NB_STATES, 1>::Zero();
    double offset_total_ = 100.0e-6;
    double depth_fusion_ = 0.0;
    double depth_set_point_ = 0.0;
    double limit_velocity_ = 0.0;
    double approach_velocity_ = 1.0;
    bool control_filter_ = false;
    double temperature_ = 288.15;
    double pressure_ = 101325;

    std::chrono::milliseconds last_waypoint_max_delay_ = 5s;
    rclcpp::Time last_waypoint_time_{};

    /// State machine
    enum STATE_MACHINE {STATE_SURFACE, STATE_SINK, STATE_CONTROL, STATE_PISTON_ISSUE, STATE_HOLD_DEPTH};
    STATE_MACHINE regulation_state_ = STATE_SURFACE;

    /// Debug
    double y_debug_ = 0.0;
    double dy_debug_ = 0.0;
    double u_debug_ = 0.0;

public:

    /**
     * @brief Update the state of the depth control
     * @param velocity
     * @param depth
     * @param offset
     * @param chi
     * @param chi2
     * @param cz
     * @param volume_air
     * @param offset_total
     * @param time_update
     */
    void update_state(const double &velocity,
                        const double &depth,
                        const double &offset,
                        const double &chi,
                        const double &chi2,
                        const double &cz,
                        const double &volume_air,
                        const double &offset_total,
                        const rclcpp::Time &time_update);

    /**
     * @brief Update the data from the piston
     * @param position
     * @param switch_top
     * @param switch_bottom
     * @param state
     * @param time_update
     */
    void update_piston(const int &position,
                                     const bool &switch_top,
                                     const bool &switch_bottom,
                                     const int &state,
                                     const rclcpp::Time &time_update);

    /**
     * @brief Update the data from depth
     * @param depth [in m]
     * @param pressure [in bar]
     */
    void update_depth(const double &depth,
                      const double &pressure);

    /**
     * @brief Update the data from safety
     * @param emergency
     * @param limit_depth
     */
    void update_safety(const bool &emergency,
                                     const float &limit_depth);

    /**
     * @brief Update the data from the mission
     * @param depth
     * @param limit_velocity
     * @param time_update
     * @param mission_enable
     */
    void update_waypoint(const float &depth,
                    const double &limit_velocity,
                    const rclcpp::Time &time_update,
                    const bool &mission_enable);

    /**
     * @brief Update the data from density
     * @param density
     */
    void update_density(const float &density);

    /**
     * @brief Update the data from temperature
     * @param temperature [in degree celsius]
     */
    void update_temperature(const float &temperature);

    /**
     * @brief Compute the command associated to the depth control
     * @param set_point
     * @param limit_velocity
     * @return
     */
    double compute_u(double set_point, double limit_velocity);

    /**
     *  @brief Optimize the command to reduce power consumptionr
     * @param u_tab
     */
    double optimize_u(std::array<double, 4> &u_tab);

    /**
     * @brief Update coefficients computed from parameters
     */
    void update_coeff();

    /**
     * @brief Update the state machine
     * @param dt
     * @param current_time
     * @return the command to send to the piston
     */
    void state_machine_step(const double &dt, const rclcpp::Time &current_time);

private:

};


#endif //BUILD_DEPTH_CONTROL_H
