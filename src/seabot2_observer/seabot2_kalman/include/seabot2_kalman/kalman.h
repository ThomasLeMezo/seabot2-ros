//
// Created by lemezoth on 12/11/22.
//

#ifndef BUILD_KALMAN_H
#define BUILD_KALMAN_H

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

#define NB_MESURES 1
#define NB_STATES 7
#define NB_COMMAND 1

class Kalman {

public:
    /**
     *
     */
    Kalman();

    /**
     *
     */
    void init_parameters(const rclcpp::Time &init_time);

    /**
     *
     * @param physics_rho
     */
    void update_density(double physics_rho);

public:

    /// Physical characteristics
    double physics_rho_ =  1025.0;
    double physics_g_ =  9.81;
    double robot_mass_ =  12.0;
    double robot_diameter_ =  0.125;
    double screw_thread_ =  1.e-3;
    double tick_per_turn_ =  2048*4;
    double piston_diameter_ =  0.045;
    double piston_max_tick_ =  1146880;

    double Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    double tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;
    double coeff_A_ = physics_g_ * physics_rho_ / robot_mass_;
    double coeff_B_ = 0.5 * physics_rho_ * Cf_ / robot_mass_;

    double piston_max_volume_ = piston_max_tick_ * tick_to_volume_;

    /// Initialization variables
    double enable_kalman_depth_ = 0.5; /// m
    double piston_volume_eq_init_ =  80e-6; /// m3
    double init_chi_ = 0.0; /// m3/m
    double init_chi2_ = 0.0; /// m3/m2
    double init_volume_air_ = 5e-4; /// m3/m
    bool enable_volume_air_ = true;

    double gamma_alpha_velocity_ =  1e-3; // 1e-5
    double gamma_alpha_depth_ =  1e-5; // 1e-5
    double gamma_alpha_offset_ =  5e-2*tick_to_volume_; // 2e-5
    double gamma_alpha_chi_ =  1e-3*tick_to_volume_; // 2e-8
    double gamma_alpha_chi2_ =  1e-3*tick_to_volume_; // 2e-8
    double gamma_alpha_cz_ =  1e-3;
    double gamma_alpha_volume_air_ =  1e-2;

    double gamma_init_velocity_ =  1e-1;
    double gamma_init_depth_ =  1.0e-2;
    double gamma_init_offset_ = piston_max_tick_ * tick_to_volume_; // 1e-2
    double gamma_init_chi_ =  30.0*tick_to_volume_; // 20
    double gamma_init_chi2_ =  30.0*tick_to_volume_; // 1e-1
    double gamma_init_cz_ =  0.1;
    double gamma_init_volume_air_ =  30e-6;

    double gamma_beta_depth_ =  1.0e-3; // 5e-4 (m)

public:
    /// Callback data
    double fusion_depth_ = 0.;
    double fusion_velocity_ = 0.;
    rclcpp::Time fusion_stamp_;

    double piston_position_ = 0.;
    double piston_position_last_ = 0.;
    rclcpp::Time piston_stamp_;
    double piston_set_point_ = 0.;

    double temperature_=288.15; /// in K
    double pressure_; /// in Pa

public:
    /// Kalman variables
/*
 *  xhat_ definition
 *  xhat_(0) velocity
 *  xhat_(1) depth
 *  xhat_(2) Piston volume to equilibrium
 *  xhat_(3) chi (chi*z)
 *  xhat_(4) chi2 (chi2*z²)
 *  xhat_(5) Cz
 *  xhat_(6) Bubble (Vb/z)
 */
    Matrix<double, NB_STATES, 1> xhat_ = Matrix<double, NB_STATES, 1>::Zero();
    Matrix<double,NB_STATES, 1> x_forcast_ = Matrix<double, NB_STATES, 1>::Zero();
    Matrix<double,NB_STATES,NB_STATES> gamma_ = Matrix<double,NB_STATES,NB_STATES>::Zero();
    Matrix<double,NB_STATES, NB_STATES> gamma_forcast_ = Matrix<double,NB_STATES,NB_STATES>::Zero();

    Matrix<double, NB_STATES, NB_STATES> gamma_alpha_ = Matrix<double, NB_STATES, NB_STATES>::Zero();
    Matrix<double, NB_MESURES, NB_MESURES> gamma_beta_ = Matrix<double, NB_MESURES, NB_MESURES>::Zero();
    Matrix<double, NB_MESURES, NB_STATES> Ck_ = Matrix<double, NB_MESURES, NB_STATES>::Zero();

    bool enable_kalman_ = true;
    bool is_valid_ = true;
    rclcpp::Time time_last_predict_;
    std::chrono::milliseconds forecast_dt_ = 0ms;

private:

    /**
 *
 * @param x
 * @param u
 * @return
 */
    Matrix<double,NB_STATES, 1> f_dyn(const Matrix<double,NB_STATES,1> &x,
                                      const Matrix<double,NB_COMMAND, 1> &u);

    /**
     *
     * @param x
     * @param gamma
     * @param u
     * @param gamma_alpha
     * @param dt
     */
    void kalman_predict(Matrix<double,NB_STATES, 1> &x,
                        Matrix<double,NB_STATES, NB_STATES> &gamma,
                        const Matrix<double,NB_COMMAND, 1> &u,
                        const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                        const double &dt);

    /**
     *
     * @param x
     * @param gamma
     * @param y
     * @param gamma_beta
     * @param Ck
     */
    void kalman_correc(Matrix<double,NB_STATES, 1> &x,
                       Matrix<double,NB_STATES,NB_STATES> &gamma,
                       const Matrix<double,NB_MESURES, 1> &y,
                       const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                       const Matrix<double,NB_MESURES, NB_STATES> &Ck);

    /**
     *
     * @param xhat
     */
    void init_kalman();

    /**
     * Test if state of Kalman filter is out of range of admissible values
     * @param xhat
     * @return
     */
    bool is_out_of_range(const Matrix<double, NB_STATES, 1> &xhat);

public:
    /**
     *
     * @param new_depth_data
     * @param new_piston_data
     */
    void compute_kalman(bool new_depth_data=false, bool new_piston_data=false);

    /**
     *
     * @param position
     * @param set_point
     * @param stamp
     */
    void set_new_piston_data(double position, double set_point, const rclcpp::Time &stamp);

    /**
     *
     * @param depth
     * @param velocity
     * @param stamp
     */
    void set_new_depth_data(double depth, double velocity, const rclcpp::Time &stamp);

    /**
     *
     * @param temperature
     */
    void update_temperature(double temperature);

    /**
     *
     * @param pressure
     */
    void update_pressure(double pressure);

};


#endif //BUILD_KALMAN_H
