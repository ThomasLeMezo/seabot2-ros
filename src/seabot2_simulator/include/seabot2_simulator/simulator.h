//
// Created by lemezoth on 06/11/22.
//

#ifndef BUILD_SIMULATOR_H
#define BUILD_SIMULATOR_H

#include <eigen3/Eigen/Dense>
#include "seabot2_density/teos/TeosSea.h"
#include "seabot2_kalman/kalman/kalman.h"
#include <random>

//using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

#define SIMU_NB_STATES 5

#define MOTOR_STOP 2000
#define MOTOR_DEAD_ZONE 50
#define MOTOR_DOWN 500
#define MOTOR_UP 3500

class Simulator{

public:
        Simulator();

        Matrix<double, SIMU_NB_STATES, 1> f(const Matrix<double, SIMU_NB_STATES, 1> &x, int pwm=MOTOR_STOP);

        void run_simulation(double t_max=10.0);

        double salinity_from_depth(double z);

        double temperature_from_depth(double z);

        double get_density_from_depth(double z, double sea_pressure);

        int control_pwm(int position_set_point);

        void clear_memory();

        void simulate_pressure();

        void simulate_depth();

        void simulate_piston_position();

        void save_data(double t);

public:

    /* state variable x_
     * x[0] : theta (motor)
     * x[1] : dtheta (motor)
     * x[2] : i (current)
     * x[3] : dz (velocity)
     * x[4] : z (depth)
     */
    Matrix<double, SIMU_NB_STATES, 1> x_ = Matrix<double, SIMU_NB_STATES, 1>::Zero();
    int motor_cmd_ = MOTOR_STOP;
    double temperature_{}, sea_pressure_{}, salinity_{}, rho_{}, g_{};

    unsigned long int nb_steps = 0;

    /// ************** ///

    double dt_ = 1e-4;

    double latitude_ = 48.368894;
    const double robot_mass_ =  12.0;
    const double robot_diameter_ =  0.125;
    const double screw_thread_ =  1.e-3;
    double tick_per_turn_ =  2048*4;
    const double piston_diameter_ =  0.045;
    double piston_max_tick_ =  1146880;
    double screw_Radius_ = 6e-3; /// m
    double screw_FrictionCoefficient_ = 0.2; /// To be correctly evaluated ! Copper-Copper = 1.2

    double Cf_ = M_PI*pow(robot_diameter_/2.0, 2);
    double tick_to_volume_ = (screw_thread_/tick_per_turn_)*pow(piston_diameter_/2.0, 2)*M_PI;

    double piston_max_volume_ = piston_max_tick_ * tick_to_volume_;

    double battery_tension_ = 16.0; /// V
    double volume_equilibrium_ = 100e-6; /// m3
    double chi_ = 0.0;
    double chi2_ = 0.0;
    const double volume_air_init = 20e-6; /// m3
    const double volume_air_p0 = ts.gtc.gsw_p0; /// Pa
    const double volume_air_temp0 = ts.gtc.gsw_t0 + 15.0; /// 15°C in K
    double antenna_diam_ = 30e-3; /// m
    double Cz_ = 1.0;

    /// ************** MAXON MOTOR ************** ///
    double maxon_RotorInertia_ = 9.0; /// gcm2
    double maxon_SpeedConstant_ = 416; /// rpm/V
    double maxon_TorqueConstant_ = 22.9; /// mNm/A
    double maxon_TerminalResistance_ = 1.84; /// Ohms
    double maxon_TerminalInductance_ = 0.198; /// mH

    double maxon_NominalCurrent_ = 1.3; /// A
    double maxon_NominalTorque_ = 29.5; /// mMm
    double maxon_StallCurrent_ = 6.54; /// A

    double maxon_MechanicalTimeConstant_ = 3.14e-3; /// s
    /// No load speed = 4980 rpm (12V)
    double maxon_NoLoadSpeed_ = 4080; /// rpm
    double maxon_NoLoadCurrent_ = 20e-3; /// A

    double maxon_Reduction_ = 103; /// 103:1

    double seabot_AddedInductance_ = 1.619e-3; /// H
    double seabot_AddedInductanceResistance_ = 480e-3; /// Ohms

    double rpm_to_rad_s_ = M_PI/30.;
    double J_ = maxon_RotorInertia_ * 1e-7; /// Moment of inertia (kg.m^2)
    double Ke_ = 1./(maxon_SpeedConstant_*rpm_to_rad_s_); /// Electromotive force constant  (V/rad/sec)
    double Kt_ = maxon_TorqueConstant_*1e-3; /// Motor torque constant (N.m/Amp)
    double R_ = maxon_TerminalResistance_ + seabot_AddedInductanceResistance_; /// Electric resistance (Ohm)
    double L_ = maxon_TerminalInductance_*1e-3 + seabot_AddedInductance_; /// Electric inductance (H)
    double rad_to_tick_ = tick_per_turn_ / maxon_Reduction_;

    const double Tfstatic_ = Kt_*20e-3; /// 20mA
    const double pistonSurface_ = M_PI*pow(piston_diameter_/2.,2);
    const double i_screw_=screw_thread_/(2*screw_Radius_); /// # Note : error in the thesis formula : diam and not radius
    const double force_to_torque_coeff_ = screw_Radius_*tan(screw_FrictionCoefficient_+i_screw_);
    const double Tz_coeff = pistonSurface_ * force_to_torque_coeff_ / maxon_Reduction_;

    /// ******* dsPic control loop *******  ///
    #define REGULATION_LOOP_FREQ 50
    #define MOTOR_PWM_MAX 4000
    #define MOTOR_V_TO_CMD MOTOR_PWM_MAX/(2*16)
    int motor_delta_speed = (100/REGULATION_LOOP_FREQ)*MOTOR_V_TO_CMD; // Limit to 100V/s, delta_speed in PWM quantum/0.02s = 250
    double control_pwm_last_time = 0.0;
    double control_pwm_dt = 0.02; /// 50Hz

    TeosSea ts;

    /// ******* Memory *******  ///
    double memory_dt = 0.02;
    double memory_last_time = 0;
    std::vector<double> memory_time, memory_piston_position, memory_piston_velocity, memory_velocity, memory_depth;
    std::vector<double> memory_temperature, memory_salinity, memory_sea_pressure, memory_density;
    std::vector<double> memory_kalman_depth, memory_kalman_velocity, memory_kalman_offset, memory_kalman_chi,
                        memory_kalman_chi2, memory_kalman_cz, memory_kalman_air;

    /// Kalman ///
    Kalman k_;

    /// Sensors
    double pressure_sensor_last_time = 0.0;
    double pressure_sensor_dt = 0.2; /// s
    double pressure_sensor_ = 0.; /// Simulated (noisy) value of the pressure sensor
    const double pressure_sensor_mean_ = 0.0;
    const double pressure_sensor_stddev_ = 0.01;
    std::default_random_engine generator_;
    std::normal_distribution<double> pressure_sensor_dist_{pressure_sensor_mean_, pressure_sensor_stddev_};

    double piston_last_time_ = 0.01;
    double piston_dt_ = 0.2;
    double piston_position_ = 0.;
    int piston_set_point_ = 0;

    /// Fusion
    double fusion_depth_{}, fusion_velocity_{};
};


#endif //BUILD_SIMULATOR_H
