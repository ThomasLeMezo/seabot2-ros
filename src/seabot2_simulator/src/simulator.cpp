//
// Created by lemezoth on 06/11/22.
//

#include "seabot2_simulator/simulator.h"
#include <math.h>
#include <iostream>
#include <chrono>
using namespace std::chrono;

double Simulator::get_density_from_depth(double z, double sea_pressure) {
    return ts.gsw_rho_t_exact(this->salinity_from_depth(z),
                              this->temperature_from_depth(z),
                              sea_pressure);
}

Simulator::Simulator(): ts(), k_() {

}

double Simulator::temperature_from_depth(double z) {
    temperature_ = 15.0;
    return temperature_;
}

double Simulator::salinity_from_depth(double z) {
    salinity_ = 0.0;
    return salinity_;
}

Matrix<double, SIMU_NB_STATES, 1> Simulator::f(const Matrix<double, SIMU_NB_STATES, 1> &x, int pwm) {
    Matrix<double, SIMU_NB_STATES, 1> dx = Matrix<double, SIMU_NB_STATES, 1>::Zero();

    double temp_K = temperature_from_depth(x(4))+ts.gtc.gsw_t0; /// in K
    sea_pressure_ = ts.gsw_p_from_z(x(4), latitude_); /// dbar
    double abs_pressure = sea_pressure_*1e5 + ts.gtc.gsw_p0; /// Pa
    g_ = ts.gsw_grav(latitude_, sea_pressure_);
    rho_ = get_density_from_depth(x(4), sea_pressure_);
    double coeff_A_ = g_ * rho_ / robot_mass_;
    double coeff_B_ = 0.5 * rho_ * Cf_ / robot_mass_;

    double V = battery_tension_*(static_cast<double>(pwm)-MOTOR_STOP)/(double)MOTOR_STOP;

    double T = 0.;

    dx(0) = x(1);
    dx(1) = Kt_/J_*x(2)-T/J_;
    dx(2) = -Ke_/L_*x(1)-R_/L_*x(2)+V/L_;

    double Vp = -(x(0)/M_PI)*screw_thread_*pow(piston_diameter_/2.0, 2);
    double Vair = (volume_air_p0*volume_air_init/volume_air_temp0)*(temp_K/abs_pressure);
    double Vext = fmin(0, M_PI*pow(antenna_diam_/2, 2)*x(4)-volume_equilibrium_); /// Volume of antenna when emerged [neg]

    dx(3) = -coeff_A_*(Vext+volume_equilibrium_+Vp-(chi_*x(4)+chi2_*pow(x(4), 2))+Vair)-coeff_B_*Cz_*abs(x(3))*x(3);
    dx(4) = x(3);

    return dx;
}

int Simulator::control_pwm(int position_set_point){
    double position = round(rad_to_tick_ * x_(0));

    const double motor_regulation_K = 0.3;
    const int motor_regulation_dead_zone = 50;
    int motor_set_point = MOTOR_STOP;

    int position_error = position_set_point-position;
    if(abs(position_error)>motor_regulation_dead_zone){

        int val = floor(((float)position_error)*motor_regulation_K);

        if(val>=0)
            motor_set_point = min(max(val, MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_UP);
        else
            motor_set_point = max(min(val, -MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_DOWN);
    }
    else{
        motor_set_point = MOTOR_STOP;
    }

    if((motor_set_point<MOTOR_STOP && x_(4)<=0)
       || (motor_set_point>MOTOR_STOP && x_(4) * rad_to_tick_ >= piston_max_tick_)){
        motor_cmd_ = MOTOR_STOP;
    }
    else{
        if(abs(motor_set_point-motor_cmd_)>motor_delta_speed){
            motor_cmd_ += (motor_cmd_<motor_set_point)?motor_delta_speed:-motor_delta_speed;
        }
        else{
            motor_cmd_ = motor_set_point;
        }
    }
    return motor_cmd_;
}

void Simulator::clear_memory(){
    memory_time.clear();
    memory_piston_position.clear();
    memory_piston_velocity.clear();
    memory_velocity.clear();
    memory_depth.clear();
}

void Simulator::simulate_pressure(){
    /// ToDo : simulation of filter
    pressure_sensor_ = x_(4) + pressure_sensor_dist_(generator_);
}

void Simulator::simulate_depth(){
    fusion_depth_ = pressure_sensor_ / (g_*rho_/1e5);
    fusion_velocity_ = 0.;
}

void Simulator::simulate_piston_position() {
    piston_position_ = x_(0)*rad_to_tick_;
}

void Simulator::save_data(double t){
    memory_time.push_back(t);
    memory_piston_position.push_back(piston_position_);
    memory_piston_velocity.push_back(x_(1) * rad_to_tick_);
    memory_velocity.push_back(x_(2));
    memory_depth.push_back(x_(3));

    memory_temperature.push_back(temperature_);
    memory_salinity.push_back(salinity_);
    memory_sea_pressure.push_back(sea_pressure_);
    memory_density.push_back(rho_);

    memory_kalman_velocity.push_back(k_.x_forcast_(0));
    memory_kalman_depth.push_back(k_.x_forcast_(1));
    memory_kalman_offset.push_back(k_.x_forcast_(2));
    memory_kalman_chi.push_back(k_.x_forcast_(3));
    memory_kalman_chi2.push_back(k_.x_forcast_(4));
    memory_kalman_cz.push_back(k_.x_forcast_(5));
    memory_kalman_air.push_back(k_.x_forcast_(6));

}

void Simulator::run_simulation(double t_max) {
    clear_memory();
    int pwm = MOTOR_STOP;
    piston_set_point_ = 100000;
    k_.init_parameters(rclcpp::Time(0., RCL_STEADY_TIME));

    auto start = high_resolution_clock::now();
    for(double t=0; t<t_max; t+=dt_) {

        /// Physical simulation
        if((t-control_pwm_last_time) >= control_pwm_dt) {
            control_pwm_last_time = t;
            pwm = control_pwm(piston_set_point_);
        }
        x_ += dt_ * f(x_, pwm);

        /// Pressure Simulation
        if((t-pressure_sensor_last_time) >= pressure_sensor_dt) {
            pressure_sensor_last_time = t;
            simulate_pressure();
            simulate_depth();

            /// Compute Kalman
            k_.set_new_depth_data(fusion_depth_, fusion_velocity_, rclcpp::Time(t*1e9, RCL_STEADY_TIME));
        }

        if((t-piston_last_time_)>= piston_dt_){
            piston_last_time_ = t;
            simulate_piston_position();

            /// Compute Kalman
            k_.set_new_piston_data(piston_position_, piston_set_point_, rclcpp::Time(t*1e9, RCL_STEADY_TIME));
        }

        if((t-memory_last_time)>=memory_dt) {
            memory_last_time = t;
            save_data(t);
        }
        nb_steps++;
    }
    auto end = high_resolution_clock::now();
    cout << "Time exec = " << duration_cast<microseconds>((end-start)).count() << "µs" << endl;
}


