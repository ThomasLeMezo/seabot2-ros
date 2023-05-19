//
// Created by lemezoth on 06/11/22.
//

#include "seabot2_simulator/simulator.h"
#include <cmath>
#include <iostream>
#include <chrono>

#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "rosbag2_storage/storage_options.hpp"

using namespace std::chrono;

double Simulator::get_density_from_depth(double z, double sea_pressure) {
    return ts.gsw_rho_t_exact(this->salinity_from_depth(z),
                              this->temperature_from_depth(z),
                              sea_pressure);
}

void Simulator::set_start_time(const rclcpp::Time &start_time){
    start_time_ = start_time;
}

Simulator::Simulator(const rclcpp::Time &start_time):
    ts(),
    k_(),
    dc_(start_time),
    mission_(){

    init_bag_writer();
    start_time_ = start_time;
}

void Simulator::init_bag_writer(){
    bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options({"my_bag", "mcap"});;

    bag_writer_->open(storage_options);

    bag_writer_->create_topic( {"/observer/kalman",
                                 "seabot2_kalman/msg/KalmanState",
                                 rmw_get_serialization_format(),
                                 ""});

    bag_writer_->create_topic( {"/driver/piston_position",
                                 "seabot2_piston_driver/msg/PistonState",
                                 rmw_get_serialization_format(),
                                 ""});
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
    abs_pressure_ = sea_pressure_*1e5 + ts.gtc.gsw_p0; /// Pa
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
    double Vair = (volume_air_p0*volume_air_init/volume_air_temp0)*(temp_K/abs_pressure_);
    double Vext = fmin(0, M_PI*pow(antenna_diam_/2, 2)*x(4)-volume_equilibrium_); /// Volume of antenna when emerged [neg]

    dx(3) = -coeff_A_*(Vext+volume_equilibrium_+Vp-(chi_*x(4)+chi2_*pow(x(4), 2))+Vair)-coeff_B_*Cz_*abs(x(3))*x(3);
    dx(4) = x(3);

    return dx;
}

int Simulator::control_pwm(int position_set_point){
    double position = round(rad_to_tick_ * x_(0));

    const double motor_regulation_K = 0.3;
    const int motor_regulation_dead_zone = 50;
    motor_set_point_ = MOTOR_STOP;

    int position_error = position_set_point-position;
    if(abs(position_error)>motor_regulation_dead_zone){

        int val = floor(((float)position_error)*motor_regulation_K);

        if(val>=0)
            motor_set_point_ = min(max(val, MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_UP);
        else
            motor_set_point_ = max(min(val, -MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_DOWN);
    }
    else{
        motor_set_point_ = MOTOR_STOP;
    }

    if((motor_set_point_<MOTOR_STOP && x_(4)<=0)
       || (motor_set_point_>MOTOR_STOP && x_(4) * rad_to_tick_ >= piston_max_tick_)){
        motor_cmd_ = MOTOR_STOP;
    }
    else{
        if(abs(motor_set_point_-motor_cmd_)>motor_delta_speed){
            motor_cmd_ += (motor_cmd_<motor_set_point_)?motor_delta_speed:-motor_delta_speed;
        }
        else{
            motor_cmd_ = motor_set_point_;
        }
    }

    /// Switchs
    if(x_(4)<=0)
        switch_bottom_ = true;
    else
        switch_bottom_ = false;

    if(x_(4) * rad_to_tick_ >= piston_max_tick_)
        switch_top_ = true;
    else
        switch_top_ = false;

    return motor_cmd_;
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

void Simulator::save_data(const rclcpp::Time &t){
//    memory_time.push_back(t);
//    memory_piston_position.push_back(piston_position_);
//    memory_piston_velocity.push_back(x_(1) * rad_to_tick_);
//    memory_velocity.push_back(x_(2));
//    memory_depth.push_back(x_(3));
//
//    memory_temperature.push_back(temperature_);
//    memory_salinity.push_back(salinity_);
//    memory_sea_pressure.push_back(sea_pressure_);
//    memory_density.push_back(rho_);
//
//    memory_kalman_velocity.push_back(k_.x_forcast_(0));
//    memory_kalman_depth.push_back(k_.x_forcast_(1));
//    memory_kalman_offset.push_back(k_.x_forcast_(2));
//    memory_kalman_chi.push_back(k_.x_forcast_(3));
//    memory_kalman_chi2.push_back(k_.x_forcast_(4));
//    memory_kalman_cz.push_back(k_.x_forcast_(5));
//    memory_kalman_air.push_back(k_.x_forcast_(6));

    seabot2_kalman::msg::KalmanState msg_kalman;
    msg_kalman.velocity = k_.x_forcast_(0);
    msg_kalman.depth = k_.x_forcast_(1);
    msg_kalman.offset = k_.x_forcast_(2);
    msg_kalman.chi = k_.x_forcast_(3);
    msg_kalman.chi2 = k_.x_forcast_(4);
    msg_kalman.cz = k_.x_forcast_(5);
    msg_kalman.volume_air = k_.x_forcast_(6);
    msg_kalman.offset_total = k_.offset_total_;
    msg_kalman.header.stamp = k_.time_last_predict_;
    msg_kalman.variance[0] = k_.gamma_forcast_(0,0);
    msg_kalman.variance[1] = k_.gamma_forcast_(1,1);
    msg_kalman.variance[2] = k_.gamma_forcast_(2,2);
    msg_kalman.variance[3] = k_.gamma_forcast_(3,3);
    msg_kalman.variance[4] = k_.gamma_forcast_(4,4);
    msg_kalman.variance[5] = k_.gamma_forcast_(5,5);
    msg_kalman.variance[6] = k_.gamma_forcast_(6,6);
    msg_kalman.valid = k_.is_valid_;
    bag_writer_->write(msg_kalman, "/observer/kalman", t);

    seabot2_piston_driver::msg::PistonState msg_piston;
    msg_piston.header.stamp = t_;
    msg_piston.position = piston_position_;
    msg_piston.position_set_point = dc_.piston_set_point_;
    msg_piston.switch_top = switch_top_;
    msg_piston.switch_bottom = switch_bottom_;
    msg_piston.enable = true;
    msg_piston.motor_sens = (x_(1)>0)?1:0;
    msg_piston.state = (int)DepthControl::PISTON_STATE_OK;
    msg_piston.motor_speed_set_point = motor_set_point_;
    msg_piston.motor_speed = motor_cmd_;
    msg_piston.battery_voltage = battery_tension_;
    msg_piston.motor_current = x_(2);

}

void Simulator::run_simulation() {
    int pwm = MOTOR_STOP;
    mission_.load_mission(mission_file_name_, mission_path_);
    start_time_ = mission_.get_start_time() - mission_delay_before_start_;
    end_time_ = mission_.get_end_time() + mission_delay_after_end_;

    k_.init_parameters(start_time_);
    dc_.set_start_time(start_time_);

    auto start = high_resolution_clock::now();
    for(t_=start_time_; t_<=end_time_; t_+=dt_) {
        /// Physical simulation
        if((t_-control_pwm_last_time) >= control_pwm_dt) {
            control_pwm_last_time = t_;
            pwm = control_pwm(round(dc_.piston_set_point_));
        }
        x_ += dt_.seconds() * f(x_, pwm);

        /// Pressure Simulation
        if((t_-pressure_sensor_last_time) >= pressure_sensor_dt) {
            pressure_sensor_last_time = t_;
            simulate_pressure();
            simulate_depth();

            k_.update_density(rho_);
            k_.update_temperature(temperature_);
            k_.update_pressure(abs_pressure_/1e5);

            /// Compute Kalman
            k_.set_new_depth_data(fusion_depth_, fusion_velocity_, t_);
        }

        /// Piston simulation
        if((t_-piston_last_time_)>= piston_dt_){
            piston_last_time_ = t_;
            simulate_piston_position();

            /// Compute Kalman
            k_.set_new_piston_data(piston_position_, dc_.piston_set_point_, t_);
        }

        /// Mission simulation
        if(t_-mission_last_time_>= mission_dt_){
            mission_last_time_ = t_;
            mission_.compute_command(current_wp_, t_);
        }

        /// Depth control simulation
        if((t_-dc_last_time_)>= dc_dt_){
            dc_last_time_ = t_;

            dc_.update_state(k_.x_forcast_(0),
                             k_.x_forcast_(1),
                             k_.x_forcast_(2),
                             k_.x_forcast_(3),
                             k_.x_forcast_(4),
                             k_.x_forcast_(5),
                             k_.x_forcast_(6),
                             k_.offset_total_,
                             k_.time_last_predict_);
            dc_.update_piston(piston_position_,
                              switch_top_,
                              switch_bottom_,
                              (int)DepthControl::PISTON_STATE_OK,
                                piston_last_time_);
            dc_.update_depth(x_(4),
                             abs_pressure_/1e5);
            dc_.update_safety(false,
                               100.0);
            dc_.update_waypoint(current_wp_.depth,
                                current_wp_.limit_velocity,
                                t_,
                                current_wp_.mission_enable);
            dc_.update_density(rho_);
            dc_.update_temperature(temperature_);

            dc_.state_machine_step(dc_dt_, t_);
        }

        if((t_-memory_last_time)>=memory_dt) {
            memory_last_time = t_;
            save_data(t_);
        }
        nb_steps++;
    }
    auto end = high_resolution_clock::now();
    cout << "Time exec = " << duration_cast<milliseconds>((end-start)).count() << "ms" << endl;
}


