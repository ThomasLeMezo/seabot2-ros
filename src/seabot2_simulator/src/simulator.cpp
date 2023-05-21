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

#include "seabot2_depth_control/msg/alpha_debug.hpp"
#include "seabot2_depth_control/msg/depth_control_debug.hpp"
#include "gpsd_client/msg/gps_fix.hpp"
#include "seabot2_piston_driver/msg/piston_state.hpp"
#include "seabot2_piston_driver/msg/piston_set_point.hpp"
#include "seabot2_power_driver/msg/power_state.hpp"
#include "seabot2_depth_filter/msg/pressure_sensor_data.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include "bluerobotics_ping_driver/msg/profile.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_mission/msg/waypoint.hpp"
#include "seabot2_density/msg/density.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_log_parameters/msg/log_parameter.hpp"
#include "seabot2_power_driver/msg/power_state.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_safety/msg/safety_status.hpp"

#include "seabot2_simulator/msg/simulation_debug.hpp"

using namespace std::chrono;

double Simulator::get_density_from_depth(double z, double sea_pressure) {
    return ts.gsw_rho_t_exact(this->salinity_from_depth(z),
                              this->temperature_from_depth(z),
                              sea_pressure*1e-4);
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

    bag_writer_->create_topic( {"/simulation/debug",
                                "seabot2_simulator/msg/SimulationDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/control/alpha_debug",
                                "seabot2_depth_control/msg/AlphaDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/control/depth_control_debug",
                                "seabot2_depth_control/msg/DepthControlDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/fix",
                                "gpsd_client/msg/GpsFix",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/piston",
                                "seabot2_piston_driver/msg/PistonState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/piston_set_point",
                                "seabot2_piston_driver/msg/PistonSetPoint",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/power",
                                "seabot2_power_driver/msg/PowerState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/pressure_external",
                                "seabot2_depth_filter/msg/PressureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/pressure_internal",
                                "pressure_bme280_driver/msg/Bme280Data",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/profile",
                                "bluerobotics_ping_driver/msg/Profile",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/temperature",
                                "temperature_tsys01_driver/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/mission/waypoint",
                                "seabot2_mission/msg/Waypoint",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/density",
                                "seabot2_density/msg/Density",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/depth",
                                "seabot2_depth_filter/msg/DepthPose",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/kalman",
                                "seabot2_kalman/msg/KalmanState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/parameters",
                                "seabot2_log_parameters/msg/LogParameter",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/power",
                                "seabot2_power_driver/msg/PowerState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/pressure_internal",
                                "pressure_bme280_driver/msg/Bme280Data",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/temperature",
                                "temperature_tsys01_driver/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/safety/safety",
                                "seabot2_safety/msg/SafetyStatus",
                                rmw_get_serialization_format(), ""});

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
    sea_pressure_ = ts.gsw_p_from_z(-x(4), latitude_)*1e4; /// in Pa (z is negative, output dbar)
    abs_pressure_ = sea_pressure_ + ts.gtc.gsw_p0; /// Pa
    g_ = ts.gsw_grav(latitude_, sea_pressure_*1e-4);
    rho_ = get_density_from_depth(x(4), sea_pressure_);

    double coeff_A_ = g_ * rho_ / (2.0 * robot_mass_);
    double coeff_B_ = 0.5 * rho_ * S_ / (2.0 * robot_mass_);

    double V = battery_tension_*(static_cast<double>(pwm-MOTOR_STOP))/(double)MOTOR_STOP;

    dx(0) = x(1);
    dx(1) = Kt_/J_*x(2);
    dx(2) = -Ke_/L_*x(1)-R_/L_*x(2)+V/L_;

    piston_volume_ = (-(x(0)/(2*M_PI*maxon_Reduction_))*screw_thread_)*(M_PI*pow(piston_diameter_/2.0, 2));
    volume_air_ = (volume_air_nR_)*(temp_K/abs_pressure_);
    volume_antenna_ = min(0.0, M_PI*pow(robot_diameter_/2.0, 2)*x(4)-volume_equilibrium_); /// Volume of antenna when emerged [neg]

    volume_total_ = volume_antenna_+volume_equilibrium_+piston_volume_-(chi_*x(4)+chi2_*pow(x(4), 2))+volume_air_;
    dx(3) = -coeff_A_*(volume_total_)-coeff_B_*Cz_*abs(x(3))*x(3);
    dx(4) = x(3);

    if(isnan(volume_air_) || isnan(x(3) || isnan(piston_volume_) || isnan(abs_pressure_))){
        cout << V << " " << x(0) << " " << x(1) << " " << x(2) << " " << x(3) << " " << x(4) << ' '
        << sea_pressure_ << ' ' << piston_volume_ << ' ' << volume_air_ << ' '  << abs_pressure_ << endl;
        exit(EXIT_FAILURE);
    }

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
       || (motor_set_point_>MOTOR_STOP && ((x_(4) * rad_to_tick_) >= piston_max_tick_))){
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

void Simulator::simulate_sensors(){
    pressure_sensor_ = abs_pressure_/1e5 + pressure_sensor_dist_(generator_); // in Bar
    fusion_depth_ = (pressure_sensor_*1e5 - ts.gtc.gsw_p0) / (g_*rho_);
    fusion_velocity_ = x_(3); /// ToDo add diff
}

void Simulator::simulate_piston_position() {
    piston_position_ = x_(0)*rad_to_tick_;
}

void Simulator::save_data(const rclcpp::Time &t){

//    "/driver/fix"
//    "/driver/profile"
//    "/observer/parameters"
//    "/observer/power"
//    "/observer/pressure_internal"
//    "/observer/temperature"
//    "/safety/safety"

    seabot2_simulator::msg::SimulationDebug msg_simu_debug;
    msg_simu_debug.theta = x_(0);
    msg_simu_debug.dtheta = x_(1);
    msg_simu_debug.i = x_(2);
    msg_simu_debug.dz = x_(3);
    msg_simu_debug.z = x_(4);
    msg_simu_debug.piston_volume = piston_volume_;
    msg_simu_debug.volume_total = volume_total_;
    msg_simu_debug.volume_air = volume_air_;
    msg_simu_debug.volume_antenna = volume_antenna_;
    bag_writer_->write(msg_simu_debug, "/simulation/debug", t);

    seabot2_depth_filter::msg::DepthPose msg_depth;
    msg_depth.depth = fusion_depth_;    /// Todo verify
    msg_depth.zero_depth_pressure = 0.0;
    msg_depth.pressure = pressure_sensor_;
    msg_depth.velocity = fusion_velocity_;
    msg_depth.header.stamp = t_;
    bag_writer_->write(msg_depth, "/observer/depth", t);

    seabot2_density::msg::Density msg_density;
    msg_density.density = rho_;
    msg_density.header.stamp = t_;
    bag_writer_->write(msg_density, "/observer/density", t);

    bag_writer_->write(current_wp_, "/mission/waypoint", t);

    temperature_tsys01_driver::msg::TemperatureSensorData msg_temperature;
    msg_temperature.temperature = temperature_;
    msg_temperature.header.stamp = t_;
    bag_writer_->write(msg_temperature, "/driver/temperature", t);

    pressure_bme280_driver::msg::Bme280Data msg_pressure_internal;
    msg_pressure_internal.pressure = 0.7;
    msg_pressure_internal.temperature = 20.0;
    msg_pressure_internal.humidity = 0.4;
    bag_writer_->write(msg_pressure_internal, "/driver/pressure_internal", t);

    seabot2_depth_filter::msg::PressureSensorData msg_pressure_external;
    msg_pressure_external.pressure = pressure_sensor_;
    msg_pressure_external.temperature = temperature_;
    msg_pressure_external.header.stamp = t_;
    bag_writer_->write(msg_pressure_external, "/driver/pressure_external", t);

    seabot2_power_driver::msg::PowerState msg_power;
    msg_power.battery_volt = (float)battery_tension_;
    msg_power.cell_volt = array<float, 2>{(float)battery_tension_, 0.0};
    msg_power.esc_current = array<float, 2>{0, 0.};
    msg_power.motor_current = (float)x_(2);
    bag_writer_->write(msg_power, "/driver/power", t);

    seabot2_depth_control::msg::AlphaDebug msg_alpha;
    msg_alpha.approach_velocity = dc_.approach_velocity_;
    bag_writer_->write(msg_alpha, "/control/alpha_debug", t);

    seabot2_depth_control::msg::DepthControlDebug msg_control_debug;
    msg_control_debug.mode = dc_.regulation_state_;
    msg_control_debug.dy = dc_.dy_debug_;
    msg_control_debug.y = dc_.y_debug_;
    msg_control_debug.u = dc_.u_debug_;
    msg_control_debug.piston_set_point = dc_.piston_set_point_;
    bag_writer_->write(msg_control_debug, "/control/depth_control_debug", t);

    seabot2_piston_driver::msg::PistonSetPoint  msg_piston_set_point;
    msg_piston_set_point.position = dc_.piston_set_point_;
    msg_piston_set_point.exit = dc_.is_exit_;
    bag_writer_->write(msg_piston_set_point, "/driver/piston_set_point", t);

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
    bag_writer_->write(msg_piston, "/driver/piston", t);

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
        x_ += dt_.seconds() * f(x_, pwm);

        /// PWM motor simuluation
        if((t_-control_pwm_last_time) >= control_pwm_dt) {
            control_pwm_last_time = t_;
            pwm = control_pwm(round(dc_.piston_set_point_));
        }

        /// Pressure Simulation
        if((t_-pressure_sensor_last_time) >= pressure_sensor_dt) {
            pressure_sensor_last_time = t_;
            simulate_sensors();

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

            k_.update_density(rho_);
            k_.update_temperature(temperature_);
            k_.update_pressure(abs_pressure_/1e5);

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
                             k_.x_forcast_(3),
                             k_.x_forcast_(4),
                             k_.x_forcast_(5),
                             k_.offset_total_,
                             k_.time_last_predict_);

            /// if trhuth for dc
//            dc_.update_state(fusion_velocity_,fusion_depth_, 0.0, 0.0, Cz_, volume_total_-piston_volume_, t_);

            dc_.update_piston(piston_position_,
                              switch_top_,
                              switch_bottom_,
                              (int)DepthControl::PISTON_STATE_OK,
                                piston_last_time_);
            dc_.update_depth(fusion_depth_,
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


