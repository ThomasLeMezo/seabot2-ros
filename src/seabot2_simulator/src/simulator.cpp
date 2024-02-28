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
#include "seabot2_mission/msg/depth_control_set_point.hpp"
#include "seabot2_mission/msg/mission_state.hpp"
#include "seabot2_density/msg/density.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "seabot2_kalman/msg/kalman_state.hpp"
#include "seabot2_log_parameters/msg/log_parameter.hpp"
#include "seabot2_power_driver/msg/power_state.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "seabot2_safety/msg/safety_status.hpp"
#include "seabot2_temperature_profile/temperature_profile.h"
#include "seabot2_temperature_profile/msg/temperature_profile.hpp"

#include "seabot2_simulator/msg/simulation_debug.hpp"

using namespace std::chrono;

double Simulator::get_density_from_depth(double z, double sea_pressure) {
    return ts.gsw_rho_t_exact(this->salinity_from_depth(z),
                              this->temperature_from_depth(z),
                              sea_pressure*1e-4);
}

Simulator::Simulator():
        ts(),
        k_(),
        tp_(),
        dc_(rclcpp::Time(0., RCL_ROS_TIME)),
        mission_()
{

}

void Simulator::compute_std_generators(){
    pressure_sensor_dist_= std::normal_distribution<double>{pressure_sensor_mean_, pressure_sensor_stddev_};
    temperature_sensor_dist_ = std::normal_distribution<double>{0.0, temperature_sensor_stddev_};
}

void Simulator::init_bag_writer(){
    bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options({bag_path_, "mcap"});;

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
    bag_writer_->create_topic( {"/mission/depth_control_set_point",
                                "seabot2_mission/msg/DepthControlSetPoint",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/mission/mission_state",
                                "seabot2_mission/msg/MissionState",
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
    bag_writer_->create_topic( {"/observer/temperature_profile",
                                "seabot2_temperature_profile/msg/TemperatureProfile",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/temperature",
                                "temperature_tsys01_driver/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});

}

double Simulator::find_index_center_thermocline(){
    // Compute the derivative of temperature over depth
    vector<double> dTdz;
    for(size_t i=0; i<temperature_profile_temperature_.size()-1; i++){
        if(temperature_profile_depth_[i+1]-temperature_profile_depth_[i] != 0.0)
            dTdz.push_back(abs((temperature_profile_temperature_[i+1]-temperature_profile_temperature_[i])/
                       (temperature_profile_depth_[i+1]-temperature_profile_depth_[i])));
        else
            dTdz.push_back(0.0);
    }

    // Find the index of the maximum value of the derivative
    auto max = max_element(dTdz.begin(), dTdz.end());

    index_center_thermocline_ = distance(dTdz.begin(), max);
    thermocline_depth_ = temperature_profile_depth_[index_center_thermocline_];
    return thermocline_depth_;
}

double Simulator::compute_wave(double t, double z) {
    double dz1 = 0.0;
    for(auto & wave_generator : wave_generators_){
        if(wave_generator.starting_time_<=t && (t<=wave_generator.starting_time_+wave_generator.duration_ || wave_generator.duration_==-1)){
            if(!wave_generator.is_contraction_)
                dz1 += wave_generator.offset_ + wave_generator.amplitude_ * sin(2.*M_PI/wave_generator.period_*(t-wave_generator.starting_time_)+wave_generator.phase_);
        }
    }

    double dz2 = 0.0;
    for(auto & wave_generator : wave_generators_){
        if(wave_generator.starting_time_<=t && (t<=wave_generator.starting_time_+wave_generator.duration_ || wave_generator.duration_==-1)){
            if(wave_generator.is_contraction_)
                dz2 += wave_generator.offset_ + (z+dz1-thermocline_depth_)*wave_generator.amplitude_ * sin(2*M_PI/wave_generator.period_*(t-wave_generator.starting_time_)+wave_generator.phase_);
        }
    }

    return dz1+dz2;
}

double Simulator::temperature_from_depth(double z) {
    double temperature;
    if(temperature_profile_temperature_.size()<2) {
        temperature = max(min(18.0 - 0.25 * z, 18.0), 8.0);
    }
    else{
        double dz = compute_wave((t_-start_time_).seconds(), z);
        // Find first value of temperature_profile_depth_ which is greater than z
        size_t idx = 0;
        for(size_t i=idx+1; i<temperature_profile_depth_.size(); i++){
            if(temperature_profile_depth_[i]<(z+dz))
                idx = i;
            else
                break;
        }
        double z0 = temperature_profile_depth_[idx];
        double z1 = temperature_profile_depth_[idx+1];
        double t0 = temperature_profile_temperature_[idx];
        double t1 = temperature_profile_temperature_[idx+1];
        if(z1-z0 != 0.0)
            temperature = (z-z0)/(z1-z0)*(t1-t0)+t0;
        else
            temperature = t0;
    }
    return temperature;
}

double Simulator::salinity_from_depth(double z) {
    salinity_ = salinity_cst_;
    return salinity_;
}

Matrix<double, SIMU_NB_STATES, 1> Simulator::f(const Matrix<double, SIMU_NB_STATES, 1> &x, int pwm) {
    Matrix<double, SIMU_NB_STATES, 1> dx = Matrix<double, SIMU_NB_STATES, 1>::Zero();

    double temp_K = temperature_from_depth(x(4))+ts.gtc.gsw_t0; /// in K
    sea_pressure_ = rho_*g_*x(4); /// ts.gsw_p_from_z(-x(4), latitude_)*1e4; /// in Pa (z is negative, output dbar)
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

    if(isnan(volume_air_) || isnan(x(3)) || isnan(piston_volume_) || isnan(abs_pressure_)){
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

    int position_error = static_cast<int>(round(position_set_point-position));
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
//    fusion_depth_ = x_(4) + pressure_sensor_dist_(generator_);
    fusion_velocity_ = x_(3);

    temperature_sensor_ = temperature_from_depth(x_(4))*temperature_sensor_coeff_
                          + temperature_sensor_*(1.-temperature_sensor_coeff_)
                          + temperature_sensor_dist_(generator_temperature_);
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

    seabot2_mission::msg::MissionState msg_mission_state;
    msg_mission_state.mode = mission_.get_mission_mode();
    msg_mission_state.waypoint_id = mission_.get_current_waypoint_id();
    msg_mission_state.waypoint_length = mission_.get_number_waypoints();
    msg_mission_state.time_to_next_waypoint = mission_.get_time_to_next_waypoint(),
            msg_mission_state.header.stamp = t_;
    bag_writer_->write(msg_mission_state, "/mission/state", t);

    seabot2_mission::msg::DepthControlSetPoint msg_depth_control_set_point;
    msg_depth_control_set_point.depth = mission_.get_depth_control_set_point().depth;
    msg_depth_control_set_point.header.stamp = t_;
    msg_depth_control_set_point.limit_velocity = mission_.get_depth_control_set_point().limit_velocity;
    msg_depth_control_set_point.enable_control = mission_.get_depth_control_set_point().enable_control;
    bag_writer_->write(msg_depth_control_set_point, "/mission/depth_control_set_point", t);

    temperature_tsys01_driver::msg::TemperatureSensorData msg_temperature;
    msg_temperature.temperature = temperature_sensor_;
    msg_temperature.header.stamp = t_;
    bag_writer_->write(msg_temperature, "/driver/temperature", t);
    bag_writer_->write(msg_temperature, "/observer/temperature", t);

    pressure_bme280_driver::msg::Bme280Data msg_pressure_internal;
    msg_pressure_internal.pressure = 0.7;
    msg_pressure_internal.temperature = 20.0;
    msg_pressure_internal.humidity = 0.4;
    bag_writer_->write(msg_pressure_internal, "/driver/pressure_internal", t);

    seabot2_depth_filter::msg::PressureSensorData msg_pressure_external;
    msg_pressure_external.pressure = pressure_sensor_;
    msg_pressure_external.temperature = temperature_sensor_;
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

#include <filesystem>
namespace fs = std::filesystem;

void Simulator::run_simulation() {
    int pwm = MOTOR_STOP;

    for (const auto & entry : fs::directory_iterator(mission_path_)) {
        if (!entry.is_directory()) {
            std::cout << entry.path() << std::endl;
            if (entry.path().extension() == ".xml") {
                mission_file_name_ = entry.path().filename();
                bag_path_ = entry.path().stem();
            }

            if(entry.path().extension() == ".wave"){
                wave_file_name_ = entry.path().filename();
            }
        }
    }

    init_bag_writer();
    mission_.load_mission(mission_file_name_, mission_path_);
    start_time_ = mission_.get_start_time() - mission_delay_before_start_;
    end_time_ = mission_.get_end_time() + mission_delay_after_end_;

    k_.init_parameters(start_time_);
    dc_.set_start_time(start_time_);
    temperature_sensor_ = temperature_from_depth(x_(4));

    // Thermocline computation
    std::cout << "Thermocline depth = " << find_index_center_thermocline() << std::endl;
    init_wave_file();

    auto start = high_resolution_clock::now();
    for(t_=start_time_; t_<=end_time_; t_+=dt_) {

        /// Physical simulation
        x_ += dt_.seconds() * f(x_, pwm);

        /// Seafloor reached
        if(x_(4)>seafloor_depth_ && x_(3)>0.0){
            x_(3)=-x_(3)*seafloor_hardness_;
        }

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
            k_.update_temperature(temperature_sensor_);
            k_.update_pressure(abs_pressure_/1e5);

            /// Compute Kalman
            k_.set_new_depth_data(fusion_depth_, fusion_velocity_, t_);
        }

        /// Piston simulation
        if((t_-piston_last_time_)>= piston_dt_){
            piston_last_time_ = t_;
            simulate_piston_position();

            k_.update_density(rho_);
            k_.update_temperature(temperature_sensor_);
            k_.update_pressure(abs_pressure_/1e5);

            /// Compute Kalman
            k_.set_new_piston_data(piston_position_, dc_.piston_set_point_, t_);
        }

        if((t_-temperature_last_time_)>= temperature_dt_){
            temperature_last_time_ = t_;
            /// Temperature profile simulation
            tp_.update_temperature(temperature_sensor_, fusion_depth_);
            mission_.update_temperature(temperature_sensor_);
        }

        /// Mission simulation
        if(t_-mission_last_time_>= mission_dt_){
            mission_last_time_ = t_;
            mission_.update_depth(k_.x_forcast_(1));
            mission_.update_state(t_);
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
            dc_.update_waypoint(mission_.get_depth_control_set_point().depth,
                                mission_.get_depth_control_set_point().limit_velocity,
                                t_,
                                mission_.get_depth_control_set_point().enable_control);
            dc_.update_density(rho_);
            dc_.update_temperature(temperature_sensor_);

            dc_.state_machine_step(dc_dt_, t_);
        }

        if((t_-memory_last_time)>=memory_dt) {
            memory_last_time = t_;
            save_data(t_);
        }
        nb_steps++;
    }
    auto end = high_resolution_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time exec = %ld ms", duration_cast<milliseconds>((end-start)).count());
}

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
namespace pt = boost::property_tree;

int Simulator::init_wave_file(){
    if(wave_file_name_.empty())
        return EXIT_FAILURE;

    pt::ptree tree;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Simulator] Read xml file : %s", wave_file_name_.c_str());
    try {
        pt::read_xml(wave_file_name_, tree);
    } catch (std::exception const&  ex) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"[Seabot_Simulator] %s", ex.what());
        return EXIT_FAILURE;
    }

    int return_code = EXIT_SUCCESS;
    BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("")){
        WaveGenerator w(v.second.get<double>("amplitude", 0.0),
                        v.second.get<double>("period", 0.0),
                        v.second.get<double>("phase", 0.0),
                        v.second.get<double>("offset", 0.0),
                        v.second.get<bool>("is_contraction", false),
                        v.second.get<double>("starting_time", 0.0),
                        v.second.get<double>("duration", 0.0));
        wave_generators_.emplace_back(w);
        cout << "Amplitude = " << w.amplitude_ << " Period = " << w.period_ << " Phase = " << w.phase_
             << " Offset = " << w.offset_
             << " Contraction = " << w.is_contraction_ << " Starting time = " << w.starting_time_
             << " Duration = " << w.duration_ << endl;
    }
    return return_code;
}