#include "seabot2_simulator/simulator.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include "rosbag2_storage/storage_options.hpp"
// #include "seabot2_temperature_profile/temperature_profile.h"
#include "seabot2_msgs/msg/alpha_debug.hpp"
#include "seabot2_msgs/msg/depth_control_debug.hpp"
#include "seabot2_msgs/msg/gps_fix.hpp"
#include "seabot2_msgs/msg/piston_state.hpp"
#include "seabot2_msgs/msg/piston_set_point.hpp"
#include "seabot2_msgs/msg/power_state.hpp"
#include "seabot2_msgs/msg/pressure_sensor_data.hpp"
#include "seabot2_msgs/msg/bme280_data.hpp"
#include "seabot2_msgs/msg/profile.hpp"
#include "seabot2_msgs/msg/temperature_sensor_data.hpp"
#include "seabot2_msgs/msg/depth_control_set_point.hpp"
#include "seabot2_msgs/msg/mission_state.hpp"
#include "seabot2_msgs/msg/density.hpp"
#include "seabot2_msgs/msg/depth_pose.hpp"
#include "seabot2_msgs/msg/kalman_state.hpp"
#include "seabot2_msgs/msg/log_parameter.hpp"
#include "seabot2_msgs/msg/power_state.hpp"
#include "seabot2_msgs/msg/bme280_data.hpp"
#include "seabot2_msgs/msg/temperature_sensor_data.hpp"
#include "seabot2_msgs/msg/safety_status2.hpp"
#include "seabot2_msgs/msg/temperature_profile.hpp"
#include "seabot2_msgs/msg/simulation_debug.hpp"
#include "seabot2_msgs/msg/simulation_thermocline.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <filesystem>
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

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
                                "seabot2_msgs/msg/SimulationDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/simulation/thermocline",
                                "seabot2_msgs/msg/SimulationThermocline",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/control/alpha_debug",
                                "seabot2_msgs/msg/AlphaDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/control/depth_control_debug",
                                "seabot2_msgs/msg/DepthControlDebug",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/fix",
                                "seabot2_msgs/msg/GpsFix",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/piston",
                                "seabot2_msgs/msg/PistonState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/piston_set_point",
                                "seabot2_msgs/msg/PistonSetPoint",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/power",
                                "seabot2_msgs/msg/PowerState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/pressure_external",
                                "seabot2_msgs/msg/PressureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/pressure_internal",
                                "seabot2_msgs/msg/Bme280Data",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/profile",
                                "seabot2_msgs/msg/Profile",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/driver/temperature",
                                "seabot2_msgs/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/mission/depth_control_set_point",
                                "seabot2_msgs/msg/DepthControlSetPoint",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/mission/mission_state",
                                "seabot2_msgs/msg/MissionState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/density",
                                "seabot2_msgs/msg/Density",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/depth",
                                "seabot2_msgs/msg/DepthPose",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/kalman",
                                "seabot2_msgs/msg/KalmanState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/parameters",
                                "seabot2_msgs/msg/LogParameter",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/power",
                                "seabot2_msgs/msg/PowerState",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/pressure_internal",
                                "seabot2_msgs/msg/Bme280Data",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/temperature",
                                "seabot2_msgs/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/safety/safety",
                                "seabot2_msgs/msg/SafetyStatus2",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/temperature_profile",
                                "seabot2_msgs/msg/TemperatureProfile",
                                rmw_get_serialization_format(), ""});
    bag_writer_->create_topic( {"/observer/temperature",
                                "seabot2_msgs/msg/TemperatureSensorData",
                                rmw_get_serialization_format(), ""});

}

//double Simulator::find_index_center_thermocline(){
//    // Compute the derivative of temperature over depth
//    vector<double> dTdz;
//    for(size_t i=0; i<temperature_profile_temperature_.size()-1; i++){
//        if(temperature_profile_depth_[i+1]-temperature_profile_depth_[i] != 0.0)
//            dTdz.push_back(abs((temperature_profile_temperature_[i+1]-temperature_profile_temperature_[i])/
//                       (temperature_profile_depth_[i+1]-temperature_profile_depth_[i])));
//        else
//            dTdz.push_back(0.0);
//    }
//
//    // Find the index of the maximum value of the derivative
//    auto max = max_element(dTdz.begin(), dTdz.end());
//
//    index_center_thermocline_ = distance(dTdz.begin(), max);
//    thermocline_depth_ = temperature_profile_depth_[index_center_thermocline_];
//    return thermocline_depth_;
//}

std::array<double, 3> Simulator::compute_wave(const double t, bool water_velocity) {
    double Dz_w = 0.0; // variation of z
    double dz_w = 0.0;
    double ddz_w = 0.0;

    // Obtenir vitesse et accélération

    for(const auto & wave_generator : wave_generators_){
        if(t >= wave_generator.starting_time_
                && (wave_generator.starting_time_+wave_generator.duration_ < t || wave_generator.duration_==-1)){
//            if(!wave_generator.is_contraction_) {
                Dz_w += wave_generator.offset_ + wave_generator.amplitude_/2.0 * sin(2. * M_PI / wave_generator.period_ *
                                                                                 (t - wave_generator.starting_time_) +
                                                                                 wave_generator.phase_);
                dz_w += wave_generator.amplitude_/2.0 * 2. * M_PI / wave_generator.period_ *
                         cos(2. * M_PI / wave_generator.period_ * (t - wave_generator.starting_time_) +
                             wave_generator.phase_);

                ddz_w += - pow(wave_generator.amplitude_/2.0 * 2. * M_PI / wave_generator.period_, 2) *
                         sin(2. * M_PI / wave_generator.period_ * (t - wave_generator.starting_time_) +
                             wave_generator.phase_);
//            }

//            if(wave_generator.is_contraction_ && wave_generator.period_ !=0.)
//                dz_w += wave_generator.offset_ + (z+dz1-thermocline_depth_)*wave_generator.amplitude_ * sin(2*M_PI/wave_generator.period_*(t-wave_generator.starting_time_)+wave_generator.phase_);
        }
    }
    
    return {Dz_w, dz_w, ddz_w};
}

double Simulator::temperature_from_depth(double z) {
    double temperature;
    double Dz_w = compute_wave((t_-start_time_).seconds(), true)[0];

    if(temperature_profile_temperature_.size()<2) {
        temperature = max(min(18.0 - 0.25 * (z-Dz_w), 18.0), 8.0);
    }
    else{
        // Find first value of temperature_profile_depth_ which is greater than z
        size_t idx = 0;
        for(size_t i=idx+1; i<temperature_profile_depth_.size()-1; i++){
            if(temperature_profile_depth_[i]<(z-Dz_w))
                idx = i;
            else
                break;
        }
        const double z0 = temperature_profile_depth_[idx];
        const double z1 = temperature_profile_depth_[idx+1];
        const double t0 = temperature_profile_temperature_[idx];
        const double t1 = temperature_profile_temperature_[idx+1];
        if(z1-z0 != 0.0)
            temperature = ((z-Dz_w)-z0)/(z1-z0)*(t1-t0)+t0;
        else
            temperature = t0;
    }
    return temperature;
}

double Simulator::depth_from_temperature(const double temperature) {
    double depth;
    double Dz_w = compute_wave((t_-start_time_).seconds(), true)[0];

    if(temperature_profile_temperature_.size()<2) {
        // Linear interpolation
        depth = max(min((1./0.25)*(18.0-temperature), 40.0), 0.0);
    }
    else{
        // Find first value of temperature_profile_depth_ which is greater than z
        size_t idx = 0;
        for(size_t i=idx+1; i<temperature_profile_temperature_.size()-1; i++){
            if(temperature_profile_temperature_[i]<(temperature))
                idx = i;
            else
                break;
        }
        const double z0 = temperature_profile_depth_[idx];
        const double z1 = temperature_profile_depth_[idx+1];
        const double t0 = temperature_profile_temperature_[idx];
        const double t1 = temperature_profile_temperature_[idx+1];
        if(t1-t0 != 0.0)
            depth = ((temperature)-t0)/(t1-t0)*(z1-z0)+z0;
        else
            depth = z0;
    }
    return depth+Dz_w;
}

double Simulator::salinity_from_depth(double z) {
    salinity_ = salinity_cst_;
    return salinity_;
}

double Simulator::fz_computation(const double velocity) const {
    // Compute the drag coefficient
    // Inside the [-0.3, 0.3] range, the drag coefficient is computed using the cd_function provided by CFD analysis
    // Outside this range, the drag coefficient is assumed to be constant

    if (velocity <= fz_model_boundary_velocity_positive_ && velocity >= fz_model_boundary_velocity_negative_) {
        return 119.9 * pow(velocity, 5)
                   + 1.0928 * pow(velocity, 4)
                   - 29.224 * pow(velocity, 3)
                   - 0.0388 * pow(velocity, 2)
                   - 0.4588 * velocity;
    }
    else{
        if(velocity > fz_model_boundary_velocity_positive_)
            return velocity * fz_derivative_at_model_boundary_positive_ + fz_offset_at_model_boundary_positive_;
        else
            return velocity * fz_derivative_at_model_boundary_negative_ + fz_offset_at_model_boundary_negative_;
    }
}

double Simulator::fz_derivative_computation(const double velocity) const {
    const double velocity_clamped = std::clamp(velocity, fz_model_boundary_velocity_negative_, fz_model_boundary_velocity_positive_);
    return 5.0 * 119.9 * pow(velocity_clamped, 4)
           + 4.0 * 1.0928 * pow(velocity_clamped, 3)
           - 3.0 * 29.224 * pow(velocity_clamped, 2)
           - 2.0 * 0.0388 * velocity_clamped
           - 0.4588;
}

Matrix<double, SIMU_NB_STATES, 1> Simulator::f(const Matrix<double, SIMU_NB_STATES, 1> &x, const int pwm) {
    Matrix<double, SIMU_NB_STATES, 1> dx = Matrix<double, SIMU_NB_STATES, 1>::Zero();

    auto wave = compute_wave((t_-start_time_).seconds(), true);

    double dz_w = wave[1];
    double ddz_w = wave[2];

    double temp_K = temperature_degree_+ts.gtc.gsw_t0; /// in K
    sea_pressure_ = rho_*g_*x(4); /// ts.gsw_p_from_z(-x(4), latitude_)*1e4; /// in Pa (z is negative, output dbar)
    abs_pressure_ = sea_pressure_ + ts.gtc.gsw_p0; /// Pa
    g_ = ts.gsw_grav(latitude_, sea_pressure_*1e-4);
    rho_ = get_density_from_depth(x(4), sea_pressure_);

    const double coeff_A_ = g_ * rho_ / (robot_added_mass_ + robot_mass_);
    const double coeff_B_ = 1.0 / (robot_added_mass_ + robot_mass_);
    // double coeff_B_ = 1.0 / (robot_added_mass_ + robot_mass_);

    const double V = battery_tension_*static_cast<double>(pwm-MOTOR_STOP)/static_cast<double>(MOTOR_STOP);

    dx(0) = x(1);
    dx(1) = Kt_/J_*x(2);
    dx(2) = -Ke_/L_*x(1)-R_/L_*x(2)+V/L_;

    piston_volume_ = -(x(0)/(2*M_PI*maxon_Reduction_))*screw_thread_*(M_PI*pow(piston_diameter_/2.0, 2));
    volume_air_ = (volume_air_P0_/abs_pressure_)*(temp_K/volume_air_T0_)*volume_air_V0_;
    volume_antenna_ = min(0.0, M_PI*pow(robot_diameter_/2.0, 2)*x(4)-volume_equilibrium_); /// Volume of antenna when emerged [neg]

    const double Veq = volume_antenna_ + volume_equilibrium_;

    const double Vc = robot_mass_/rho_ + chi_ * x(3) + chi2_ * x(3)*x(3); // incompressible float if chi_/chi2_ equal 0.0

    const double rho_w = get_density_from_depth(x(4), sea_pressure_);
    const double rho_f = robot_mass_/(Vc + Veq + volume_air_ + piston_volume_);

    double dz_r = x(3)-dz_w;
    double Fz = fz_computation(dz_r);

    const double f_b = (robot_mass_/rho_) * (rho_w - rho_f)/rho_;

    dx(3) = ddz_w -coeff_A_*(f_b)+coeff_B_*Fz;
    dx(4) = x(3);

    bool one_is_nan = false;
    for(int i=0; i<SIMU_NB_STATES; i++) {
        if(isnan(dx(i))){
            one_is_nan = true;
            break;
        }
    }

    if(one_is_nan || isnan(volume_air_) || isnan(x(3)) || isnan(piston_volume_) || isnan(abs_pressure_)){
        cout << "time = " << (t_ - start_time_).seconds() << endl;
        cout << "V [motor tension] = " << V << endl;
        cout << "x(0) [theta motor] = " << x(0) << endl;
        cout << "x(1) [dtheta motor] = " << x(1) << endl;
        cout << "x(2) [current motor] = " << x(2) << endl;
        cout << "x(3) [dz] = " << x(3) << endl;
        cout << "x(4) [z] = " << x(4) << endl;
        cout << "dx(0) = " << dx(0) << endl;
        cout << "dx(1) = " << dx(1) << endl;
        cout << "dx(2) = " << dx(2) << endl;
        cout << "dx(3) = " << dx(3) << endl;
        cout << "dx(4) = " << dx(4) << endl;
        cout << "dz_w = " << dz_w << endl;
        cout << "ddz_w = " << ddz_w << endl;
        cout << "f_b = " << f_b << endl;
        cout << "Fz = " << Fz << endl;
        cout << "dz_r = " << dz_r << endl;
        cout << "rho = " << rho_ << endl;
        cout << "sea_pressure = " << sea_pressure_ << endl;
        cout << "piston_volume = " << piston_volume_ << endl;
        cout << "volume_air = " << volume_air_ << endl;
        cout << "abs_pressure = " << abs_pressure_ << endl;
        cout << "temperature = " << temperature_degree_ << endl;
        exit(EXIT_FAILURE);
    }

    return dx;
}

int Simulator::control_pwm(const int position_set_point){
    const double position = round(rad_to_tick_ * x_(0));

    motor_set_point_ = MOTOR_STOP;

    if(const int position_error = static_cast<int>(round(position_set_point-position));
        abs(position_error)>motor_regulation_dead_zone_){
        if(const int val = floor(static_cast<float>(position_error)*motor_regulation_K_);
            val>=0)
            motor_set_point_ = min(max(val, MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_UP);
        else
            motor_set_point_ = max(min(val, -MOTOR_DEAD_ZONE)+MOTOR_STOP, MOTOR_DOWN);
    }
    else{
        motor_set_point_ = MOTOR_STOP;
    }

    /// Switchs
    if(x_(0)<=0)
        switch_bottom_ = true;
    else
        switch_bottom_ = false;

    if(x_(0) * rad_to_tick_ >= piston_max_tick_)
        switch_top_ = true;
    else
        switch_top_ = false;

    if((motor_set_point_<MOTOR_STOP && switch_bottom_)
       || (motor_set_point_>MOTOR_STOP && switch_top_)){
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

    return motor_cmd_;
}

void Simulator::simulate_sensors(){
    pressure_sensor_ = abs_pressure_/1e5 + pressure_sensor_dist_(generator_); // in Bar
    fusion_depth_ = (pressure_sensor_*1e5 - ts.gtc.gsw_p0) / (g_*rho_);
//    fusion_depth_ = x_(4) + pressure_sensor_dist_(generator_);
    fusion_velocity_ = x_(3);

    temperature_sensor_ = temperature_degree_*temperature_sensor_coeff_
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

    seabot2_msgs::msg::SimulationDebug msg_simu_debug;
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

std::shared_ptr<WaypointTemperatureKeeping> wpt = mission_.get_current_waypoint_temperature_keeping();
    if(wpt!= nullptr) {
        seabot2_msgs::msg::SimulationThermocline msg_thermocline;
        msg_thermocline.temperature_target = wpt->temperature_;
        std::array<double, 3> wave = compute_wave((t - start_time_).seconds(), true);

        msg_thermocline.thermocline_depth = depth_from_temperature(msg_thermocline.temperature_target);
        msg_thermocline.thermocline_velocity = wave[1];
        msg_thermocline.thermocline_acceleration = wave[2];
        bag_writer_->write(msg_thermocline, "/simulation/thermocline", t);
    }

    seabot2_msgs::msg::DepthPose msg_depth;
    msg_depth.depth = fusion_depth_;    /// Todo verify
    msg_depth.zero_depth_pressure = 0.0;
    msg_depth.pressure = pressure_sensor_;
    msg_depth.velocity = fusion_velocity_;
    msg_depth.header.stamp = t_;
    bag_writer_->write(msg_depth, "/observer/depth", t);

    seabot2_msgs::msg::Density msg_density;
    msg_density.density = rho_;
    msg_density.header.stamp = t_;
    bag_writer_->write(msg_density, "/observer/density", t);

    seabot2_msgs::msg::MissionState msg_mission_state;
    msg_mission_state.mode = mission_.get_mission_mode();
    msg_mission_state.waypoint_id = mission_.get_current_waypoint_id();
    msg_mission_state.waypoint_length = mission_.get_number_waypoints();
    msg_mission_state.time_to_next_waypoint = mission_.get_time_to_next_waypoint(),
            msg_mission_state.header.stamp = t_;
    bag_writer_->write(msg_mission_state, "/mission/state", t);

    seabot2_msgs::msg::DepthControlSetPoint msg_depth_control_set_point;
    msg_depth_control_set_point.depth = mission_.get_depth_control_set_point().depth;
    msg_depth_control_set_point.header.stamp = t_;
    msg_depth_control_set_point.limit_velocity = mission_.get_depth_control_set_point().limit_velocity;
    msg_depth_control_set_point.enable_control = mission_.get_depth_control_set_point().enable_control;
    bag_writer_->write(msg_depth_control_set_point, "/mission/depth_control_set_point", t);

    seabot2_msgs::msg::TemperatureSensorData msg_temperature;
    msg_temperature.temperature = temperature_sensor_;
    msg_temperature.header.stamp = t_;
    bag_writer_->write(msg_temperature, "/driver/temperature", t);
    bag_writer_->write(msg_temperature, "/observer/temperature", t);

    seabot2_msgs::msg::Bme280Data msg_pressure_internal;
    msg_pressure_internal.pressure = 0.7;
    msg_pressure_internal.temperature = 20.0;
    msg_pressure_internal.humidity = 0.4;
    bag_writer_->write(msg_pressure_internal, "/driver/pressure_internal", t);

    seabot2_msgs::msg::PressureSensorData msg_pressure_external;
    msg_pressure_external.pressure = pressure_sensor_;
    msg_pressure_external.temperature = temperature_sensor_;
    msg_pressure_external.header.stamp = t_;
    bag_writer_->write(msg_pressure_external, "/driver/pressure_external", t);

    seabot2_msgs::msg::PowerState msg_power;
    msg_power.battery_volt = (float)battery_tension_;
    msg_power.cell_volt = array<float, 2>{(float)battery_tension_, 0.0};
    msg_power.esc_current = array<float, 2>{0, 0.};
    msg_power.motor_current = (float)x_(2);
    bag_writer_->write(msg_power, "/driver/power", t);

    seabot2_msgs::msg::AlphaDebug msg_alpha;
    msg_alpha.approach_velocity = dc_.approach_velocity_;
    bag_writer_->write(msg_alpha, "/control/alpha_debug", t);

    seabot2_msgs::msg::DepthControlDebug msg_control_debug;
    msg_control_debug.mode = dc_.regulation_state_;
    msg_control_debug.dy = dc_.dy_debug_;
    msg_control_debug.y = dc_.y_debug_;
    msg_control_debug.u = dc_.u_debug_;
    msg_control_debug.piston_set_point = dc_.piston_set_point_;
    bag_writer_->write(msg_control_debug, "/control/depth_control_debug", t);

    seabot2_msgs::msg::PistonSetPoint  msg_piston_set_point;
    msg_piston_set_point.position = dc_.piston_set_point_;
    msg_piston_set_point.exit = dc_.is_exit_;
    bag_writer_->write(msg_piston_set_point, "/driver/piston_set_point", t);

    seabot2_msgs::msg::KalmanState msg_kalman;
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

    seabot2_msgs::msg::PistonState msg_piston;
    msg_piston.header.stamp = t_;
    msg_piston.position = piston_position_;
    msg_piston.position_set_point = dc_.piston_set_point_;
    msg_piston.switch_top = switch_top_;
    msg_piston.switch_bottom = switch_bottom_;
    msg_piston.enable = true;
    msg_piston.motor_sens = (x_(1)>0)?true:false;
    msg_piston.state = static_cast<int>(DepthControl::PISTON_STATE_OK);
    msg_piston.motor_speed_set_point = motor_set_point_;
    msg_piston.motor_speed = motor_cmd_;
    msg_piston.battery_voltage = battery_tension_;
    msg_piston.motor_current = x_(2);
    bag_writer_->write(msg_piston, "/driver/piston", t);
}


void Simulator::write_to_file_fz() const {
    std::vector<std::array<double, 3>> data;
    for (double i = -0.5; i < 0.5; i += 0.01) {
        data.push_back({i, fz_computation(i), fz_derivative_computation(i)});
    }
    // Write to a file
    std::ofstream file_fz;
    file_fz.open("fz_computation.csv");
    for (const auto &value: data) {
        file_fz << value[0] << "," << value[1] << "," << value[2] << '\n';
    }
    file_fz.close();
}

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

    // Thermocline computation
//    std::cout << "Thermocline depth = " << find_index_center_thermocline() << std::endl;
    init_wave_file();

    const auto start = high_resolution_clock::now();
    for(t_=start_time_; t_<=end_time_; t_+=dt_) {

        /// Physical simulation
        x_ += dt_.seconds() * f(x_, pwm);
        temperature_degree_ = temperature_from_depth(x_(4));

        /// Seafloor reached
        if(x_(4)>seafloor_depth_ && x_(3)>0.0){
            x_(3)=-x_(3)*seafloor_hardness_;
        }

        /// PWM motor simuluation
        if((t_-control_pwm_last_time) >= control_pwm_dt) {
            control_pwm_last_time = t_;
            pwm = control_pwm(round(dc_.piston_set_point_));
        }

        /// Pressure and temperature Simulation
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
                              static_cast<int>(DepthControl::PISTON_STATE_OK),
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
    const auto end = high_resolution_clock::now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time exec = %ld ms", duration_cast<milliseconds>((end-start)).count());
}

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

    BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("")){
        WaveGenerator w(v.second.get<double>("amplitude", 0.0),
                        v.second.get<double>("period", 0.0),
                        v.second.get<double>("phase", 0.0),
                        v.second.get<double>("offset", 0.0),
                        v.second.get<bool>("is_contraction", false),
                        v.second.get<bool>("water_velocity", true),
                        v.second.get<double>("starting_time", 0.0),
                        v.second.get<double>("duration", 0.0));
        wave_generators_.emplace_back(w);
        cout << "Amplitude = " << w.amplitude_
             << " Period = " << w.period_
             << " Phase = " << w.phase_
             << " Offset = " << w.offset_
             << " Contraction = " << w.is_contraction_
             << " Water velocity = " << w.water_velocity_
             << " Starting time = " << w.starting_time_
             << " Duration = " << w.duration_ << endl;
    }
    return EXIT_SUCCESS;
}