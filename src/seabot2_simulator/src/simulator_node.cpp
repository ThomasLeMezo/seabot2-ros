#include "seabot2_simulator/simulator_node.hpp"

using namespace std::placeholders;

SimulatorNode::SimulatorNode()
        : Node("simulator_node"), s_(){

    RCLCPP_INFO(this->get_logger(), "[Simulator_node] Init node simulation");
    init_parameters();
    s_.run_simulation();

    RCLCPP_INFO(this->get_logger(), "[Simulator_node] Simulation ended");
    exit(EXIT_SUCCESS);
}

void SimulatorNode::init_parameters() {
    this->declare_parameter<double>("simu_robot_mass", s_.robot_mass_);
    this->declare_parameter<double>("simu_salinity", s_.salinity_cst_);
    this->declare_parameter<double>("simu_cz", s_.Cz_);
    this->declare_parameter<double>("simu_chi", s_.chi_);
    this->declare_parameter<double>("simu_chi2", s_.chi2_);
    this->declare_parameter<double>("simu_volume_air_init", s_.volume_air_init_);
    this->declare_parameter<double>("simu_pressure_sensor_stddev", s_.pressure_sensor_stddev_);
    this->declare_parameter<double>("simu_seafloor_depth", s_.seafloor_depth_);

    this->declare_parameter<double>("kalman_robot_mass", s_.k_.robot_mass_);
    this->declare_parameter<double>("kalman_enable_kalman_depth", s_.k_.enable_kalman_depth_);
    this->declare_parameter<double>("kalman_piston_volume_eq_init", s_.k_.piston_volume_eq_init_);
    this->declare_parameter<double>("kalman_init_chi", s_.k_.init_chi_);
    this->declare_parameter<double>("kalman_init_chi2", s_.k_.init_chi2_);
    this->declare_parameter<double>("kalman_init_cz", s_.k_.init_cz_);
    this->declare_parameter<double>("kalman_init_volume_air", s_.k_.init_volume_air_);
    this->declare_parameter<double>("kalman_gamma_alpha_velocity", s_.k_.gamma_alpha_velocity_);
    this->declare_parameter<double>("kalman_gamma_alpha_depth", s_.k_.gamma_alpha_depth_);
    this->declare_parameter<double>("kalman_gamma_alpha_offset", s_.k_.gamma_alpha_offset_);
    this->declare_parameter<double>("kalman_gamma_alpha_chi", s_.k_.gamma_alpha_chi_);
    this->declare_parameter<double>("kalman_gamma_alpha_chi2", s_.k_.gamma_alpha_chi2_);
    this->declare_parameter<double>("kalman_gamma_alpha_cz", s_.k_.gamma_alpha_cz_);
    this->declare_parameter<double>("kalman_gamma_alpha_volume_air", s_.k_.gamma_alpha_volume_air_);
    this->declare_parameter<double>("kalman_gamma_init_velocity", s_.k_.gamma_init_velocity_);
    this->declare_parameter<double>("kalman_gamma_init_depth", s_.k_.gamma_init_depth_);
    this->declare_parameter<double>("kalman_gamma_init_offset", s_.k_.gamma_init_offset_);
    this->declare_parameter<double>("kalman_gamma_init_chi", s_.k_.gamma_init_chi_);
    this->declare_parameter<double>("kalman_gamma_init_chi2", s_.k_.gamma_init_chi2_);
    this->declare_parameter<double>("kalman_gamma_init_cz", s_.k_.gamma_init_cz_);
    this->declare_parameter<double>("kalman_gamma_init_volume_air", s_.k_.gamma_init_volume_air_);
    this->declare_parameter<double>("kalman_gamma_beta_depth", s_.k_.gamma_beta_depth_);

    this->declare_parameter<double>("dc_robot_mass", s_.dc_.robot_mass_);
    this->declare_parameter<double>("dc_root_regulation", s_.dc_.root_regulation_);
    this->declare_parameter<double>("dc_limit_depth_control", s_.dc_.limit_depth_control_);
    this->declare_parameter<double>("dc_flow_piston_sink", s_.dc_.flow_piston_sink_);
    this->declare_parameter<double>("dc_piston_flow_security_percentage", s_.dc_.piston_flow_security_percentage_);
    this->declare_parameter<bool>("dc_hold_depth_enable", s_.dc_.hold_depth_enable_);
    this->declare_parameter<double>("dc_hold_depth_value_enter", s_.dc_.hold_depth_value_enter_);
    this->declare_parameter<double>("dc_hold_depth_value_exit", s_.dc_.hold_depth_value_exit_);
    this->declare_parameter<double>("dc_hold_velocity_enter", s_.dc_.hold_velocity_enter_);
    this->declare_parameter<double>("dc_hold_velocity_exit", s_.dc_.hold_velocity_exit_);
    this->declare_parameter<bool>("dc_control_filter", s_.dc_.control_filter_);
    this->declare_parameter<double>("dc_delta_velocity_lb", s_.dc_.delta_velocity_lb_);
    this->declare_parameter<double>("dc_delta_velocity_ub", s_.dc_.delta_velocity_ub_);
    this->declare_parameter<double>("dc_delta_position_lb", s_.dc_.delta_position_lb_);
    this->declare_parameter<double>("dc_delta_position_ub", s_.dc_.delta_position_ub_);

    this->declare_parameter<std::vector<double>>("temperature_profile_depth", s_.temperature_profile_depth_);
    this->declare_parameter<std::vector<double>>("temperature_profile_temp", s_.temperature_profile_temperature_);

    this->declare_parameter<std::vector<double>>("solver_velocity", s_.solver_velocity_);
    this->declare_parameter<std::vector<double>>("solver_alpha", s_.solver_alpha_);

    s_.robot_mass_ = this->get_parameter_or("simu_robot_mass", s_.robot_mass_);
    s_.salinity_cst_ = this->get_parameter_or("simu_salinity", s_.salinity_cst_);
    s_.Cz_ = this->get_parameter_or("simu_cz", s_.Cz_);
    s_.chi_ = this->get_parameter_or("simu_chi", s_.chi_);
    s_.chi2_ = this->get_parameter_or("simu_chi2", s_.chi2_);
    s_.volume_air_init_ = this->get_parameter_or("simu_volume_air_init", s_.volume_air_init_);
    s_.volume_air_nR_ = 101325.0*s_.volume_air_init_/(273.15+15.0);
    s_.pressure_sensor_stddev_ = this->get_parameter_or("simu_pressure_sensor_stddev", s_.pressure_sensor_stddev_);
    s_.mission_file_name_ = this->get_parameter_or("simu_mission_file_name", s_.mission_file_name_);
    s_.mission_path_ = this->get_parameter_or("simu_mission_path", s_.mission_path_);
    s_.bag_path_ = this->get_parameter_or("bag_path", s_.bag_path_);
    s_.seafloor_depth_ = this->get_parameter_or("simu_seafloor_depth", s_.seafloor_depth_);

    s_.k_.robot_mass_ = this->get_parameter_or("kalman_robot_mass", s_.k_.robot_mass_);
    s_.k_.enable_kalman_depth_ = this->get_parameter_or("kalman_enable_kalman_depth", s_.k_.enable_kalman_depth_);
    s_.k_.piston_volume_eq_init_ = this->get_parameter_or("kalman_piston_volume_eq_init", s_.k_.piston_volume_eq_init_);
    s_.k_.init_chi_ = this->get_parameter_or("kalman_init_chi", s_.k_.init_chi_);
    s_.k_.init_chi2_ = this->get_parameter_or("kalman_init_chi2", s_.k_.init_chi2_);
    s_.k_.init_cz_ = this->get_parameter_or("kalman_init_cz", s_.k_.init_cz_);
    s_.k_.init_volume_air_ = this->get_parameter_or("kalman_init_volume_air", s_.k_.init_volume_air_);
    s_.k_.gamma_alpha_velocity_ = this->get_parameter_or("kalman_gamma_alpha_velocity", s_.k_.gamma_alpha_velocity_);
    s_.k_.gamma_alpha_depth_ = this->get_parameter_or("kalman_gamma_alpha_depth", s_.k_.gamma_alpha_depth_);
    s_.k_.gamma_alpha_offset_ = this->get_parameter_or("kalman_gamma_alpha_offset", s_.k_.gamma_alpha_offset_);
    s_.k_.gamma_alpha_chi_ = this->get_parameter_or("kalman_gamma_alpha_chi", s_.k_.gamma_alpha_chi_);
    s_.k_.gamma_alpha_chi2_ = this->get_parameter_or("kalman_gamma_alpha_chi2", s_.k_.gamma_alpha_chi2_);
    s_.k_.gamma_alpha_cz_ = this->get_parameter_or("kalman_gamma_alpha_cz", s_.k_.gamma_alpha_cz_);
    s_.k_.gamma_alpha_volume_air_ = this->get_parameter_or("kalman_gamma_alpha_volume_air", s_.k_.gamma_alpha_volume_air_);
    s_.k_.gamma_init_velocity_ = this->get_parameter_or("kalman_gamma_init_velocity", s_.k_.gamma_init_velocity_);
    s_.k_.gamma_init_depth_ = this->get_parameter_or("kalman_gamma_init_depth", s_.k_.gamma_init_depth_);
    s_.k_.gamma_init_offset_ = this->get_parameter_or("kalman_gamma_init_offset", s_.k_.gamma_init_offset_);
    s_.k_.gamma_init_chi_ = this->get_parameter_or("kalman_gamma_init_chi", s_.k_.gamma_init_chi_);
    s_.k_.gamma_init_chi2_ = this->get_parameter_or("kalman_gamma_init_chi2", s_.k_.gamma_init_chi2_);
    s_.k_.gamma_init_cz_ = this->get_parameter_or("kalman_gamma_init_cz", s_.k_.gamma_init_cz_);
    s_.k_.gamma_init_volume_air_ = this->get_parameter_or("kalman_gamma_init_volume_air", s_.k_.gamma_init_volume_air_);
    s_.k_.gamma_beta_depth_ = this->get_parameter_or("kalman_gamma_beta_depth", s_.k_.gamma_beta_depth_);

    s_.dc_.robot_mass_ = this->get_parameter_or("dc_robot_mass", s_.dc_.robot_mass_);
    s_.dc_.root_regulation_ = this->get_parameter_or("dc_root_regulation", s_.dc_.root_regulation_);
    s_.dc_.limit_depth_control_ = this->get_parameter_or("dc_limit_depth_control", s_.dc_.limit_depth_control_);
    s_.dc_.flow_piston_sink_ = this->get_parameter_or("dc_flow_piston_sink", s_.dc_.flow_piston_sink_);
    s_.dc_.piston_flow_security_percentage_ = this->get_parameter_or("dc_piston_flow_security_percentage", s_.dc_.piston_flow_security_percentage_);
    s_.dc_.hold_depth_enable_ = this->get_parameter_or("dc_hold_depth_enable", s_.dc_.hold_depth_enable_);
    s_.dc_.hold_depth_value_enter_ = this->get_parameter_or("dc_hold_depth_value_enter", s_.dc_.hold_depth_value_enter_);
    s_.dc_.hold_depth_value_exit_ = this->get_parameter_or("dc_hold_depth_value_exit", s_.dc_.hold_depth_value_exit_);
    s_.dc_.hold_velocity_enter_ = this->get_parameter_or("dc_hold_velocity_enter", s_.dc_.hold_velocity_enter_);
    s_.dc_.hold_velocity_exit_ = this->get_parameter_or("dc_hold_velocity_exit", s_.dc_.hold_velocity_exit_);
    s_.dc_.control_filter_ = this->get_parameter_or("dc_control_filter", s_.dc_.control_filter_);
    s_.dc_.delta_velocity_lb_ = this->get_parameter_or("dc_delta_velocity_lb", s_.dc_.delta_velocity_lb_);
    s_.dc_.delta_velocity_ub_ = this->get_parameter_or("dc_delta_velocity_ub", s_.dc_.delta_velocity_ub_);
    s_.dc_.delta_position_lb_ = this->get_parameter_or("dc_delta_position_lb", s_.dc_.delta_position_lb_);
    s_.dc_.delta_position_ub_ = this->get_parameter_or("dc_delta_position_ub", s_.dc_.delta_position_ub_);

    s_.temperature_profile_depth_ = this->get_parameter_or("temperature_profile_depth", s_.temperature_profile_depth_);
    s_.temperature_profile_temperature_ = this->get_parameter_or("temperature_profile_temp", s_.temperature_profile_temperature_);

    s_.solver_velocity_ = this->get_parameter_or("solver_velocity", s_.solver_velocity_);
    s_.solver_alpha_ = this->get_parameter_or("solver_alpha", s_.solver_alpha_);

    if(s_.solver_velocity_.size()!=0){
        for(size_t i=0; i<s_.solver_velocity_.size(); i++)
            s_.dc_.alpha_solver_.add_to_memory(s_.solver_alpha_[i], s_.solver_velocity_[i]);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();

    return 0;
}