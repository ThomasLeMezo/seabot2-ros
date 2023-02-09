#include "seabot2_wtf/wtf_node.hpp"
#include "sys/sysinfo.h"
#include <ctime>
#include <iomanip>
#include <unistd.h>

using namespace placeholders;

WtfNode::WtfNode()
        : Node("wtf_node"){
    init_parameters();
    init_interfaces();

    char hostname[40];
    gethostname(hostname, 40);
    hostname_ = hostname;

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&WtfNode::timer_callback, this));

    initscr();
    windows_robot_              = subwin(stdscr, 5, 88, 0, 0);
    windows_safety_             = subwin(stdscr, 14, 44, 5, 0);
    windows_internal_pressure_  = subwin(stdscr, 7, 44, 19, 0);
    windows_power_              = subwin(stdscr, 13, 44, 26, 0);
    windows_depth_control_      = subwin(stdscr, 11, 44, 39, 0);
    windows_mission_            = subwin(stdscr, 15, 44, 5, 45);
    windows_depth_              = subwin(stdscr, 8, 44, 20, 45);
    windows_piston_             = subwin(stdscr, 15, 44, 28, 45);

    box(windows_robot_, ACS_VLINE, ACS_HLINE);
    box(windows_safety_, ACS_VLINE, ACS_HLINE);
    box(windows_internal_pressure_, ACS_VLINE, ACS_HLINE);
    box(windows_power_, ACS_VLINE, ACS_HLINE);
    box(windows_depth_control_, ACS_VLINE, ACS_HLINE);
    box(windows_depth_, ACS_VLINE, ACS_HLINE);
    box(windows_piston_, ACS_VLINE, ACS_HLINE);
    box(windows_mission_, ACS_VLINE, ACS_HLINE);

    mvwprintw(windows_robot_, 1, 1, "SEABOT");
    mvwprintw(windows_safety_, 1, 1, "SAFETY");
    mvwprintw(windows_internal_pressure_, 1, 1, "INTERNAL PRESSURE");
    mvwprintw(windows_power_, 1, 1, "POWER");
    mvwprintw(windows_depth_control_, 1, 1, "DEPTH CONTROL");
    mvwprintw(windows_depth_, 1, 1, "DEPTH");
    mvwprintw(windows_piston_, 1, 1, "PISTON");
    mvwprintw(windows_mission_, 1, 1, "MISSION");

    refresh();
}

WtfNode::~WtfNode(){
    endwin();
//    free(windows_safety_);
//    free(windows_internal_pressure_);
//    free(windows_power_);
//    free(windows_depth_);
//    free(windows_piston_);
//    free(windows_mission_);
}

void WtfNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void WtfNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    msg_depth_data_ = msg;
    time_last_depth_data_ = this->now();
    msg_first_received_depth_data_ = true;
}

void WtfNode::internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg){
    msg_internal_sensor_filter_ = msg;
    time_last_internal_sensor_filter_ = this->now();
    msg_first_received_internal_sensor_filter_ = true;
}

void WtfNode::power_callback(const seabot2_power_driver::msg::PowerState &msg){
    msg_power_data_ = msg;
    time_last_power_data_ = this->now();
    msg_first_received_power_data_ = true;
}

void WtfNode::piston_callback(const seabot2_piston_driver::msg::PistonState &msg){
    msg_piston_data_ = msg;
    time_last_piston_data_ = this->now();
    msg_first_received_piston_data_ = true;
}

void WtfNode::safety_callback(const seabot2_safety::msg::SafetyStatus &msg){
    msg_safety_ = msg;
    time_last_safety_ = this->now();
    msg_first_received_safety_ = true;
}

void WtfNode::waypoint_callback(const seabot2_mission::msg::Waypoint &msg){
    msg_waypoint_ = msg;
    time_last_waypoint_ = this->now();
    msg_first_received_waypoint_ = true;
}

void WtfNode::depth_control_callback(const seabot2_depth_control::msg::DepthControlDebug &msg){
    msg_depth_control_ = msg;
    time_last_depth_control_ = this->now();
    msg_first_received_depth_control_ = true;
}

void WtfNode::init_interfaces() {

    subscriber_safety_ = this->create_subscription<seabot2_safety::msg::SafetyStatus>(
            "/safety/safety", 10, std::bind(&WtfNode::safety_callback, this, _1));

    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&WtfNode::depth_callback, this, _1));

    subscriber_internal_sensor_filter_ = this->create_subscription<pressure_bme280_driver::msg::Bme280Data>(
            "/observer/pressure_internal", 10, std::bind(&WtfNode::internal_sensor_callback, this, _1));

    subscriber_power_data_ = this->create_subscription<seabot2_power_driver::msg::PowerState>(
            "/observer/power", 10, std::bind(&WtfNode::power_callback, this, _1));

    subscriber_piston_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/piston", 10, std::bind(&WtfNode::piston_callback, this, _1));

    subscriber_mission_ = this->create_subscription<seabot2_mission::msg::Waypoint>(
            "/mission/waypoint", 10, std::bind(&WtfNode::waypoint_callback, this, _1));

    subscriber_control_debug_ = this->create_subscription<seabot2_depth_control::msg::DepthControlDebug>(
            "/control/depth_control_debug", 10, std::bind(&WtfNode::depth_control_callback, this, _1));
}

void WtfNode::update_internal_pressure_windows(){
    if(msg_first_received_internal_sensor_filter_) {
        mvwprintw(windows_internal_pressure_, 1, 30, to_string((this->now() - time_last_internal_sensor_filter_).seconds()).c_str());

        mvwprintw(windows_internal_pressure_, 3, 1, "pressure");
        mvwprintw(windows_internal_pressure_, 3, 30, to_string(msg_internal_sensor_filter_.pressure).c_str());

        mvwprintw(windows_internal_pressure_, 4, 1, "temperature");
        mvwprintw(windows_internal_pressure_, 4, 30, to_string(msg_internal_sensor_filter_.temperature).c_str());

        mvwprintw(windows_internal_pressure_, 5, 1, "humidity");
        mvwprintw(windows_internal_pressure_, 5, 30, to_string(msg_internal_sensor_filter_.humidity).c_str());

        wrefresh(windows_internal_pressure_);
    }
}

void WtfNode::update_mission_windows(){
    if(msg_first_received_waypoint_) {
        mvwprintw(windows_mission_, 1, 30, to_string((this->now() - time_last_waypoint_).seconds()).c_str());

        mvwprintw(windows_mission_, 3, 1, "north");
        mvwprintw(windows_mission_, 3, 30, to_string(msg_waypoint_.north).c_str());

        mvwprintw(windows_mission_, 4, 1, "east");
        mvwprintw(windows_mission_, 4, 30, to_string(msg_waypoint_.east).c_str());

        mvwprintw(windows_mission_, 5, 1, "depth");
        mvwprintw(windows_mission_, 5, 30, to_string(msg_waypoint_.depth).c_str());

        mvwprintw(windows_mission_, 6, 1, "limit_velocity");
        mvwprintw(windows_mission_, 6, 30, to_string(msg_waypoint_.limit_velocity).c_str());

        mvwprintw(windows_mission_, 7, 1, "approach_velocity");
        mvwprintw(windows_mission_, 7, 30, to_string(msg_waypoint_.approach_velocity).c_str());

        mvwprintw(windows_mission_, 8, 1, "mission_enable");
        mvwprintw(windows_mission_, 8, 30, msg_waypoint_.mission_enable ? "True " : "False");

        mvwprintw(windows_mission_, 9, 1, "enable_thrusters");
        mvwprintw(windows_mission_, 9, 30, msg_waypoint_.enable_thrusters ? "True " : "False");

        mvwprintw(windows_mission_, 10, 1, "waypoint_id");
        mvwprintw(windows_mission_, 10, 30, to_string(msg_waypoint_.waypoint_id).c_str());

        mvwprintw(windows_mission_, 11, 1, "waypoint_length");
        mvwprintw(windows_mission_, 11, 30, to_string(msg_waypoint_.waypoint_length).c_str());

        mvwprintw(windows_mission_, 12, 1, "time_to_next_waypoint");
        mvwprintw(windows_mission_, 12, 30, to_string(msg_waypoint_.time_to_next_waypoint).c_str());

        mvwprintw(windows_mission_, 13, 1, "seafloor_landing");
        mvwprintw(windows_mission_, 13, 30, msg_waypoint_.seafloor_landing  ? "True " : "False");

        wrefresh(windows_mission_);
    }
}

void WtfNode::update_safety_windows(){
    if(msg_first_received_safety_) {
        mvwprintw(windows_safety_, 1, 30, to_string((this->now() - time_last_safety_).seconds()).c_str());

        mvwprintw(windows_safety_, 3, 1, "global_safety_valid");
        mvwprintw(windows_safety_, 3, 30, msg_safety_.global_safety_valid ? "True " : "False");

        mvwprintw(windows_safety_, 4, 1, "published_frequency");
        mvwprintw(windows_safety_, 4, 30, msg_safety_.published_frequency ? "True " : "False");

        mvwprintw(windows_safety_, 5, 1, "depth_limit");
        mvwprintw(windows_safety_, 5, 30, msg_safety_.depth_limit ? "True " : "False");

        mvwprintw(windows_safety_, 6, 1, "batteries_limit");
        mvwprintw(windows_safety_, 6, 30, msg_safety_.batteries_limit ? "True " : "False");

        mvwprintw(windows_safety_, 7, 1, "depressurization");
        mvwprintw(windows_safety_, 7, 30, msg_safety_.depressurization ? "True " : "False");

        mvwprintw(windows_safety_, 8, 1, "seafloor");
        mvwprintw(windows_safety_, 8, 30, msg_safety_.seafloor ? "True " : "False");

        mvwprintw(windows_safety_, 9, 1, "piston");
        mvwprintw(windows_safety_, 9, 30, msg_safety_.piston ? "True " : "False");

        mvwprintw(windows_safety_, 10, 1, "zero_depth");
        mvwprintw(windows_safety_, 10, 30, msg_safety_.zero_depth ? "True " : "False");

        mvwprintw(windows_safety_, 11, 1, "cpu");
        mvwprintw(windows_safety_, 11, 30, to_string(msg_safety_.cpu).c_str());

        mvwprintw(windows_safety_, 12, 1, "ram");
        mvwprintw(windows_safety_, 12, 30, to_string((int)msg_safety_.ram).c_str());

        wrefresh(windows_safety_);
    }
}

void WtfNode::update_power(){
    if(msg_first_received_power_data_) {
        mvwprintw(windows_power_, 1, 30, to_string((this->now() - time_last_power_data_).seconds()).c_str());

        mvwprintw(windows_power_, 3, 1, "cell_volt[0]");
        mvwprintw(windows_power_, 3, 30, to_string(msg_power_data_.cell_volt[0]).c_str());

        mvwprintw(windows_power_, 4, 1, "cell_volt[1]");
        mvwprintw(windows_power_, 4, 30, to_string(msg_power_data_.cell_volt[1]).c_str());

        mvwprintw(windows_power_, 5, 1, "cell_volt[2]");
        mvwprintw(windows_power_, 5, 30, to_string(msg_power_data_.cell_volt[2]).c_str());

        mvwprintw(windows_power_, 6, 1, "cell_volt[3]");
        mvwprintw(windows_power_, 6, 30, to_string(msg_power_data_.cell_volt[3]).c_str());

        mvwprintw(windows_power_, 7, 1, "battery_volt");
        mvwprintw(windows_power_, 7, 30, to_string(msg_power_data_.battery_volt).c_str());

        mvwprintw(windows_power_, 8, 1, "esc_current[0]");
        mvwprintw(windows_power_, 8, 30, to_string(msg_power_data_.esc_current[0]).c_str());

        mvwprintw(windows_power_, 9, 1, "esc_current[1]");
        mvwprintw(windows_power_, 9, 30, to_string(msg_power_data_.esc_current[1]).c_str());

        mvwprintw(windows_power_, 10, 1, "motor_current");
        mvwprintw(windows_power_, 10, 30, to_string(msg_power_data_.motor_current).c_str());

        mvwprintw(windows_power_, 11, 1, "power_state");
        mvwprintw(windows_power_, 11, 30, to_string(msg_power_data_.power_state).c_str());

        wrefresh(windows_power_);
    }
}

void WtfNode::update_depth(){
    if(msg_first_received_depth_data_) {
        mvwprintw(windows_depth_, 1, 30, to_string((this->now() - time_last_depth_data_).seconds()).c_str());

        mvwprintw(windows_depth_, 3, 1, "depth");
        mvwprintw(windows_depth_, 3, 30, to_string(msg_depth_data_.depth).c_str());

        mvwprintw(windows_depth_, 4, 1, "velocity");
        mvwprintw(windows_depth_, 4, 30, to_string(msg_depth_data_.velocity).c_str());

        mvwprintw(windows_depth_, 5, 1, "zero_depth_pressure");
        mvwprintw(windows_depth_, 5, 30, to_string(msg_depth_data_.zero_depth_pressure).c_str());

        mvwprintw(windows_depth_, 6, 1, "pressure");
        mvwprintw(windows_depth_, 6, 30, to_string(msg_depth_data_.pressure).c_str());

        wrefresh(windows_depth_);
    }
}

void WtfNode::update_piston(){
    if(msg_first_received_piston_data_) {
        mvwprintw(windows_piston_, 1, 30, to_string((this->now() - time_last_piston_data_).seconds()).c_str());

        mvwprintw(windows_piston_, 3, 1, "position");
        mvwprintw(windows_piston_, 3, 30, to_string(msg_piston_data_.position).c_str());

        mvwprintw(windows_piston_, 4, 1, "position_set_point");
        mvwprintw(windows_piston_, 4, 30, to_string(msg_piston_data_.position_set_point).c_str());

        mvwprintw(windows_piston_, 5, 1, "switch_top");
        mvwprintw(windows_piston_, 5, 30, msg_piston_data_.switch_top? "True " : "False");

        mvwprintw(windows_piston_, 6, 1, "switch_bottom");
        mvwprintw(windows_piston_, 6, 30, msg_piston_data_.switch_bottom? "True " : "False");

        mvwprintw(windows_piston_, 7, 1, "enable");
        mvwprintw(windows_piston_, 7, 30, msg_piston_data_.enable? "True " : "False");

        mvwprintw(windows_piston_, 8, 1, "motor_sens");
        mvwprintw(windows_piston_, 8, 30, msg_piston_data_.motor_sens? "True " : "False");

        mvwprintw(windows_piston_, 9, 1, "state");
        mvwprintw(windows_piston_, 9, 30, to_string(msg_piston_data_.state).c_str());

        mvwprintw(windows_piston_, 10, 1, "motor_speed_set_point");
        mvwprintw(windows_piston_, 10, 30, to_string(msg_piston_data_.motor_speed_set_point).c_str());

        mvwprintw(windows_piston_, 11, 1, "motor_speed");
        mvwprintw(windows_piston_, 11, 30, to_string(msg_piston_data_.motor_speed).c_str());

        mvwprintw(windows_piston_, 12, 1, "battery_voltage");
        mvwprintw(windows_piston_, 12, 30, to_string(msg_piston_data_.battery_voltage).c_str());

        mvwprintw(windows_piston_, 13, 1, "motor_current");
        mvwprintw(windows_piston_, 13, 30, to_string(msg_piston_data_.motor_current).c_str());

        wrefresh(windows_piston_);
    }
}

void WtfNode::update_depth_control(){
    if(msg_first_received_depth_control_) {
        mvwprintw(windows_depth_control_, 1, 30, to_string((this->now() - time_last_depth_control_).seconds()).c_str());

        mvwprintw(windows_depth_control_, 3, 1, "position");
        mvwprintw(windows_depth_control_, 3, 30, to_string(msg_piston_data_.position).c_str());

        mvwprintw(windows_depth_control_, 4, 1, "u");
        mvwprintw(windows_depth_control_, 4, 30, to_string(msg_depth_control_.u).c_str());

        mvwprintw(windows_depth_control_, 5, 1, "y");
        mvwprintw(windows_depth_control_, 5, 30, to_string(msg_depth_control_.y).c_str());

        mvwprintw(windows_depth_control_, 6, 1, "dy");
        mvwprintw(windows_depth_control_, 6, 30, to_string(msg_depth_control_.dy).c_str());

        mvwprintw(windows_depth_control_, 7, 1, "piston_set_point");
        mvwprintw(windows_depth_control_, 7, 30, to_string(msg_depth_control_.piston_set_point).c_str());

        mvwprintw(windows_depth_control_, 8, 1, "mode");
        mvwprintw(windows_depth_control_, 8, 30, to_string(msg_depth_control_.mode).c_str());

        wrefresh(windows_depth_control_);
    }
}

void WtfNode::update_robot(){

    mvwprintw(windows_robot_, 1, 20, to_string((this->now()).seconds()).c_str());

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    stringstream ss;
    ss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    mvwprintw(windows_robot_, 1, 40, ss.str().c_str());
    mvwprintw(windows_robot_, 2, 2, hostname_.c_str());

    wrefresh(windows_robot_);
}

void WtfNode::timer_callback() {
    update_safety_windows();
    update_mission_windows();
    update_internal_pressure_windows();
    update_power();
    update_depth();
    update_piston();
    update_robot();
    update_depth_control();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WtfNode>());
    rclcpp::shutdown();
    return 0;
}