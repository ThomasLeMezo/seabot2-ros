#include "seabot2_wtf/wtf_node.hpp"
#include <ctime>
#include <iomanip>
#include <unistd.h>
#include <limits>

using namespace placeholders;

#define COLOR_DEFAULT 1
#define COLOR_VALID 2
#define COLOR_NOT_VALID 3

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

    use_default_colors();
    start_color();
    init_pair(COLOR_DEFAULT, -1, -1);
    init_pair(COLOR_VALID, -1, COLOR_GREEN);
    init_pair(COLOR_NOT_VALID, -1, COLOR_RED);

    create_windows();
}

void WtfNode::create_windows(){
    getmaxyx(stdscr,windows_max_y_,windows_max_x_);
    /// | - - - x
    /// |
    /// y
    /// height, width, point_x, point_y
    windows_robot_              = subwin(stdscr, 4, windows_max_x_, 0, 0);
    box(windows_robot_, ACS_VLINE, ACS_HLINE);
    mvwprintw(windows_robot_, 1, 1, "SEABOT");

    windows_log_                = subwin(stdscr, 8, windows_max_x_, 4, 0);
    box(windows_log_, ACS_VLINE, ACS_HLINE);
    mvwprintw(windows_log_, 1, 1, "LOG");

    windows_default_y_ = 12; // Size of upper windows (sum of windows_robot_ and windows_log_)
    windows_current_y_ = windows_default_y_;

    windows_safety_             = create_new_sub_window(16, 40, "SAFETY");
    windows_internal_pressure_  = create_new_sub_window(7, 40, "INTERNAL PRESSURE");
    windows_power_              = create_new_sub_window(11, 40, "POWER");
    windows_depth_control_      = create_new_sub_window(10, 40, "DEPTH CONTROL");
    windows_mission_            = create_new_sub_window(14, 40, "MISSION");
    windows_depth_              = create_new_sub_window(8, 40, "DEPTH");
    windows_piston_             = create_new_sub_window(15, 40, "PISTON");
    windows_gnss_               = create_new_sub_window(8, 40, "GNSS");
    windows_sensors_            = create_new_sub_window(8, 40, "SENSORS");
    windows_audio_              = create_new_sub_window(8, 40, "AUDIO");

    refresh();
}

WtfNode::~WtfNode(){
    endwin();
}

WINDOW *WtfNode::create_new_sub_window(const int height, int width, const string &name){
    if(const int end_y = windows_current_y_ + height; end_y > windows_max_y_){
        windows_current_y_ = windows_default_y_;
        windows_current_x_ += windows_width_max_ + 1;
        windows_width_max_ = 0;
    }

    const int start_x = windows_current_x_;

    WINDOW *local_win = subwin(stdscr, height, width, windows_current_y_, start_x);

    windows_width_max_ = max(windows_width_max_, width);
    windows_current_y_ += height;

    box(local_win, ACS_VLINE, ACS_HLINE);
    mvwprintw(local_win, 1, 1, "%s", name.c_str());

    return local_win;
}

void WtfNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", static_cast<int>(loop_dt_.count()));
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void WtfNode::depth_callback(const seabot2_msgs::msg::DepthPose &msg){
    msg_depth_data_ = msg;
    time_last_depth_data_ = this->now();
    msg_first_received_depth_data_ = true;
}

void WtfNode::internal_sensor_callback(const seabot2_msgs::msg::Bme280Data &msg){
    msg_internal_sensor_filter_ = msg;
    time_last_internal_sensor_filter_ = this->now();
    msg_first_received_internal_sensor_filter_ = true;
}

void WtfNode::power_callback(const seabot2_msgs::msg::PowerState &msg){
    msg_power_data_ = msg;
    time_last_power_data_ = this->now();
    msg_first_received_power_data_ = true;
}

void WtfNode::piston_callback(const seabot2_msgs::msg::PistonState &msg){
    msg_piston_data_ = msg;
    time_last_piston_data_ = this->now();
    msg_first_received_piston_data_ = true;
}

void WtfNode::safety_callback(const seabot2_msgs::msg::SafetyStatus2 &msg){
    msg_safety_ = msg;
    time_last_safety_ = this->now();
    msg_first_received_safety_ = true;
}

void WtfNode::mission_state_callback(const seabot2_msgs::msg::MissionState &msg){
    msg_mission_state_ = msg;
    time_last_mission_state_ = this->now();
    msg_first_received_mission_state_ = true;
}

void WtfNode::depth_control_set_point_callback(const seabot2_msgs::msg::DepthControlSetPoint &msg){
    msg_depth_control_set_point_ = msg;
    time_last_depth_control_set_point_ = this->now();
    msg_first_received_depth_control_set_point = true;
}

void WtfNode::depth_control_callback(const seabot2_msgs::msg::DepthControlDebug &msg){
    msg_depth_control_ = msg;
    time_last_depth_control_ = this->now();
    msg_first_received_depth_control_ = true;
}

void WtfNode::gnss_callback(const seabot2_msgs::msg::GpsFix &msg){
    msg_gnss_ = msg;
    time_last_gnss_ = this->now();
    msg_first_received_gnss_ = true;
}

void WtfNode::profile_callback(const seabot2_msgs::msg::Profile &msg) {
    msg_profile_ = msg;
    time_last_profile_ = this->now();
    msg_first_received_profile_ = true;
}

void WtfNode::density_callback(const seabot2_msgs::msg::Density &msg) {
    msg_density_ = msg;
    time_last_density_ = this->now();
    msg_first_received_density_ = true;
}

void WtfNode::temperature_sensor_data_callback(const seabot2_msgs::msg::TemperatureSensorData &msg) {
    msg_temperature_sensor_data_ = msg;
    time_last_temperature_sensor_data_ = this->now();
    msg_first_received_temperature_sensor_data_ = true;
}

void WtfNode::log_callback(const rcl_interfaces::msg::Log &msg) {
    msg_queue_log_.push_back(msg);
    if(msg_queue_log_.size() > msg_queue_log_size_){
        msg_queue_log_.pop_front();
    }
}

void WtfNode::audio_dspic_callback(const seabot2_msgs::msg::SyncDspic &msg) {
    msg_audio_dspic_ = msg;
    time_last_audio_dspic_ = this->now();
    msg_first_received_audio_dspic_ = true;
}

void WtfNode::init_interfaces() {

    subscriber_safety_ = this->create_subscription<seabot2_msgs::msg::SafetyStatus2>(
            "/safety/safety", 10, std::bind(&WtfNode::safety_callback, this, _1));

    subscriber_depth_data_ = this->create_subscription<seabot2_msgs::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&WtfNode::depth_callback, this, _1));

    subscriber_internal_sensor_filter_ = this->create_subscription<seabot2_msgs::msg::Bme280Data>(
            "/observer/pressure_internal", 10, std::bind(&WtfNode::internal_sensor_callback, this, _1));

    subscriber_power_data_ = this->create_subscription<seabot2_msgs::msg::PowerState>(
            "/observer/power", 10, std::bind(&WtfNode::power_callback, this, _1));

    subscriber_piston_data_ = this->create_subscription<seabot2_msgs::msg::PistonState>(
            "/driver/piston", 10, std::bind(&WtfNode::piston_callback, this, _1));

    subscriber_mission_state_ = this->create_subscription<seabot2_msgs::msg::MissionState>(
            "/mission/mission_state", 10, std::bind(&WtfNode::mission_state_callback, this, _1));

    subscriber_depth_control_set_point_ = this->create_subscription<seabot2_msgs::msg::DepthControlSetPoint>(
            "/mission/depth_control_set_point", 10, std::bind(&WtfNode::depth_control_set_point_callback, this, _1));

    subscriber_control_debug_ = this->create_subscription<seabot2_msgs::msg::DepthControlDebug>(
            "/control/depth_control_debug", 10, std::bind(&WtfNode::depth_control_callback, this, _1));

    subscriber_gnss_ = this->create_subscription<seabot2_msgs::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&WtfNode::gnss_callback, this, _1));

    subscriber_profile_ = this->create_subscription<seabot2_msgs::msg::Profile>(
            "/driver/profile", 10, std::bind(&WtfNode::profile_callback, this, _1));

    subscriber_density_ = this->create_subscription<seabot2_msgs::msg::Density>(
            "/observer/density", 10, std::bind(&WtfNode::density_callback, this, _1));

    subscriber_temperature_sensor_data_ = this->create_subscription<seabot2_msgs::msg::TemperatureSensorData>(
            "/observer/temperature", 10, std::bind(&WtfNode::temperature_sensor_data_callback, this, _1));

    subscriber_log_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 10, std::bind(&WtfNode::log_callback, this, _1));

    subscriber_audio_dspic_ = this->create_subscription<seabot2_msgs::msg::SyncDspic>(
            "/driver/audio_dspic", 10, std::bind(&WtfNode::audio_dspic_callback, this, _1));
}

void WtfNode::update_internal_pressure_windows(){
    if(msg_first_received_internal_sensor_filter_) {
        mvwprintw(windows_internal_pressure_, 1, 25, "%0.2f", (this->now() - time_last_internal_sensor_filter_).seconds());

        mvwprintw(windows_internal_pressure_, 3, 1, "pressure");
        mvwprintw(windows_internal_pressure_, 3, 25, "%4.0f", msg_internal_sensor_filter_.pressure);

        mvwprintw(windows_internal_pressure_, 4, 1, "temperature");
        mvwprintw(windows_internal_pressure_, 4, 25, "%2.2f", msg_internal_sensor_filter_.temperature);

        mvwprintw(windows_internal_pressure_, 5, 1, "humidity");
        mvwprintw(windows_internal_pressure_, 5, 25, "%2.0f %%", msg_internal_sensor_filter_.humidity);

        wrefresh(windows_internal_pressure_);
    }
}

void WtfNode::update_mission_windows(){
    if(msg_first_received_mission_state_ && msg_first_received_depth_control_set_point) {
        mvwprintw(windows_mission_, 1, 25, "%0.2f", (this->now() - min(time_last_mission_state_, time_last_depth_control_set_point_)).seconds());

//        mvwprintw(windows_mission_, 3, 1, "north");
////        mvwprintw(windows_mission_, 3, 25, "%f", msg_waypoint_.north);
//
//        mvwprintw(windows_mission_, 4, 1, "east");
////        mvwprintw(windows_mission_, 4, 25, "%f", msg_waypoint_.east);

        mvwprintw(windows_mission_, 3, 1, "depth");
        mvwprintw(windows_mission_, 3, 25, "%0.2f", msg_depth_control_set_point_.depth);

        mvwprintw(windows_mission_, 4, 1, "limit_velocity");
        mvwprintw(windows_mission_, 4, 25, "%0.3f", msg_depth_control_set_point_.limit_velocity);

        mvwprintw(windows_mission_, 5, 1, "mission mode");
        mvwprintw(windows_mission_, 5, 25, "%5s", mission_mode_string_[msg_mission_state_.mode].c_str());

        mvwprintw(windows_mission_, 6, 1, "mission state");
        mvwprintw(windows_mission_, 6, 25, "%5s", mission_state_string_[msg_mission_state_.state].c_str());

//        mvwprintw(windows_mission_, 7, 1, "mission status");
//        mvwprintw(windows_mission_, 7, 25, "%5s", get_bool_text(msg_mission_state_..mission_enable).c_str());

//        mvwprintw(windows_mission_, 8, 1, "enable_thrusters");
//        mvwprintw(windows_mission_, 8, 25, "%5s", get_bool_text(msg_waypoint_.enable_thrusters).c_str());

        mvwprintw(windows_mission_, 7, 1, "waypoint_id");
        mvwprintw(windows_mission_, 7, 25, "%4d", msg_mission_state_.waypoint_id);

        mvwprintw(windows_mission_, 10, 1, "waypoint_length");
        mvwprintw(windows_mission_, 10, 25, "%4d", msg_mission_state_.waypoint_length);

        mvwprintw(windows_mission_, 11, 1, "time_to_next_waypoint");
        mvwprintw(windows_mission_, 11, 25, "%6d", static_cast<int>(round(msg_mission_state_.time_to_next_waypoint)));

        wrefresh(windows_mission_);
    }
}

std::string WtfNode::get_bool_text(bool valid){
    return (valid ? "True" : "False");
}

std::string WtfNode::set_color_valid(WINDOW *w, bool valid, const std::string& text){
    if(valid){
        wattron(w, COLOR_PAIR(COLOR_VALID));
        return (text.empty())?"True":text;
    } else {
        wattron(w, COLOR_PAIR(COLOR_NOT_VALID));
        return (text.empty())?"False":text;
    }
}

void WtfNode::update_safety_windows(){
    if(msg_first_received_safety_) {
        mvwprintw(windows_safety_, 1, 25, "%0.2f", (this->now() - time_last_safety_).seconds());

        mvwprintw(windows_safety_, 3, 1, "global_safety_valid");
        mvwprintw(windows_safety_, 3, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.global_safety_valid).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 4, 1, "published_frequency");
        mvwprintw(windows_safety_, 4, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.published_frequency).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 5, 1, "depth_limit");
        mvwprintw(windows_safety_, 5, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.depth_limit).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 6, 1, "batteries_limit");
        mvwprintw(windows_safety_, 6, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.batteries_limit).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 7, 1, "depressurization");
        mvwprintw(windows_safety_, 7, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.depressurization).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 8, 1, "seafloor");
        mvwprintw(windows_safety_, 8, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.seafloor).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 9, 1, "piston");
        mvwprintw(windows_safety_, 9, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.piston).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 10, 1, "zero_depth");
        mvwprintw(windows_safety_, 10, 25, "%5s", set_color_valid(windows_safety_, msg_safety_.zero_depth).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 11, 1, "gnss");
        mvwprintw(windows_safety_, 11, 25, "%5s", set_color_valid(windows_safety_, (msg_gnss_.mode > seabot2_msgs::msg::GpsFix::MODE_NO_FIX)).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 12, 1, "cpu");
        mvwprintw(windows_safety_, 12, 25, "%0.2f", msg_safety_.cpu);

        mvwprintw(windows_safety_, 13, 1, "ram");
        mvwprintw(windows_safety_, 13, 25, "%6d", static_cast<int>(msg_safety_.ram));

        mvwprintw(windows_safety_, 14, 1, "hdd");
        mvwprintw(windows_safety_, 14, 25, "%.1f", msg_safety_.hdd/1024);

        mvwprintw(windows_safety_, 15, 1, "limit_depth");
        mvwprintw(windows_safety_, 15, 25, "%3.2f", msg_safety_.limit_depth);

        wrefresh(windows_safety_);
    }
}

void WtfNode::update_power(){
    if(msg_first_received_power_data_) {
        mvwprintw(windows_power_, 1, 25, "%0.2f", (this->now() - time_last_power_data_).seconds());

        mvwprintw(windows_power_, 3, 1, "battery_volt");
        mvwprintw(windows_power_, 3, 25, "%0.1f", msg_power_data_.battery_volt);

        mvwprintw(windows_power_, 4, 1, "cell_volt[0]");
        mvwprintw(windows_power_, 4, 25, "%0.1f", msg_power_data_.cell_volt[0]);

        mvwprintw(windows_power_, 5, 1, "cell_volt[1]");
        mvwprintw(windows_power_, 5, 25, "%0.1f", msg_power_data_.cell_volt[1]);

        mvwprintw(windows_power_, 6, 1, "esc_current[0]");
        mvwprintw(windows_power_, 6, 25, "%0.2f", msg_power_data_.esc_current[0]);

        mvwprintw(windows_power_, 7, 1, "esc_current[1]");
        mvwprintw(windows_power_, 7, 25, "%0.2f", msg_power_data_.esc_current[1]);

        mvwprintw(windows_power_, 8, 1, "motor_current");
        mvwprintw(windows_power_, 8, 25, "%0.2f", msg_power_data_.motor_current);

        mvwprintw(windows_power_, 9, 1, "power_state");
        mvwprintw(windows_power_, 9, 25, "%s", power_state_string_[msg_power_data_.power_state].c_str());

        wrefresh(windows_power_);
    }
}

void WtfNode::update_depth(){
    if(msg_first_received_depth_data_) {
        mvwprintw(windows_depth_, 1, 25, "%0.2f", (this->now() - time_last_depth_data_).seconds());

        mvwprintw(windows_depth_, 3, 1, "depth");
        mvwprintw(windows_depth_, 3, 25, "%0.3f", msg_depth_data_.depth);

        mvwprintw(windows_depth_, 4, 1, "velocity");
        mvwprintw(windows_depth_, 4, 25, "%0.3f", msg_depth_data_.velocity);

        mvwprintw(windows_depth_, 5, 1, "zero_depth_pressure");
        mvwprintw(windows_depth_, 5, 25, "%0.3f", msg_depth_data_.zero_depth_pressure);

        mvwprintw(windows_depth_, 6, 1, "pressure");
        mvwprintw(windows_depth_, 6, 25, "%0.3f", msg_depth_data_.pressure);

        wrefresh(windows_depth_);
    }
}

void WtfNode::update_piston(){
    if(msg_first_received_piston_data_) {
        mvwprintw(windows_piston_, 1, 25, "%0.2f", (this->now() - time_last_piston_data_).seconds());

        mvwprintw(windows_piston_, 3, 1, "position");
        mvwprintw(windows_piston_, 3, 25, "%7d", msg_piston_data_.position);

        mvwprintw(windows_piston_, 4, 1, "position_set_point");
        mvwprintw(windows_piston_, 4, 25, "%7d", msg_piston_data_.position_set_point);

        mvwprintw(windows_piston_, 5, 1, "switch_top");
        mvwprintw(windows_piston_, 5, 25, "%5s", get_bool_text(msg_piston_data_.switch_top).c_str());

        mvwprintw(windows_piston_, 6, 1, "switch_bottom");
        mvwprintw(windows_piston_, 6, 25, "%5s", get_bool_text(msg_piston_data_.switch_bottom).c_str());

        mvwprintw(windows_piston_, 7, 1, "enable");
        mvwprintw(windows_piston_, 7, 25, "%5s", get_bool_text(msg_piston_data_.enable).c_str());

        mvwprintw(windows_piston_, 8, 1, "motor_sens");
        mvwprintw(windows_piston_, 8, 25, "%5s", get_bool_text(msg_piston_data_.motor_sens).c_str());

        mvwprintw(windows_piston_, 9, 1, "state");
        mvwprintw(windows_piston_, 9, 18, "%21s", piston_state_string_[msg_piston_data_.state].c_str());

        mvwprintw(windows_piston_, 10, 1, "motor_speed_set_point");
        mvwprintw(windows_piston_, 10, 25, "%4hu", msg_piston_data_.motor_speed_set_point);

        mvwprintw(windows_piston_, 11, 1, "motor_speed");
        mvwprintw(windows_piston_, 11, 25, "%4hu", msg_piston_data_.motor_speed);

        mvwprintw(windows_piston_, 12, 1, "battery_voltage");
        mvwprintw(windows_piston_, 12, 25, "%0.2f", msg_piston_data_.battery_voltage);

        mvwprintw(windows_piston_, 13, 1, "motor_current");
        mvwprintw(windows_piston_, 13, 25, "%0.2f", msg_piston_data_.motor_current);

        wrefresh(windows_piston_);
    }
}

void WtfNode::update_depth_control(){
    if(msg_first_received_depth_control_) {
        mvwprintw(windows_depth_control_, 1, 25, "%0.2f", (this->now() - time_last_depth_control_).seconds());

        mvwprintw(windows_depth_control_, 3, 1, "position");
        mvwprintw(windows_depth_control_, 3, 25, "%7d", msg_piston_data_.position);

        mvwprintw(windows_depth_control_, 4, 1, "u");
        mvwprintw(windows_depth_control_, 4, 25, "%f", msg_depth_control_.u);

        mvwprintw(windows_depth_control_, 5, 1, "y");
        mvwprintw(windows_depth_control_, 5, 25, "%f", msg_depth_control_.y);

        mvwprintw(windows_depth_control_, 6, 1, "dy");
        mvwprintw(windows_depth_control_, 6, 25, "%f", msg_depth_control_.dy);

        mvwprintw(windows_depth_control_, 7, 1, "piston_set_point");
        mvwprintw(windows_depth_control_, 7, 25, "%7.2f", msg_depth_control_.piston_set_point);

        mvwprintw(windows_depth_control_, 8, 1, "mode");
        mvwprintw(windows_depth_control_, 8, 21, "%12s", depth_control_string_[msg_depth_control_.mode].c_str());

        wrefresh(windows_depth_control_);
    }
}

void WtfNode::update_robot(){

    mvwprintw(windows_robot_, 1, 20, "%f", this->now().seconds());

    auto t = std::time(nullptr);
    auto tm = *std::gmtime(&t); // localtime
    stringstream ss;
    ss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    mvwprintw(windows_robot_, 1, 40, "%s", ss.str().c_str());
    mvwprintw(windows_robot_, 2, 2, "%s", hostname_.c_str());

    wrefresh(windows_robot_);
}

void WtfNode::update_gnss(){
    if(msg_first_received_gnss_) {
        mvwprintw(windows_gnss_, 1, 25, "%0.2f", (this->now() - time_last_gnss_).seconds());

        mvwprintw(windows_gnss_, 3, 1, "mode");
        mvwprintw(windows_gnss_, 3, 25, "%8s", gpsd_mode_string_[msg_gnss_.mode].c_str());
        wattron(windows_gnss_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_gnss_, 4, 1, "latitude");
        mvwprintw(windows_gnss_, 4, 25, "%f", msg_gnss_.latitude);

        mvwprintw(windows_gnss_, 5, 1, "longitude");
        mvwprintw(windows_gnss_, 5, 25, "%f", msg_gnss_.longitude);

        mvwprintw(windows_gnss_, 6, 1, "time (GNSS)");
        const time_t t = round(msg_gnss_.time);
        auto tm = *std::gmtime(&t); // localtime
        stringstream ss;
        ss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        mvwprintw(windows_gnss_, 6, 20, "%17s", ss.str().c_str());

        wrefresh(windows_gnss_);
    }
}

void WtfNode::update_sensors(){
    if(msg_first_received_profile_){
        mvwprintw(windows_gnss_, 1, 25, "%0.2f", (this->now() - time_last_profile_).seconds());

        mvwprintw(windows_sensors_, 3, 1, "ping distance");
        mvwprintw(windows_sensors_, 3, 25, "%3.1f", msg_profile_.distance/1e3);
        msg_profile_.distance = std::numeric_limits<uint32_t>::quiet_NaN();

        mvwprintw(windows_sensors_, 4, 1, "ping confidence");
        mvwprintw(windows_sensors_, 4, 25, "%3d %%", msg_profile_.confidence);
        msg_profile_.confidence = std::numeric_limits<uint16_t>::quiet_NaN();
    }

    if(msg_first_received_density_){
        mvwprintw(windows_sensors_, 5, 1, "density");
        mvwprintw(windows_sensors_, 5, 25, "%4.1f", msg_density_.density);
        msg_density_.density = std::numeric_limits<float>::quiet_NaN();;
    }

    if(msg_first_received_temperature_sensor_data_){
        mvwprintw(windows_sensors_, 6, 1, "temperature");
        mvwprintw(windows_sensors_, 6, 25, "%2.2f", msg_temperature_sensor_data_.temperature);
        msg_temperature_sensor_data_.temperature = std::numeric_limits<float>::quiet_NaN();
    }

    wrefresh(windows_sensors_);
}

void WtfNode::update_audio() const {
    if(msg_first_received_audio_dspic_) {
        mvwprintw(windows_audio_, 1, 25, "%0.2f", (this->now() - time_last_audio_dspic_).seconds());

        mvwprintw(windows_audio_, 3, 1, "time dspic");
        mvwprintw(windows_audio_, 3, 25, "%u", msg_audio_dspic_.posix_time);

        mvwprintw(windows_audio_, 4, 1, "emission number");
        mvwprintw(windows_audio_, 4, 25, "%u", msg_audio_dspic_.signal_number);
    }
    wrefresh(windows_audio_);
}

void WtfNode::update_log() const {
    size_t i = 0;
    for(auto &msg:msg_queue_log_){
        string msg_string = msg.name + ": " + msg.msg;
        const size_t space_numbers = max(0, (windows_max_x_-2) - static_cast<int>(msg_string.length()));
        const string spaces(space_numbers, ' ');
        msg_string += spaces;
        mvwprintw(windows_log_, 2+i, 1, "%s", msg_string.c_str());
        i++;
    }
    wrefresh(windows_log_);
}

void WtfNode::timer_callback() {
    update_log();
    update_safety_windows();
    update_mission_windows();
    update_internal_pressure_windows();
    update_power();
    update_depth();
    update_piston();
    update_robot();
    update_depth_control();
    update_gnss();
    update_sensors();
    update_audio();
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WtfNode>());
    rclcpp::shutdown();
    return 0;
}