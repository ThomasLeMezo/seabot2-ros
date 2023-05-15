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

    /// height, width, point_x, point_y
    windows_robot_              = subwin(stdscr, 5, 122, 0, 0);
    windows_safety_             = subwin(stdscr, 15, 38, 5, 0);
    windows_internal_pressure_  = subwin(stdscr, 7, 38, 20, 0);
    windows_power_              = subwin(stdscr, 13, 38, 27, 0);
    windows_depth_control_      = subwin(stdscr, 11, 38, 40, 0);

    windows_mission_            = subwin(stdscr, 15, 44, 5, 39);
    windows_depth_              = subwin(stdscr, 8, 44, 20, 39);
    windows_piston_             = subwin(stdscr, 15, 44, 28, 39);
    windows_gnss_               = subwin(stdscr, 8, 44, 43, 39);

    windows_sensors_               = subwin(stdscr, 15, 38, 5, 84);

    box(windows_robot_, ACS_VLINE, ACS_HLINE);
    box(windows_safety_, ACS_VLINE, ACS_HLINE);
    box(windows_internal_pressure_, ACS_VLINE, ACS_HLINE);
    box(windows_power_, ACS_VLINE, ACS_HLINE);
    box(windows_depth_control_, ACS_VLINE, ACS_HLINE);
    box(windows_depth_, ACS_VLINE, ACS_HLINE);
    box(windows_piston_, ACS_VLINE, ACS_HLINE);
    box(windows_mission_, ACS_VLINE, ACS_HLINE);
    box(windows_gnss_, ACS_VLINE, ACS_HLINE);
    box(windows_sensors_, ACS_VLINE, ACS_HLINE);

    mvwprintw(windows_robot_, 1, 1, "SEABOT");
    mvwprintw(windows_safety_, 1, 1, "SAFETY");
    mvwprintw(windows_internal_pressure_, 1, 1, "INTERNAL PRESSURE");
    mvwprintw(windows_power_, 1, 1, "POWER");
    mvwprintw(windows_depth_control_, 1, 1, "DEPTH CONTROL");
    mvwprintw(windows_depth_, 1, 1, "DEPTH");
    mvwprintw(windows_piston_, 1, 1, "PISTON");
    mvwprintw(windows_mission_, 1, 1, "MISSION");
    mvwprintw(windows_gnss_, 1, 1, "GNSS");
    mvwprintw(windows_sensors_, 1, 1, "SENSORS");

    refresh();
}

WtfNode::~WtfNode(){
    endwin();
}

void WtfNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", static_cast<int>(loop_dt_.count()));
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

void WtfNode::gnss_callback(const gpsd_client::msg::GpsFix &msg){
    msg_gnss_ = msg;
    time_last_gnss_ = this->now();
    msg_first_received_gnss_ = true;
}

void WtfNode::profile_callback(const bluerobotics_ping_driver::msg::Profile &msg) {
    msg_profile_ = msg;
    time_last_profile_ = this->now();
    msg_first_received_profile_ = true;
}

void WtfNode::density_callback(const seabot2_density::msg::Density &msg) {
    msg_density_ = msg;
    time_last_density_ = this->now();
    msg_first_received_density_ = true;
}

void WtfNode::temperature_sensor_data_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg) {
    msg_temperature_sensor_data_ = msg;
    time_last_temperature_sensor_data_ = this->now();
    msg_first_received_temperature_sensor_data_ = true;
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

    subscriber_gnss_ = this->create_subscription<gpsd_client::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&WtfNode::gnss_callback, this, _1));

    subscriber_profile_ = this->create_subscription<bluerobotics_ping_driver::msg::Profile>(
            "/driver/profile", 10, std::bind(&WtfNode::profile_callback, this, _1));

    subscriber_density_ = this->create_subscription<seabot2_density::msg::Density>(
            "/observer/density", 10, std::bind(&WtfNode::density_callback, this, _1));

    subscriber_temperature_sensor_data_ = this->create_subscription<temperature_tsys01_driver::msg::TemperatureSensorData>(
            "/observer/temperature", 10, std::bind(&WtfNode::temperature_sensor_data_callback, this, _1));
}

void WtfNode::update_internal_pressure_windows(){
    if(msg_first_received_internal_sensor_filter_) {
        mvwprintw(windows_internal_pressure_, 1, 25, "%0.2f", (this->now() - time_last_internal_sensor_filter_).seconds());

        mvwprintw(windows_internal_pressure_, 3, 1, "pressure");
        mvwprintw(windows_internal_pressure_, 3, 25, "%0.0f", msg_internal_sensor_filter_.pressure);

        mvwprintw(windows_internal_pressure_, 4, 1, "temperature");
        mvwprintw(windows_internal_pressure_, 4, 25, "%0.2f", msg_internal_sensor_filter_.temperature);

        mvwprintw(windows_internal_pressure_, 5, 1, "humidity");
        mvwprintw(windows_internal_pressure_, 5, 25, "%0.0f", msg_internal_sensor_filter_.humidity);

        wrefresh(windows_internal_pressure_);
    }
}

void WtfNode::update_mission_windows(){
    if(msg_first_received_waypoint_) {
        mvwprintw(windows_mission_, 1, 25, "%0.2f", (this->now() - time_last_waypoint_).seconds());

        mvwprintw(windows_mission_, 3, 1, "north");
        mvwprintw(windows_mission_, 3, 25, "%f", msg_waypoint_.north);

        mvwprintw(windows_mission_, 4, 1, "east");
        mvwprintw(windows_mission_, 4, 25, "%f", msg_waypoint_.east);

        mvwprintw(windows_mission_, 5, 1, "depth");
        mvwprintw(windows_mission_, 5, 25, "%0.2f", msg_waypoint_.depth);

        mvwprintw(windows_mission_, 6, 1, "limit_velocity");
        mvwprintw(windows_mission_, 6, 25, "%0.3f", msg_waypoint_.limit_velocity);

        mvwprintw(windows_mission_, 7, 1, "mission_enable");
        mvwprintw(windows_mission_, 7, 25, "%s", get_bool_text(msg_waypoint_.mission_enable).c_str());

        mvwprintw(windows_mission_, 8, 1, "enable_thrusters");
        mvwprintw(windows_mission_, 8, 25, "%s", get_bool_text(msg_waypoint_.enable_thrusters).c_str());

        mvwprintw(windows_mission_, 9, 1, "waypoint_id");
        mvwprintw(windows_mission_, 9, 25, "%d", msg_waypoint_.waypoint_id);

        mvwprintw(windows_mission_, 10, 1, "waypoint_length");
        mvwprintw(windows_mission_, 10, 25, "%d", msg_waypoint_.waypoint_length);

        mvwprintw(windows_mission_, 11, 1, "time_to_next_waypoint");
        mvwprintw(windows_mission_, 11, 25, "%6d", static_cast<int>(round(msg_waypoint_.time_to_next_waypoint)));

        mvwprintw(windows_mission_, 12, 1, "seafloor_landing");
        mvwprintw(windows_mission_, 12, 25, "%s", get_bool_text(msg_waypoint_.seafloor_landing).c_str());

        wrefresh(windows_mission_);
    }
}

std::string WtfNode::get_bool_text(bool valid){
    return (valid ? "True " : "False");
}

std::string WtfNode::set_color_valid(WINDOW *w, bool valid, const std::string& text){
    if(valid){
        wattron(w, COLOR_PAIR(COLOR_VALID));
        return (text=="")?"True ":text;
    } else {
        wattron(w, COLOR_PAIR(COLOR_NOT_VALID));
        return (text=="")?"False":text;
    }
}

void WtfNode::update_safety_windows(){
    if(msg_first_received_safety_) {
        mvwprintw(windows_safety_, 1, 25, "%0.2f", (this->now() - time_last_safety_).seconds());

        mvwprintw(windows_safety_, 3, 1, "global_safety_valid");
        mvwprintw(windows_safety_, 3, 25, "%s", set_color_valid(windows_safety_, msg_safety_.global_safety_valid).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 4, 1, "published_frequency");
        mvwprintw(windows_safety_, 4, 25, "%s", set_color_valid(windows_safety_, msg_safety_.published_frequency).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 5, 1, "depth_limit");
        mvwprintw(windows_safety_, 5, 25, "%s", set_color_valid(windows_safety_, msg_safety_.depth_limit).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 6, 1, "batteries_limit");
        mvwprintw(windows_safety_, 6, 25, "%s", set_color_valid(windows_safety_, msg_safety_.batteries_limit).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 7, 1, "depressurization");
        mvwprintw(windows_safety_, 7, 25, "%s", set_color_valid(windows_safety_, msg_safety_.depressurization).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 8, 1, "seafloor");
        mvwprintw(windows_safety_, 8, 25, "%s", set_color_valid(windows_safety_, msg_safety_.seafloor).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 9, 1, "piston");
        mvwprintw(windows_safety_, 9, 25, "%s", set_color_valid(windows_safety_, msg_safety_.piston).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 10, 1, "zero_depth");
        mvwprintw(windows_safety_, 10, 25, "%s", set_color_valid(windows_safety_, msg_safety_.zero_depth).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 11, 1, "gnss");
        mvwprintw(windows_safety_, 11, 25, "%s", set_color_valid(windows_safety_, (msg_gnss_.mode > gpsd_client::msg::GpsFix::MODE_NO_FIX)).c_str());
        wattron(windows_safety_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_safety_, 12, 1, "cpu");
        mvwprintw(windows_safety_, 12, 25, "%0.2f", msg_safety_.cpu);

        mvwprintw(windows_safety_, 13, 1, "ram");
        mvwprintw(windows_safety_, 13, 25, "%d", static_cast<int>(msg_safety_.ram));

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
        mvwprintw(windows_piston_, 3, 25, "%*d", 7, msg_piston_data_.position);

        mvwprintw(windows_piston_, 4, 1, "position_set_point");
        mvwprintw(windows_piston_, 4, 25, "%*d", 7, msg_piston_data_.position_set_point);

        mvwprintw(windows_piston_, 5, 1, "switch_top");
        mvwprintw(windows_piston_, 5, 25, msg_piston_data_.switch_top? "True " : "False");

        mvwprintw(windows_piston_, 6, 1, "switch_bottom");
        mvwprintw(windows_piston_, 6, 25, "%s", msg_piston_data_.switch_bottom? "True " : "False");

        mvwprintw(windows_piston_, 7, 1, "enable");
        mvwprintw(windows_piston_, 7, 25, "%s", msg_piston_data_.enable? "True " : "False");

        mvwprintw(windows_piston_, 8, 1, "motor_sens");
        mvwprintw(windows_piston_, 8, 25, "%s", msg_piston_data_.motor_sens? "True " : "False");

        mvwprintw(windows_piston_, 9, 1, "state");
        mvwprintw(windows_piston_, 9, 20, "%s", piston_state_string_[msg_piston_data_.state].c_str());

        mvwprintw(windows_piston_, 10, 1, "motor_speed_set_point");
        mvwprintw(windows_piston_, 10, 25, "%hu", msg_piston_data_.motor_speed_set_point);

        mvwprintw(windows_piston_, 11, 1, "motor_speed");
        mvwprintw(windows_piston_, 11, 25, "%hu", msg_piston_data_.motor_speed);

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
        mvwprintw(windows_depth_control_, 3, 25, "%*d", 7, msg_piston_data_.position);

        mvwprintw(windows_depth_control_, 4, 1, "u");
        mvwprintw(windows_depth_control_, 4, 25, "%f", msg_depth_control_.u);

        mvwprintw(windows_depth_control_, 5, 1, "y");
        mvwprintw(windows_depth_control_, 5, 25, "%f", msg_depth_control_.y);

        mvwprintw(windows_depth_control_, 6, 1, "dy");
        mvwprintw(windows_depth_control_, 6, 25, "%f", msg_depth_control_.dy);

        mvwprintw(windows_depth_control_, 7, 1, "piston_set_point");
        mvwprintw(windows_depth_control_, 7, 25, "%*.2f", 7, msg_depth_control_.piston_set_point);

        mvwprintw(windows_depth_control_, 8, 1, "mode");
        mvwprintw(windows_depth_control_, 8, 25, "%s", depth_control_string_[msg_depth_control_.mode].c_str());

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
        mvwprintw(windows_gnss_, 3, 25, "%s", gpsd_mode_string_[msg_gnss_.mode].c_str());
        wattron(windows_gnss_, COLOR_PAIR(COLOR_DEFAULT));

        mvwprintw(windows_gnss_, 4, 1, "latitude");
        mvwprintw(windows_gnss_, 4, 25, "%f", msg_gnss_.latitude);

        mvwprintw(windows_gnss_, 5, 1, "longitude");
        mvwprintw(windows_gnss_, 5, 25, "%f", msg_gnss_.longitude);

        mvwprintw(windows_gnss_, 6, 1, "time (GNSS)");
        time_t t = round(msg_gnss_.time);
        auto tm = *std::gmtime(&t); // localtime
        stringstream ss;
        ss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        mvwprintw(windows_gnss_, 6, 23, "%s", ss.str().c_str());

        wrefresh(windows_gnss_);
    }
}

void WtfNode::update_sensors(){
    if(msg_first_received_profile_){
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

void WtfNode::timer_callback() {
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
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WtfNode>());
    rclcpp::shutdown();
    return 0;
}