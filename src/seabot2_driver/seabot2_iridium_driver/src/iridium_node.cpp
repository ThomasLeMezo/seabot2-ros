//
// Created by lemezoth on 06/06/23.
//

#include "seabot2_iridium_driver/iridium_node.h"

using namespace placeholders;

IridiumNode::IridiumNode()
        : Node("iridium_node"), sbd_(), log_state_(){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();
    init_interfaces();

    int success = sbd_.init(serial_port_name_, serial_baud_rate_);
    if(success==EXIT_FAILURE) {
        RCLCPP_WARN(this->get_logger(), "[Irdium_node] Init communication with SBD failed");
        exit(EXIT_FAILURE);
    }
    sbd_.set_debug(sbd_debug_);
    sbd_.cmd_enable_indicator_reporting(true);
    sbd_.cmd_enable_alert(true);

    timer_read_ = this->create_wall_timer(
            loop_serial_read_, std::bind(&IridiumNode::timer_read_callback, this));
    timer_write_ = this->create_wall_timer(
            loop_serial_write_, std::bind(&IridiumNode::timer_write_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Irdium_node] Start Ok");
}

void IridiumNode::timer_read_callback() {
    timer_read_->cancel();
    sbd_.read();
    timer_read_->reset();
}

void IridiumNode::timer_write_callback() {
    timer_write_->cancel();
    process();
    timer_write_->reset();
}

void IridiumNode::init_parameters() {
    this->declare_parameter<string>("mission_file_name", mission_file_name_);
    this->declare_parameter<string>("mission_path", mission_path_);

    this->declare_parameter<int>("time_between_communication", time_between_communication_.count());
    this->declare_parameter<int>(" surface_wait_time",  surface_wait_time_.count());
    this->declare_parameter<double>("surface_depth_limit", surface_depth_limit_);
    this->declare_parameter<int>("delay_last_log_version", delay_last_log_version_.count());
    this->declare_parameter<int>("delay_last_gnss", delay_last_gnss_.count());
    this->declare_parameter<bool>("enable_iridium_gnss", enable_iridium_gnss_);
    this->declare_parameter<bool>("sbd_debug", sbd_debug_);
    this->declare_parameter<bool>("enable_iridium_sending", enable_iridium_sending_);

    mission_file_name_ = this->get_parameter_or("mission_file_name", mission_file_name_);
    mission_path_ = this->get_parameter_or("mission_path", mission_path_);

    time_between_communication_ = std::chrono::seconds(this->get_parameter_or("time_between_communication", time_between_communication_.count()));
    surface_wait_time_ = std::chrono::milliseconds(this->get_parameter_or("surface_wait_time", surface_wait_time_.count()));
    surface_depth_limit_ = this->get_parameter_or("surface_depth_limit", surface_depth_limit_);
    delay_last_log_version_ = std::chrono::seconds(this->get_parameter_or("delay_last_log_version", delay_last_log_version_.count()));
    delay_last_gnss_ = std::chrono::seconds(this->get_parameter_or("delay_last_gnss", delay_last_gnss_.count()));
    enable_iridium_gnss_ = this->get_parameter_or("enable_iridium_gnss", enable_iridium_gnss_);
    sbd_debug_ = this->get_parameter_or("sbd_debug", sbd_debug_);
    enable_iridium_sending_ = this->get_parameter_or("enable_iridium_sending", enable_iridium_sending_);
}

void IridiumNode::init_interfaces() {
    subscriber_safety_data_ =  this->create_subscription<seabot2_safety::msg::SafetyStatus>(
            "/safety/safety", 10, std::bind(&IridiumNode::safety_callback, this, _1));

    subscriber_internal_sensor_filter_ = this->create_subscription<pressure_bme280_driver::msg::Bme280Data>(
            "/observer/pressure_internal", 10, std::bind(&IridiumNode::internal_sensor_callback, this, _1));

    subscriber_power_data_ = this->create_subscription<seabot2_power_driver::msg::PowerState>(
            "/observer/power", 10, std::bind(&IridiumNode::power_callback, this, _1));

    subscriber_gnss_data_ = this->create_subscription<gpsd_client::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&IridiumNode::gpsd_callback, this, _1));

    subscriber_gnss_pose_ = this->create_subscription<seabot2_lambert::msg::GnssPose>(
            "/observer/pose_mean", 10, std::bind(&IridiumNode::gnss_pose_callback, this, _1));

    subscriber_depth_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&IridiumNode::depth_callback, this, _1));

    subscriber_mission = this->create_subscription<seabot2_mission::msg::MissionState >(
            "/mission/mission_state", 10, std::bind(&IridiumNode::mission_callback, this, _1));

    publisher_iridium_session_ = this->create_publisher<seabot2_iridium_driver::msg::IridiumSession>(
            "/iridium/iridium_session", 10);

    publisher_iridium_status_ = this->create_publisher<seabot2_iridium_driver::msg::IridiumStatus>(
            "/iridium/iridium_status", 10);

    publisher_iridium_log_ = this->create_publisher<std_msgs::msg::String>(
            "/iridium/iridium_log", 10);
}

void IridiumNode::internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg) {
    log_state_.internal_pressure_ = msg.pressure;
    log_state_.internal_temperature_ = msg.temperature;
    log_state_.internal_humidity_ = msg.humidity;
}

void IridiumNode::power_callback(const seabot2_power_driver::msg::PowerState &msg) {
    log_state_.battery_ = msg.battery_volt;
}

void IridiumNode::safety_callback(const seabot2_safety::msg::SafetyStatus &msg) {
    log_state_.safety_global_safety_valid_ = msg.global_safety_valid;
    log_state_.safety_published_frequency_ = msg.published_frequency;
    log_state_.safety_depth_limit_ = msg.depth_limit;
    log_state_.safety_batteries_limit_ = msg.batteries_limit;
    log_state_.safety_depressurization_ = msg.depressurization;
    log_state_.safety_seafloor_ = msg.seafloor;
    log_state_.safety_piston_ = msg.piston;
    log_state_.safety_zero_depth_ = msg.zero_depth;
}

void IridiumNode::gpsd_callback(const gpsd_client::msg::GpsFix &msg) {
    valid_fix_ = msg.mode > gpsd_client::msg::GpsFix::MODE_NO_FIX;
    fix_latitude_ = msg.latitude;
    fix_longitude_ = msg.longitude;
    time_last_gnss_ = msg.header.stamp;
}

void IridiumNode::gnss_pose_callback(const seabot2_lambert::msg::GnssPose &msg){
    log_state_.gnss_heading_ = msg.heading;
    log_state_.gnss_speed_ = msg.velocity;
    log_state_.gnss_mean_east_ = msg.east;
    log_state_.gnss_mean_north_ = msg.north;
}

void IridiumNode::mission_callback(const seabot2_mission::msg::MissionState &msg) {
    log_state_.current_waypoint_ = msg.waypoint_id;
}

void IridiumNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg) {
    if(msg.depth < surface_depth_limit_){
        if(!surface_is_valid_){
            surface_time_detected_ = this->now();
            surface_is_valid_ = true;
        }
        else{
            if((this->now()-surface_time_detected_)>surface_wait_time_)
                is_surface_ = true;
        }
    }
    else{
        surface_is_valid_ = false;
        is_surface_ = false;
    }
}

void IridiumNode::process(){

    bool send_data_required = false;
    rclcpp::Time t = this->now();

    if(is_surface_
       && ((t-last_time_communication_)>time_between_communication_)
       && sbd_.get_indicator_service() == 1
       && enable_iridium_sending_){
        send_data_required = true;

        /// Update LogData if log is too old (to be faster in case of poor connection)
        if((t-time_last_log_version_)>delay_last_log_version_){

            /// Update the data
            string log_sentence = log_state_.serialize_log_state(static_cast<long long>(round(this->now().seconds())));
            int valid = sbd_.cmd_write_message(log_sentence);

            /// Update the gnss position
            if(valid == EXIT_SUCCESS){
                if(enable_iridium_gnss_ && valid_fix_ && (t - time_last_gnss_) < delay_last_gnss_)
                    sbd_.set_gnss(fix_latitude_, fix_longitude_);
                time_last_log_version_ = t;
            }
            else{
                RCLCPP_INFO(this->get_logger(), "[Iridium] Message writing failed");
            }
        }
    }

    // Request a session (send/receive) if a ring alert is received or if there is waiting data
    if(sbd_.get_indicator_service()==1 && (send_data_required || sbd_.get_ring_alert() || sbd_.get_waiting()>0)){
        int result = sbd_.cmd_session(); /// Process a send/receive session

        if(result==1)
            RCLCPP_INFO(this->get_logger(), "[Iridium] Session not finished yet");

        seabot2_iridium_driver::msg::IridiumSession session_msg;
        session_msg.mo = sbd_.get_session_mo();
        session_msg.momsn = sbd_.get_session_momsn();
        session_msg.mt = sbd_.get_session_mt();
        session_msg.mtmsn = sbd_.get_session_mtmsn();
        session_msg.waiting = sbd_.get_waiting();
        publisher_iridium_session_->publish(session_msg);

        // Analyse success (sent message)
        bool flush_mo = false;
        int session_mo_result = sbd_.get_session_mo();
        if(session_mo_result>=0 && session_mo_result <= 4){
            last_time_communication_ = this->now();
            flush_mo = true;
            send_data_required = false;
            log_state_.last_cmd_received_ = LogData::LOG_STATE;
        }

        // Look at received message
        bool flush_mt = false;
        if(sbd_.get_session_mt()==1){
            std::string message_data = sbd_.cmd_read_message();

            // LogData Raw
            std_msgs::msg::String msg_raw;
            msg_raw.data = message_data;
            publisher_iridium_log_->publish(msg_raw);

            // Decode
            call_decode(message_data);
            flush_mt = true;
        }

        sbd_.cmd_flush_message(flush_mo, flush_mt); // Flush data to avoid re-sending it
    }

    // LogData Data
    if(is_surface_){
        seabot2_iridium_driver::msg::IridiumStatus status_msg;
        status_msg.service = sbd_.get_indicator_service();
        status_msg.signal_strength = sbd_.get_indicator_signal();
        status_msg.antenna = sbd_.get_indicator_antenna();
        publisher_iridium_status_->publish(status_msg);
    }
}

void IridiumNode::call_decode(const string &data_raw){
    // Test if is at surface for sufficient period of time
    LogData log_cmd;
    log_cmd.deserialize_log_CMD(data_raw);

    switch(log_cmd.msg_type_){
        case LogData::CMD_SLEEP:
        {
            int sleep_time = log_cmd.sleep_time_;

            /// ToDo
//            int hours = floor(sleep_time/60.);
//            int min = sleep_time-hours*60;
//            call_sleep_param(hours, min, 0, 200);
//            call_sleep();
            break;
        }
        case LogData::CMD_MISSION_NEW:
        case LogData::CMD_MISSION_KEEP:
        {
            // ToDO : write new mission file
            MissionXML mission(log_cmd);
            mission.write(mission_path_+mission_file_name_);
            break;
        }
        case LogData::CMD_PARAMETERS:
        {
            // Enable/Diseable
            // safety, flash, mission, sink etc.
//            duration_between_msg = (log_cmd.m_period_message/10.)*60.;
//            call_enable_mission(log_cmd.m_enable_mission);
            break;
        }
        default:
            break;
    }
    log_state_.last_cmd_received_ = log_cmd.msg_type_;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IridiumNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
