//
// Created by lemezoth on 06/06/23.
//

#include "seabot2_audio_recorder/audio_recorder_node.h"
#include <iostream>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <future>

using namespace std;
using namespace std::placeholders;

AudioRecorderNode::AudioRecorderNode()
        : Node("audio_recorder_node"), tlv_(this), dspic_(this){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&AudioRecorderNode::timer_callback, this));

    tlv_.i2c_open();
    tlv_.set_adc_gain(gain_ch1_, gain_ch2_);

    dspic_.i2c_open();
    dspic_.set_pps_sync_chirp_id(chirp_id_);
    dspic_.enable_chirp(enable_chirp_);

    // Find home directory and append log folder
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    workingDirectory_.append(homedir);
    workingDirectory_.append("/audio/");

    // Create log folder if it does not exist
    if (!filesystem::exists(workingDirectory_)) {
        filesystem::create_directory(workingDirectory_);
    }

    // Change working directory to log folder
    if (chdir(workingDirectory_.c_str()) != 0) {
        std::cerr << "Error changing working directory to " << workingDirectory_ << std::endl;
    }

    // Start recording
    manage_subprocess(true);




    RCLCPP_INFO(this->get_logger(), "[audio_recorder_node] Start Ok");
}

AudioRecorderNode::~AudioRecorderNode() {
    wait_kill();
    dspic_.enable_chirp(false);
}

void AudioRecorderNode::manage_subprocess(bool start_new_bag) {
    std_msgs::msg::Bool msg;

    // Check if the subprocess is still running
    if (thread_currently_running_) {
        wait_kill();
        msg.data = false;
        publisher_record_->publish(msg);
    }
    usleep(1000000);

    if(start_new_bag) {
        string command_launch = command_;
        // Create a thread for the subprocess
        subprocessFuture_ = std::async(std::launch::async, [command_launch] {
            // Call the subprocess using std::system
            return std::system(command_launch.c_str());
        });
        thread_currently_running_ = true;
        RCLCPP_INFO(this->get_logger(), "[recorder_node] Start audio recording");
    }

    msg.data = start_new_bag;
    publisher_record_->publish(msg);
}

void AudioRecorderNode::timer_callback(){
    if(gnss_fix_once_){
        dspic_.sync_pps();
        gnss_fix_once_ = true;
    }

    // Read pps_sync value and publish
    uint8_t pps = dspic_.get_pps_value();
    std_msgs::msg::Byte msg_pps;
    msg_pps.data = pps;
    publisher_pps_->publish(msg_pps);
}

void AudioRecorderNode::wait_kill() {
    // Terminate the subprocess
    string command = "pkill -SIGTERM -f '"+ command_ +"'";
    std::system(command.c_str());
    // Wait for the subprocess thread to finish
    subprocessFuture_.wait();
    thread_currently_running_ = false;
}

void AudioRecorderNode::init_parameters() {
    this->declare_parameter<int>("gain_ch1", gain_ch1_);
    this->declare_parameter<int>("gain_ch2", gain_ch2_);
    this->declare_parameter<int>("chirp_id", chirp_id_);
    this->declare_parameter<bool>("enable_chirp", enable_chirp_);

    gain_ch1_ = this->get_parameter_or("gain_ch1", gain_ch1_);
    gain_ch2_ = this->get_parameter_or("gain_ch2", gain_ch2_);
    chirp_id_ = this->get_parameter_or("chirp_id", chirp_id_);

    enable_chirp_ = this->get_parameter_or("enable_chirp", enable_chirp_);

    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void AudioRecorderNode::init_interfaces() {
    service_rosbag_ = this->create_service<std_srvs::srv::SetBool>(
            "restart_audio_record",
            std::bind(&AudioRecorderNode::callback_trigger, this, _1, _2, _3));

    publisher_record_ = this->create_publisher<std_msgs::msg::Bool>("audio_record_sync", 10);

    subscriber_gnss_data_ = this->create_subscription<gpsd_client::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&AudioRecorderNode::gpsd_callback, this, _1));

    publisher_pps_ = this->create_publisher<std_msgs::msg::Byte>("audio_pps", 10);
}

void AudioRecorderNode::gpsd_callback(const gpsd_client::msg::GpsFix &msg){
    if(msg.mode>=gpsd_client::msg::GpsFix::MODE_2D){
        gnss_fix_once_ = true;
    }
}

void AudioRecorderNode::callback_trigger(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    (void)request_header;
    (void)request;

    manage_subprocess(request->data);

    // Set the response
    response->success = true;
    response->message = "Process audio record request";
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioRecorderNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
