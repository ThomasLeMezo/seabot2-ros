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
            loop_safety_dt_, std::bind(&AudioRecorderNode::timer_callback, this));

    tlv_.i2c_open();
    tlv_.set_adc_gain(gain_ch1_, gain_ch2_);

    dspic_.i2c_open();
    dspic_.wait_recompute_signal();
    if (dspic_.set_robot_code(robot_code_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error setting robot code");
        exit(EXIT_FAILURE);
    }
    dspic_.sync_pps();
    if (dspic_.set_duration_between_shoot(duration_between_shoot_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error setting duration between shoot");
        exit(EXIT_FAILURE);
    }
    if (dspic_.set_shoot_offset_from_posix_zero(robot_code_ * time_slot_duration_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error setting shoot offset from posix zero");
        exit(EXIT_FAILURE);
    }
    if (dspic_.enable_chirp(enable_chirp_) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error setting enable chirp");
        exit(EXIT_FAILURE);
    }

    const uint16_t frequency_middle_current = dspic_.get_frequency_middle();
    const uint16_t frequency_range_current = dspic_.get_frequency_range();
    const uint8_t signal_function_current = dspic_.get_signal_function();
    if(frequency_middle_current != frequency_middle_
        || frequency_range_current != frequency_range_
        || signal_function_current != signal_function_) {
        if (dspic_.recompute_chirp(frequency_middle_, frequency_range_, signal_function_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error recompute chirp");
            exit(EXIT_FAILURE);
        }
    }

    // Find home directory and append log folder
    const struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    workingDirectory_.append(homedir);
    workingDirectory_.append(audio_save_directory_);

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
    if (dspic_.enable_chirp(false) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[audio_recorder_node] Error disabling chirp");
    }
}

std::string AudioRecorderNode::get_arecord_command() const {
    std::string command = "arecord";
    command += " -D " + audio_device_;
    command += " -f S" + to_string(audio_nb_bits_) + "_LE";
    command += " -c" + to_string(audio_nb_channels_);
    command += " -r " + to_string(audio_frequency_);
    command += " -t wav -v --use-strftime %Y/%m/%d/listen_%Y-%m-%d_%H-%M-%S_%v.wav";
    command += " --max-file-time " + to_string(audio_max_file_time_);

    return command;
}

void AudioRecorderNode::manage_subprocess(bool start_new_audio) {
    std_msgs::msg::Bool msg;

    // Check if the subprocess is still running
    if (thread_currently_running_) {
        wait_kill();
        msg.data = false;
        publisher_record_->publish(msg);
    }
    usleep(1000000); // 1s

    if(start_new_audio) {
        audio_command_last_ = get_arecord_command();
        string command_tmp = audio_command_last_;
        RCLCPP_INFO(this->get_logger(), "[audio_recorder_node] Command: %s", command_tmp.c_str());
        // Create a thread for the subprocess
        subprocessFuture_ = std::async(std::launch::async, [command_tmp] {
            // Call the subprocess using std::system
            return std::system(command_tmp.c_str());
        });
        thread_currently_running_ = true;
        RCLCPP_INFO(this->get_logger(), "[recorder_node] Start audio recording");
    }

    msg.data = start_new_audio;
    publisher_record_->publish(msg);
}

void AudioRecorderNode::timer_callback(){
    if(!dspic_posix_fix_ && gnss_fix_once_){
        dspic_.sync_pps();
        dspic_posix_fix_ = true;
    }

    // Read pps_sync value and publish
    seabot2_msgs::msg::SyncDspic msg;
    msg.posix_time = dspic_.get_posix_time();
    msg.signal_number = dspic_.get_signal_number();
    publisher_dspic_debug_->publish(msg);

    // Check amount of free space on the hard drive
    if(filesystem::space(workingDirectory_).available < audio_hdd_space_limit_stop_ * 1024 * 1024){
        if(thread_currently_running_) {
            RCLCPP_WARN(this->get_logger(), "[audio_recorder_node] Low disk space, stopping recording");
            manage_subprocess(false);
        }
    }
}

void AudioRecorderNode::wait_kill() {
    // Terminate the subprocess
    const string command = "pkill -SIGTERM -f '"+ audio_command_last_ +"'";
    std::system(command.c_str());
    // Wait for the subprocess thread to finish
    subprocessFuture_.wait();
    thread_currently_running_ = false;
}

void AudioRecorderNode::init_parameters() {
    this->declare_parameter<int>("gain_ch1", gain_ch1_);
    this->declare_parameter<int>("gain_ch2", gain_ch2_);
    this->declare_parameter<int>("robot_code", robot_code_);
    this->declare_parameter<int>("duration_between_shoot", duration_between_shoot_);
    this->declare_parameter<int>("time_slot_duration", time_slot_duration_);
    this->declare_parameter<bool>("enable_chirp", enable_chirp_);

    this->declare_parameter<int>("frequency_middle", frequency_middle_);
    this->declare_parameter<int>("frequency_range", frequency_range_);
    this->declare_parameter<int>("signal_function", signal_function_);

    this->declare_parameter<long>("loop_safety_dt", loop_safety_dt_.count());
    this->declare_parameter<std::string>("audio_device", audio_device_);
    this->declare_parameter<int>("audio_nb_bits", audio_nb_bits_);
    this->declare_parameter<int>("audio_nb_channels", audio_nb_channels_);
    this->declare_parameter<int>("audio_frequency", audio_frequency_);
    this->declare_parameter<int>("audio_max_file_time", audio_max_file_time_);
    this->declare_parameter<int>("audio_hdd_space_limit_stop", audio_hdd_space_limit_stop_);
    this->declare_parameter<std::string>("audio_save_directory", audio_save_directory_);

    gain_ch1_ = this->get_parameter_or("gain_ch1", gain_ch1_);
    gain_ch2_ = this->get_parameter_or("gain_ch2", gain_ch2_);
    robot_code_ = this->get_parameter_or("robot_code", robot_code_);
    duration_between_shoot_ = this->get_parameter_or("duration_between_shoot", duration_between_shoot_);
    time_slot_duration_ = this->get_parameter_or("time_slot_duration", time_slot_duration_);
    frequency_middle_ = this->get_parameter_or("frequency_middle", frequency_middle_);
    frequency_range_ = this->get_parameter_or("frequency_range", frequency_range_);
    signal_function_ = this->get_parameter_or("signal_function", signal_function_);

    enable_chirp_ = this->get_parameter_or("enable_chirp", enable_chirp_);

    loop_safety_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_safety_dt", loop_safety_dt_.count()));
    audio_device_ = this->get_parameter_or("audio_device", audio_device_);
    audio_nb_bits_ = this->get_parameter_or("audio_nb_bits", audio_nb_bits_);
    audio_nb_channels_ = this->get_parameter_or("audio_nb_channels", audio_nb_channels_);
    audio_frequency_ = this->get_parameter_or("audio_frequency", audio_frequency_);
    audio_max_file_time_ = this->get_parameter_or("audio_max_file_time", audio_max_file_time_);
    audio_hdd_space_limit_stop_ = this->get_parameter_or("audio_hdd_space_limit_stop", audio_hdd_space_limit_stop_);
    audio_save_directory_ = this->get_parameter_or("audio_save_directory", audio_save_directory_);
}

void AudioRecorderNode::init_interfaces() {
    service_rosbag_ = this->create_service<std_srvs::srv::SetBool>(
            "restart_audio_record",
            std::bind(&AudioRecorderNode::callback_trigger, this, _1, _2, _3));

    service_chirp_ = this->create_service<std_srvs::srv::SetBool>(
            "chirp_enable",
            std::bind(&AudioRecorderNode::chirp_callback, this, _1, _2, _3));

    publisher_record_ = this->create_publisher<std_msgs::msg::Bool>("audio_record_sync", 10);

    subscriber_gnss_data_ = this->create_subscription<seabot2_msgs::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&AudioRecorderNode::gpsd_callback, this, _1));

    publisher_dspic_debug_ = this->create_publisher<seabot2_msgs::msg::SyncDspic>("audio_dspic", 10);
}

void AudioRecorderNode::gpsd_callback(const seabot2_msgs::msg::GpsFix &msg){
    if(msg.mode>=seabot2_msgs::msg::GpsFix::MODE_2D){
        gnss_fix_once_ = true;
    }
}

void AudioRecorderNode::chirp_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    enable_chirp_ = request->data;
    const bool ret = (dspic_.enable_chirp(enable_chirp_) == EXIT_SUCCESS);
    response->success = ret;
    response->message = "Chirp state changed";
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
