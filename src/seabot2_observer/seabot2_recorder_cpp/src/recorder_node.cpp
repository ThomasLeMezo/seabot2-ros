//
// Created by lemezoth on 06/06/23.
//

#include "seabot2_recorder_cpp/recorder_node.h"
#include <iostream>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <future>

using namespace std;
using namespace std::placeholders;

RecorderNode::RecorderNode()
        : Node("recorder_node"){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();

    // Find home directory and append log folder
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    workingDirectory_.append(homedir);
    workingDirectory_.append("/log/");

    // Create log folder if it does not exist
    if (!filesystem::exists(workingDirectory_)) {
        filesystem::create_directory(workingDirectory_);
    }

    // Change working directory to log folder
    if (chdir(workingDirectory_.c_str()) != 0) {
        std::cerr << "Error changing working directory to " << workingDirectory_ << std::endl;
    }

    // Start recording
    manage_subprocess_rosbag(true);

    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&RecorderNode::test_state, this));

    RCLCPP_INFO(this->get_logger(), "[recorder_node] Start Ok");
}

void RecorderNode::test_state() {
    if(thread_currently_running_ && subprocessFuture_.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready){
        RCLCPP_INFO(this->get_logger(), "[recorder_node] Subprocess crashed, restarting");
        manage_subprocess_rosbag(true);
    }
}

void RecorderNode::manage_subprocess_rosbag(bool start_new_bag) {

    // Check if the subprocess is still running
    if (thread_currently_running_) {
        wait_kill();
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
        RCLCPP_INFO(this->get_logger(), "[recorder_node] Start rosbag recording");
    }
}

void RecorderNode::wait_kill() {
    thread_currently_running_ = false;

    // Terminate the subprocess
    string command = "pkill -SIGTERM -f '"+ command_ +"'";
    std::system(command.c_str());
    // Wait for the subprocess thread to finish
    int result = subprocessFuture_.get();

    if (result == -1) {
        std::cerr << "Error: System call failed to start the subprocess." << std::endl;
    } else if (WIFEXITED(result)) {
        int exit_status = WEXITSTATUS(result);
        if (exit_status != 0) {
            std::cerr << "Subprocess terminated with exit code " << exit_status << std::endl;
        } else {
            std::cout << "Subprocess completed successfully." << std::endl;
        }
    } else if (WIFSIGNALED(result)) {
        int signal = WTERMSIG(result);
        std::cerr << "Subprocess terminated by signal " << signal << std::endl;
    } else {
        std::cerr << "Subprocess terminated with unknown status." << std::endl;
    }
}

RecorderNode::~RecorderNode() {
    wait_kill();
}

void RecorderNode::init_parameters() {

}

void RecorderNode::init_interfaces() {
    service_rosbag_ = this->create_service<std_srvs::srv::SetBool>(
            "restart_bag",
            std::bind(&RecorderNode::callback_trigger, this, _1, _2, _3));
}

void RecorderNode::callback_trigger(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    (void)request_header;
    (void)request;

    manage_subprocess_rosbag(request->data);

    // Set the response
    response->success = true;
    response->message = "Process rosbag request";
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecorderNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
