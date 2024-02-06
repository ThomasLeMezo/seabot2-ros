//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_RECORDER_NODE_H
#define BUILD_RECORDER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>
#include <filesystem>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class AudioRecorderNode : public rclcpp::Node {
public:
    AudioRecorderNode();

    ~AudioRecorderNode();

public:
    const std::string command_ = "arecord -D dmic_sv -f S32_LE -c2 -r 192000 -t wav -v --max-file-time 900 --use-strftime %Y/%m/%d/listen-%H-%M-%v.wav";

private:

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::string workingDirectory_ = "";
    bool thread_currently_running_ = false;
    std::future<int> subprocessFuture_;

    bool ask_restart_ = false;

    /// Interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_rosbag_;

    /// Parameters

    /// Functions

    void manage_subprocess(bool start_new_bag=true);

    void wait_kill();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    /**
     * Callback for the service trigger
     * @param request
     * @param response
     */
    void callback_trigger(const std::shared_ptr<rmw_request_id_t> request_header,
                          const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

};

#endif //BUILD_RECORDER_NODE_H
