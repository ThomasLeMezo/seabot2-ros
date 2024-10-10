//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_RECORDER_NODE_H
#define BUILD_RECORDER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>
#include <filesystem>
#include "std_srvs/srv/set_bool.hpp"
#include "seabot2_audio_recorder/tlv320adc.h"
#include "std_msgs/msg/bool.hpp"
#include "seabot2_audio_recorder/dspic_acoustic.h"
#include "gpsd_client/msg/gps_fix.hpp"
#include "std_msgs/msg/byte.hpp"

using namespace std::chrono_literals;

class AudioRecorderNode : public rclcpp::Node {
public:
    AudioRecorderNode();

    ~AudioRecorderNode();

public:
    // --max-file-time 900
    const std::string command_ = "arecord -D hw:CARD=sndrpii2scard -f S32_LE -c2 -r 96000 -t wav -v --use-strftime %Y/%m/%d/listen-%H-%M-%v.wav --max-file-time 600";

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds  loop_dt_ = 1s; // loop dt

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::string workingDirectory_ = "";
    bool thread_currently_running_ = false;
    std::future<int> subprocessFuture_;

    TLV320ADC tlv_;
    DspicAcoustic dspic_;

    uint8_t gain_ch1_ = 0;
    uint8_t gain_ch2_ = 0;
    uint16_t robot_code_ = 0;
    uint16_t duration_between_shoot_ = 30; //s
    uint16_t time_slot_duration_ = 5; // s

    bool enable_chirp_ = false;
    uint16_t frequency_middle_, frequency_range_;

    bool gnss_fix_once_ = false;

    /// Interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_rosbag_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_chirp_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_record_;

    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr publisher_pps_;

    rclcpp::Subscription<gpsd_client::msg::GpsFix>::SharedPtr subscriber_gnss_data_;

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

    /**
     *
     * @param msg
     */
    void gpsd_callback(const gpsd_client::msg::GpsFix &msg);

    /**
     *
     */
    void timer_callback();

    /**
     * Callback for the service chirp
     * @param request_header
     * @param request
     * @param response
     */
    void chirp_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

#endif //BUILD_RECORDER_NODE_H
