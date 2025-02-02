//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_RECORDER_NODE_H
#define BUILD_RECORDER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "seabot2_audio_recorder/tlv320adc.h"
#include "seabot2_audio_recorder/dspic_acoustic.h"

#include <chrono>
#include <memory>
#include <filesystem>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "seabot2_msgs/msg/gps_fix.hpp"
#include "seabot2_msgs/msg/sync_dspic.hpp"

using namespace std::chrono_literals;

class AudioRecorderNode final : public rclcpp::Node {
public:
    AudioRecorderNode();

    ~AudioRecorderNode() override;

    std::string get_arecord_command() const;

private:

    int audio_frequency_ = 192000;
    int audio_max_file_time_ = 600; // s
    int audio_nb_channels_ = 1;
    int audio_nb_bits_ = 32;
    std::string audio_device_ = "hw:CARD=sndrpii2scard";
    std::string audio_command_last_ = "";
    int audio_hdd_space_limit_stop_ = 500; // MB

    std::string audio_save_directory_ = "/audio/";

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds  loop_safety_dt_ = 5s; // loop dt

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
    uint16_t frequency_middle_ = 40000;
    uint16_t frequency_range_ = 5000;
    uint8_t signal_function_ = 0;

    bool gnss_fix_once_ = false;
    bool dspic_posix_fix_ = false;

    /// Interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_rosbag_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_chirp_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_record_;

    rclcpp::Publisher<seabot2_msgs::msg::SyncDspic>::SharedPtr publisher_dspic_debug_;

    rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_data_;

    /// Parameters

    /// Functions

    void manage_subprocess(bool start_new_audio);

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
     * @param request_header
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
    void gpsd_callback(const seabot2_msgs::msg::GpsFix &msg);

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
