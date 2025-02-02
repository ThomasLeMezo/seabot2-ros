#ifndef BUILD_PING_NODE_H
#define BUILD_PING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <ping-device-ping1d.h>
#include <abstract-link.h>
#include <seabot2_msgs/msg/profile.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include <seabot2_msgs/msg/density.hpp>

using namespace std::chrono_literals;

class PingNode : public rclcpp::Node {
public:
    PingNode();
    ~PingNode();

/// Functions

    /**
     *
     */
    void wait_message();

private:

    /// Topics / Services
    rclcpp::Publisher<seabot2_msgs::msg::Profile>::SharedPtr publisher_profile_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_ping_enable_;
    rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;

    /// Variables
    std::string uart_port_ = "/dev/ping1D";
    unsigned int uart_baudrate_ = 115200;
    std::unique_ptr<Ping1d> device_;
    std::shared_ptr<AbstractLink> port_;

    bool enable_ping_ = false;
    bool mode_auto_ = true; /// mode
    double speed_of_sound_ = 1550.0; /// speed of sound [m/s]
    int ping_interval_ = 200; /// interval [ms]
    int gain_setting_ = 1; /// gain [1, 2, 3, 4, 5, 6]

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of the node
     */
    void init_interfaces();

    /**
     * Init the ping1D driver
     */
    void init_driver();

    /**
     *
     * @param request_header
     * @param request
     * @param response
     */
    void ping_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     *  Callback of the sound speed
     * @param msg
     */
    void sound_speed_callback(const seabot2_msgs::msg::Density &msg);
};

#endif //BUILD_PING_NODE_H
