#ifndef BUILD_PING_NODE_H
#define BUILD_PING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <ping-device-ping1d.h>
#include <abstract-link.h>
#include <bluerobotics_ping_driver/msg/profile.hpp>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class PingNode : public rclcpp::Node {
public:
    PingNode();
    ~PingNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt

    /// Topics / Services
    rclcpp::Publisher<bluerobotics_ping_driver::msg::Profile>::SharedPtr publisher_profile_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_ping_enable_;

    /// Variables
    std::string uart_port_ = "/dev/ping1D";
    unsigned int uart_baudrate_ = 115200;
    std::unique_ptr<Ping1d> device_;
    std::shared_ptr<AbstractLink> port_;

    bool enable_ping_ = false;
    bool mode_auto_ = false; /// mode
    int speed_of_sound_ = 1550000; /// speed of sound [mm/s]
    int ping_interval_ = 200; /// interval [ms]
    int gain_setting_ = 1; /// gain [1, 2, 3, 4, 5, 6]


    /// Functions
    void timer_callback();

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

};

#endif //BUILD_PING_NODE_H
