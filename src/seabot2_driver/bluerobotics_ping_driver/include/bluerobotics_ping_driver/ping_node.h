#ifndef BUILD_PING_NODE_H
#define BUILD_PING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <ping-device-ping1d.h>
#include <abstract-link.h>
#include <bluerobotics_ping_driver/msg/profile.hpp>

using namespace std::chrono_literals;
using namespace std;

class PingNode : public rclcpp::Node {
public:
    PingNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt

    /// Topics / Services
    rclcpp::Publisher<bluerobotics_ping_driver::msg::Profile>::SharedPtr publisher_profile_;

    /// Variables
    string uart_port_ = "/dev/ttyAMA3";
    std::unique_ptr<Ping1d> device_;
    std::shared_ptr<AbstractLink> port_;

    bool enable_ping_ = true;
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
     * Init topics to this node (publishers & subscribers)
     */
    void init_topics();

    /**
     * Init services od this node
     */
    void init_services();

    /**
     * Init the ping1D driver
     */
    void init_driver();

};

#endif //BUILD_PING_NODE_H
