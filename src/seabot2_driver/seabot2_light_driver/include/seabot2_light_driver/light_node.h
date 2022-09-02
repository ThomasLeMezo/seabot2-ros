#ifndef BUILD_LIGHT_NODE_H
#define BUILD_LIGHT_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_light_driver/light.h"
#include "seabot2_light_driver/srv/light.hpp"

using namespace std::chrono_literals;
using namespace std;

class LightNode : public rclcpp::Node {
public:
    LightNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt

    /// I2C configuration
    Light light_;

    /// Topics / Services
    rclcpp::Service<seabot2_light_driver::srv::Light>::SharedPtr service_light_ ;

    /// Variables
    rclcpp::Time time_turn_off_light_ = this->now();
    bool light_is_on_ = false;

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
     *
     * @param request
     * @param response
     */
    void service_light_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<seabot2_light_driver::srv::Light::Request> request,
                                           std::shared_ptr<seabot2_light_driver::srv::Light::Response> response);

};

#endif //BUILD_LIGHT_NODE_H
