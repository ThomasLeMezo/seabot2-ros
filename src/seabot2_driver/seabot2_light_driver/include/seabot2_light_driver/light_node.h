#ifndef BUILD_LIGHT_NODE_H
#define BUILD_LIGHT_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_light_driver/light.h"
#include "seabot2_light_driver/srv/light.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std;

class LightNode : public rclcpp::Node {
public:
    LightNode();
    ~LightNode();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; /// loop dt

    /// I2C configuration
    Light light_;

    /// Topics / Services
    rclcpp::Service<seabot2_light_driver::srv::Light>::SharedPtr service_light_ ;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_flash_surface_ ;

    /// Variables
    rclcpp::Time time_turn_off_light_ = this->now();
    bool special_flash_ = false;
    bool light_is_on_ = false;
    bool is_surface_ = false;
    const int nb_surface_flash_ = 1;

    /// Functions
    void timer_callback();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     * Callback service flash surface
     * @param request_header
     * @param request
     * @param response
     */
    void service_flash_surface_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response);

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
