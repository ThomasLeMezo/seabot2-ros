#ifndef BUILD_LAMBERT_NODE_HPP
#define BUILD_LAMBERT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_msgs/msg/gps_fix.hpp"
#include "seabot2_msgs/msg/gnss_pose.hpp"
#include <deque>

#include <proj.h>

using namespace std::chrono_literals;
using namespace std;

class LambertNode final : public rclcpp::Node {
public:
    LambertNode();

private:

    /// Variable
    PJ_CONTEXT *C_;
    PJ *P_;
    PJ* P_for_GIS_;

    string epsg_source_ = "EPSG:4326";
    string epsg_target_ = "EPSG:2154";

    deque<double> east_memory_;
    deque<double> north_memory_;
    deque<rclcpp::Time> time_memory_;

    std::chrono::seconds filter_position_mean_ = 5s;
    std::chrono::seconds filter_dt_heading_computation_ = 15s;

    /// Interfaces
    rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_data_;
    rclcpp::Publisher<seabot2_msgs::msg::GnssPose>::SharedPtr publisher_lambert_data_;
    rclcpp::Publisher<seabot2_msgs::msg::GnssPose>::SharedPtr publisher_lambert_mean_data_;

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     * Callback of the external pressure
     * @param msg
     */
    void gnss_callback(const seabot2_msgs::msg::GpsFix &msg);

    /**
     * Compute mean position and heading
     */
    void compute_mean();

private:

};
#endif //BUILD_LAMBERT_NODE_HPP
