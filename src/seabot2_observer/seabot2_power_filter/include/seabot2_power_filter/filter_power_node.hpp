#ifndef BUILD_FILTER_POWER_NODE_HPP
#define BUILD_FILTER_POWER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_msgs/msg/power_state.hpp"
#include <deque>

using namespace std::chrono_literals;
using namespace std;

class FilterPowerNode final : public rclcpp::Node {
public:
    FilterPowerNode();

private:

    /// Rclcpp
    size_t filter_window_size_ = 5;
    size_t filter_median_remove_side_samples_ = 1;

    /// Variable
    deque<double> battery_volt_memory_;
    deque<double> motor_current_memory_;
    array<deque<double>,2> esc_current_memory_;
    array<deque<double>, 4> cell_volt_memory_;

    /// Interfaces
    rclcpp::Subscription<seabot2_msgs::msg::PowerState>::SharedPtr subscriber_power_data_;
    rclcpp::Publisher<seabot2_msgs::msg::PowerState>::SharedPtr publisher_power_data_;

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
     * Callback of the power data
     * @param msg
     */
    void power_callback(const seabot2_msgs::msg::PowerState &msg);

    /**
     * Compute the median and mean filter
     * @param queue
     * @return
     */
    double compute_filter(deque<double> queue) const; /// Make a copy of the queue

private:

};
#endif //BUILD_FILTER_POWER_NODE_HPP
