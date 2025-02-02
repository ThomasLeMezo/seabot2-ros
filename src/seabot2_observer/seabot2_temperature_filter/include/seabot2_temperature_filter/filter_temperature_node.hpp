#ifndef BUILD_TEMPERATURE_FILTER_NODE_HPP
#define BUILD_TEMPERATURE_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "seabot2_msgs/msg/temperature_sensor_data.hpp"
#include <deque>

using namespace std::chrono_literals;
using namespace std;

class TemperatureFilterNode final : public rclcpp::Node {
public:
    TemperatureFilterNode();

private:
    size_t filter_window_size_ = 5;
    size_t filter_median_remove_side_samples_ = 1;

    /// Variable
    deque<double> temperature_memory_;

    /// Interfaces
    rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_data_;
    rclcpp::Publisher<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr publisher_temperature_data_;

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
    void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg);

    /**
     * Compute the median and mean filter
     * @param queue
     * @return
     */
    double compute_filter(deque<double> queue) const; /// Make a copy of the queue

private:

};
#endif //BUILD_TEMPERATURE_FILTER_NODE_HPP
