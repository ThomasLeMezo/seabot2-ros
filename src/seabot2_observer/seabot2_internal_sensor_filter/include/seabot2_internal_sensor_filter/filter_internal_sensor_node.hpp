//
// Created by lemezoth on 05/09/22.
//

#ifndef BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP
#define BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include <deque>

using namespace std::chrono_literals;
using namespace std;

class InternalSensorFilterNode : public rclcpp::Node {
public:
    InternalSensorFilterNode();

private:
    size_t filter_window_size_ = 5;
    size_t filter_median_remove_side_samples_ = 1;

    /// Variable
    deque<double> pressure_memory_;
    deque<double> temperature_memory_;
    deque<double> humidity_memory_;

    /// Interfaces
    rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_pressure_data_;
    rclcpp::Publisher<pressure_bme280_driver::msg::Bme280Data>::SharedPtr publisher_pressure_data_;

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
    void pressure_callback(const pressure_bme280_driver::msg::Bme280Data &msg);

    /**
     * Compute the median and mean filter
     * @param queue
     * @return
     */
    double compute_filter(deque<double> queue) const; /// Make a copy of the queue

private:

};
#endif //BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP
