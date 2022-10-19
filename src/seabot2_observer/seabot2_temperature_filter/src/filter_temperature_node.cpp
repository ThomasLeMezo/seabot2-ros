#include "seabot2_temperature_filter/filter_temperature_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

TemperatureFilterNode::TemperatureFilterNode()
        : Node("filter_temperature_node"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Filter_internal_sensor_node] Start Ok");
}

void TemperatureFilterNode::init_parameters() {
    this->declare_parameter<long>("filter_window_size", filter_window_size_);
    this->declare_parameter<long>("filter_median_remove_side_samples", filter_median_remove_side_samples_);

    filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
    filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples", filter_median_remove_side_samples_);
}

double TemperatureFilterNode::compute_filter(deque<double> queue) const {
    /// Sort to take median
    sort(queue.begin(), queue.end());
    /// Remove side values
    deque<double> queue_median(queue.begin()+filter_median_remove_side_samples_, queue.end()-filter_median_remove_side_samples_);
    /// Sum elements
    double data_sum = std::accumulate(queue_median.begin(), queue_median.end(), 0.0);
    /// Compute mean value
    return data_sum / (double)queue_median.size();
}

void TemperatureFilterNode::temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg) {
    temperature_tsys01_driver::msg::TemperatureSensorData msg_filter;

    /// Add new data to deque for pressure
    temperature_memory_.push_front(msg.temperature);

    if(temperature_memory_.size()>filter_window_size_) {
        temperature_memory_.pop_back();
    }

    if(temperature_memory_.size()==filter_window_size_){
        msg_filter.temperature = compute_filter(temperature_memory_);
        publisher_temperature_data_->publish(msg_filter);
    }
}

void TemperatureFilterNode::init_interfaces() {
    publisher_temperature_data_ = this->create_publisher<temperature_tsys01_driver::msg::TemperatureSensorData>("temperature", 1);

    subscriber_temperature_data_ = this->create_subscription<temperature_tsys01_driver::msg::TemperatureSensorData>(
            "/driver/temperature", 10, std::bind(&TemperatureFilterNode::temperature_callback, this, _1));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureFilterNode>());
    rclcpp::shutdown();
    return 0;
}