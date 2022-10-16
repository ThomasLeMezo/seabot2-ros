#include "seabot2_power_filter/filter_power_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

FilterPowerNode::FilterPowerNode()
        : Node("filter_power_node"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Filter_power_node] Start Ok");
}

void FilterPowerNode::init_parameters() {
    this->declare_parameter<long>("filter_window_size", filter_window_size_);
    this->declare_parameter<long>("filter_median_remove_side_samples", filter_median_remove_side_samples_);

    filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
    filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples", filter_median_remove_side_samples_);
}

double FilterPowerNode::compute_filter(deque<double> queue) const {
    /// Sort to take median
    sort(queue.begin(), queue.end());
    /// Remove side values
    deque<double> queue_median(queue.begin()+filter_median_remove_side_samples_, queue.end()-filter_median_remove_side_samples_);
    /// Sum elements
    double data_sum = std::accumulate(queue_median.begin(), queue_median.end(), 0.0);
    /// Compute mean value
    return data_sum / (double)queue_median.size();
}

void FilterPowerNode::power_callback(const seabot2_power_driver::msg::PowerState &msg) {
    seabot2_power_driver::msg::PowerState msg_filter;
    msg_filter.header.stamp = msg.header.stamp;

    /// Add new data to deque
    battery_volt_memory_.push_front(msg.battery_volt);
    motor_current_memory_.push_front(msg.motor_current);
    for(size_t i=0; i<2; i++)
        esc_current_memory_[i].push_front(msg.esc_current[i]);
    for(size_t i=0; i<4; i++)
        cell_volt_memory_[i].push_front(msg.cell_volt[i]);

    if(battery_volt_memory_.size()>filter_window_size_) {
        battery_volt_memory_.pop_back();
        motor_current_memory_.pop_back();
        for(size_t i=0; i<2; i++)
            esc_current_memory_[i].pop_back();
        for(size_t i=0; i<4; i++)
            cell_volt_memory_[i].pop_back();
    }

    if(battery_volt_memory_.size()==filter_window_size_){
        msg_filter.battery_volt = compute_filter(battery_volt_memory_);
        msg_filter.motor_current = compute_filter(motor_current_memory_);
        msg_filter.header.stamp = msg.header.stamp;
        for(size_t i=0; i<2; i++)
            msg_filter.esc_current[i] = compute_filter(esc_current_memory_[i]);
        for(size_t i=0; i<4; i++)
            msg_filter.cell_volt[i] = compute_filter(cell_volt_memory_[i]);
        publisher_power_data_->publish(msg_filter);
    }
}

void FilterPowerNode::init_interfaces() {
    publisher_power_data_ = this->create_publisher<seabot2_power_driver::msg::PowerState>("power", 1);

    subscriber_power_data_ = this->create_subscription<seabot2_power_driver::msg::PowerState>(
            "/driver/power", 10, std::bind(&FilterPowerNode::power_callback, this, _1));

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterPowerNode>());
    rclcpp::shutdown();
    return 0;
}