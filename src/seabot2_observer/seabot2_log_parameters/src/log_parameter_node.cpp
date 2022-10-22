#include "seabot2_log_parameters/log_parameter_node.hpp"
#include <algorithm>    // std::sort

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

using namespace placeholders;

LogParameterNode::LogParameterNode()
        : Node("log_parameter_node"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Start Ok");

    rclcpp::sleep_for(5s);

    record_parameters();
}

void LogParameterNode::init_parameters() {

}

void LogParameterNode::init_interfaces() {
    publisher_parameters_ = this->create_publisher<seabot2_log_parameters::msg::LogParameter>("parameters", 1);

}

std::vector<std::string> LogParameterNode::get_param_list(const std::string &node_name){
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr client_list =
            this->create_client<rcl_interfaces::srv::ListParameters>(node_name+"/list_parameters");

    auto request_list = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request_list->depth = 10;
    while (!client_list->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "[log_parameter_node] Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "[log_parameter_node] service not available, waiting again...");
    }

    auto result = client_list->async_send_request(request_list);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        return result.get()->result.names;
    }
    else{
        return std::vector<std::string>();
    }
}

void LogParameterNode::get_param_values(const std::string &node_name,
                                                            const std::vector<std::string> &param_name){
    if(param_name.empty())
        return;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_param_value =
            this->create_client<rcl_interfaces::srv::GetParameters>(node_name+"/get_parameters");

    auto request_param = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request_param->names = param_name;
    while (!client_param_value->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "[log_parameter_node] Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "[log_parameter_node] service not available, waiting again...");
    }

    auto result = client_param_value->async_send_request(request_param);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {

        auto param_values = result.get()->values;
        for(int i=0; i<param_values.size(); i++){
            seabot2_log_parameters::msg::LogParameter msg;
            msg.node_name = node_name;
            msg.param_name = param_name[i];
            msg.value = param_values[i];
            publisher_parameters_->publish(msg);
            rclcpp::sleep_for(100ms);
        }
    }
}

void LogParameterNode::record_parameters() {
    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Record");
    std::string current_node_name = this->get_fully_qualified_name();
    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Current node %s", current_node_name.c_str());
    std::vector<std::string> node_list = this->get_node_names();
    for(auto node_name:node_list){
        if(node_name.compare(current_node_name)!=0
            && node_name.find("rqt_gui") == std::string::npos
            && node_name.find("_ros2cli") == std::string::npos) {

            get_param_values(node_name, get_param_list(node_name));
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogParameterNode>());
    rclcpp::shutdown();
    return 0;
}