#include "seabot2_log_parameters/log_parameter_node.hpp"
#include <unistd.h>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

using namespace placeholders;

LogParameterNode::LogParameterNode()
        : Node("log_parameter_node"){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Start Ok");
}

void LogParameterNode::service_record(const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    record_parameters();
    response->success = true;
}

void LogParameterNode::init_interfaces() {
    publisher_parameters_ = this->create_publisher<seabot2_msgs::msg::LogParameter>("parameters", 1);

    service_log_parameters_ = this->create_service<std_srvs::srv::Trigger>("log_parameters",
                                                                            bind(&LogParameterNode::service_record, this, _1, _2, _3),
                                                                            rmw_qos_profile_services_default, callback_group_);
}

std::vector<std::string> LogParameterNode::get_param_list(const std::string &node_name){
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr client_list =
            this->create_client<rcl_interfaces::srv::ListParameters>(node_name+"/list_parameters",
                                                                     rmw_qos_profile_services_default, callback_group_);

    const auto request_list = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request_list->depth = 10;
    while (!client_list->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "[log_parameter_node] Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "[log_parameter_node] service not available, waiting again...");
    }

    auto future = client_list->async_send_request(request_list);

    if (const auto status = future.wait_for(3s); status == std::future_status::ready){
        return future.get()->result.names;
    }
    else{
        RCLCPP_WARN(get_logger(), "[log_parameter_node] Fail get response from param_value %s", node_name.c_str());
        client_list->remove_pending_request(future);
        return {};
    }
}

void LogParameterNode::get_param_values(const std::string &node_name,
                                                            const std::vector<std::string> &param_name){
    if(param_name.empty())
        return;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_param_value =
            this->create_client<rcl_interfaces::srv::GetParameters>(node_name+"/get_parameters",
                                                                    rmw_qos_profile_services_default, callback_group_);

    const auto request_param = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request_param->names = param_name;
    while (!client_param_value->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "[log_parameter_node] Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "[log_parameter_node] service not available, waiting again...");
    }

    auto future = client_param_value->async_send_request(request_param);

    if (const auto status = future.wait_for(3s); status == std::future_status::ready)
    {
        auto param_values = future.get()->values;
        for(size_t i=0; i<param_values.size(); i++){
            seabot2_msgs::msg::LogParameter msg;
            msg.node_name = node_name;
            msg.param_name = param_name[i];
            msg.value = param_values[i];
            publisher_parameters_->publish(msg);
            rclcpp::sleep_for(100ms);
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "[log_parameter_node] Fail get response from param_value %s", node_name.c_str());
        client_param_value->remove_pending_request(future);
    }
}

void LogParameterNode::record_parameters() {
    rclcpp::sleep_for(30s); // Wait to be sure log is started & seabot2_depth_control has finished computing parameters
    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Start recording parameters");

    // Get hostname
    char hostname[40];
    gethostname(hostname, 40);
    seabot2_msgs::msg::LogParameter msg;
    msg.node_name = "linux";
    msg.param_name = "/hostname";
    msg.value.string_value = std::string(hostname);
    msg.value.type = 4;
    publisher_parameters_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] %s %s %s %i", msg.node_name.c_str(), msg.param_name.c_str(), msg.value.string_value.c_str(), msg.value.type);

    // Get parameters
    const std::string current_node_name = this->get_fully_qualified_name();
    std::vector<std::string> node_list = this->get_node_names();
    for(auto node_name:node_list){
        if(node_name.compare(current_node_name)!=0
            && node_name.find("rqt_gui") == std::string::npos
            && node_name.find("_ros2cli") == std::string::npos) {
            get_param_values(node_name, get_param_list(node_name));
        }
    }
    RCLCPP_INFO(this->get_logger(), "[log_parameter_node] Parameters have been recorded");
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogParameterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}