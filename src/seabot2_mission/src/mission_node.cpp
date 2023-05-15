#include "seabot2_mission/mission_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

MissionNode::MissionNode()
        : Node("mission_node"), mission_(this){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();
    init_interfaces();

    rclcpp::sleep_for(30s); // Wait to be sure to log mission data
    mission_.is_new_mission_file(mission_file_name_, mission_path_);
    load_mission();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&MissionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Mission_node] Start Ok");
}

void MissionNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<string>("mission_file_name", mission_file_name_);
    this->declare_parameter<string>("mission_path", mission_path_);
    this->declare_parameter<double>("flash_next_waypoint_time", flash_next_waypoint_time_);
    this->declare_parameter<int>("flash_number", flash_number_);
    this->declare_parameter<double>("limit_velocity_default", limit_velocity_default_);


    mission_file_name_ = this->get_parameter_or("mission_file_name", mission_file_name_);
    mission_path_ = this->get_parameter_or("mission_path", mission_path_);

    flash_next_waypoint_time_ = this->get_parameter_or("flash_next_waypoint_time", flash_next_waypoint_time_);
    flash_number_ = this->get_parameter_or("flash_number", flash_number_);
    limit_velocity_default_ = this->get_parameter_or("limit_velocity_defualt", limit_velocity_default_);

}

void MissionNode::init_interfaces() {

    service_mission_reload_ = this->create_service<std_srvs::srv::Trigger>("mission_reload",
                                                                            std::bind(&MissionNode::service_mission_reload_callback, this, _1, _2, _3),
                                                                            rmw_qos_profile_services_default,callback_group_);

    service_mission_enable_ = this->create_service<std_srvs::srv::SetBool>("mission_enable",
                                                                           std::bind(&MissionNode::service_mission_enable_callback, this, _1, _2, _3),
                                                                           rmw_qos_profile_services_default,callback_group_);

    publisher_waypoint_ = this->create_publisher<seabot2_mission::msg::Waypoint>("waypoint", 10);

    client_light_ = this->create_client<seabot2_light_driver::srv::Light>("/driver/light", rmw_qos_profile_services_default,callback_group_);

    client_log_parameters_ = this->create_client<std_srvs::srv::Trigger>("/observer/log_parameters", rmw_qos_profile_services_default,callback_group_);

    client_alpha_mission_ =  this->create_client<seabot2_mission::srv::AlphaMission>("/control/alpha_mission", rmw_qos_profile_services_default,callback_group_);

    client_bag_recorder_ = this->create_client<std_srvs::srv::Trigger>("/observer/restart_bag", rmw_qos_profile_services_default,callback_group_);
}

void MissionNode::call_light(){
    auto request = std::make_shared<seabot2_light_driver::srv::Light::Request>();
    request->duration = flash_next_waypoint_time_;
    request->number_of_flash = flash_number_;

    client_light_->wait_for_service(500ms);
    if (!client_light_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Mission_node] Light service not available");
    }
    else {
        if(rclcpp::ok()) {
            auto result = client_light_->async_send_request(request);
            // Wait for the result.
//            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
//                rclcpp::FutureReturnCode::SUCCESS) {
//                RCLCPP_INFO(this->get_logger(), "[Mission_node] Fail calling Light service");
//            }
//            else{
//                RCLCPP_INFO(this->get_logger(), "[Mission_node] Call light to flash");
//            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Mission_node] rclcpp not ok");
        }
    }
}

void MissionNode::call_log_params(){
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_log_parameters_->wait_for_service(500ms);
    if (!client_log_parameters_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Mission_node] Log Parameters service not available");
    }
    else {
        if(rclcpp::ok()) {
            auto future = client_log_parameters_->async_send_request(request);

            // Do not wait to the result because cannont handle async call (deadlock with this node)
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Mission_node] rclcpp not ok");
        }
    }
}

void MissionNode::call_restart_bag(){
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_bag_recorder_->wait_for_service(500ms);
    if (!client_bag_recorder_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Mission_node] Bag recorder service not available");
    }
    else {
        if(rclcpp::ok()) {
            auto future = client_bag_recorder_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "[Mission_node] Call to restart bag");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Mission_node] rclcpp not ok");
        }
    }
}

void MissionNode::call_velocity_computation(std::vector<float> &velocity_list){
    auto request = std::make_shared<seabot2_mission::srv::AlphaMission::Request>();
    request->velocity_limits = velocity_list;
    client_alpha_mission_->wait_for_service(500ms);
    if (!client_alpha_mission_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Mission_node] Alpha Mission service not available");
    }
    else {
        if(rclcpp::ok()) {
            auto future = client_alpha_mission_->async_send_request(request);

            // Do not wait to the result because cannont handle async call (deadlock with this node)
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Mission_node] rclcpp not ok");
        }
    }
}

int MissionNode::load_mission(){
    // Call for a new ros2 bag
    call_restart_bag();
    rclcpp::sleep_for(1s); // Wait for the change of bag

    // Call for log of parameters
    call_log_params();

    // Reload mission
    int ret =  mission_.load_mission(mission_file_name_, mission_path_);

    if(ret==EXIT_SUCCESS){
        vector<float> velocity_list = mission_.get_velocity_list();
        for(auto &v: velocity_list)
            RCLCPP_INFO(this->get_logger(), "[Mission_node] Velocity to compute : %f", v);
        call_velocity_computation(velocity_list);
    }

    return ret;
}

void MissionNode::timer_callback() {
    if(mission_.is_new_mission_file(mission_file_name_, mission_path_)){
        load_mission();
    }

    seabot2_mission::msg::Waypoint wp_msg;
    bool is_new_waypoint = mission_.compute_command(wp_msg);

    if(!mission_enable_) /// Check if mission was disabled by service
        wp_msg.mission_enable = false;

    wp_msg.header.stamp = this->now();
    publisher_waypoint_->publish(wp_msg);

    if(is_new_waypoint)
        call_light();
}

void MissionNode::service_mission_reload_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    int error_code = load_mission();
    if(error_code==EXIT_SUCCESS)
        response->success = true;
    else
        response->success = false;
}

void MissionNode::service_mission_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    mission_enable_ = request->data;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

// FILE *proc_bag_ = nullptr;
// #include <stdio.h>

//if(proc_bag_ != nullptr){
//int ret = pclose(proc_bag_);
//if(ret != EXIT_SUCCESS) {
//response->success = false;
//return;
//}
//}
//
//proc_bag_ = popen("ros2 bag record -a", "r");
//response->success = true;