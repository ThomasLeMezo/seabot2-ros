#include "seabot2_mission/mission_node.hpp"

using namespace placeholders;

MissionNode::MissionNode()
        : Node("mission_node"), mission_(){

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    init_parameters();
    init_interfaces();

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
    this->declare_parameter<double>("temperature_keeping_k", mission_.temperature_keeping_k_);

    mission_file_name_ = this->get_parameter_or("mission_file_name", mission_file_name_);
    mission_path_ = this->get_parameter_or("mission_path", mission_path_);
    flash_next_waypoint_time_ = this->get_parameter_or("flash_next_waypoint_time", flash_next_waypoint_time_);
    flash_number_ = this->get_parameter_or("flash_number", flash_number_);
    limit_velocity_default_ = this->get_parameter_or("limit_velocity_default", limit_velocity_default_);
    mission_.temperature_keeping_k_ = this->get_parameter_or("temperature_keeping_k", mission_.temperature_keeping_k_);

}

void MissionNode::init_interfaces() {

    service_mission_reload_ = this->create_service<std_srvs::srv::Trigger>("mission_reload",
                                                                            std::bind(&MissionNode::service_mission_reload_callback, this, _1, _2, _3),
                                                                            rmw_qos_profile_services_default,callback_group_);

    service_mission_enable_ = this->create_service<std_srvs::srv::SetBool>("mission_enable",
                                                                           std::bind(&MissionNode::service_mission_enable_callback, this, _1, _2, _3),
                                                                           rmw_qos_profile_services_default,callback_group_);

    subscriber_depth_data_ = this->create_subscription<seabot2_msgs::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&MissionNode::depth_callback, this, _1));

    subscriber_temperature_data_ = this->create_subscription<seabot2_msgs::msg::TemperatureSensorData>(
            "/observer/temperature", 10, std::bind(&MissionNode::temperature_callback, this, _1));

    publisher_mission_state_ = this->create_publisher<seabot2_msgs::msg::MissionState>("mission_state", 10);

    publisher_depth_control_set_point_ = this->create_publisher<seabot2_msgs::msg::DepthControlSetPoint>("depth_control_set_point", 10);

    publisher_temperature_keeping_debug_ = this->create_publisher<seabot2_msgs::msg::TemperatureKeepingDebug>("temperature_keeping_debug", 10);

    client_light_ = this->create_client<seabot2_srvs::srv::Light>("/driver/light", rmw_qos_profile_services_default,callback_group_);

    client_log_parameters_ = this->create_client<std_srvs::srv::Trigger>("/observer/log_parameters", rmw_qos_profile_services_default,callback_group_);

    client_bag_recorder_ = this->create_client<std_srvs::srv::Trigger>("/observer/restart_bag", rmw_qos_profile_services_default,callback_group_);
}

void MissionNode::depth_callback(const seabot2_msgs::msg::DepthPose::SharedPtr msg) {
    mission_.update_depth(msg->depth);
}

void MissionNode::temperature_callback(const seabot2_msgs::msg::TemperatureSensorData::SharedPtr msg) {
    mission_.update_temperature(msg->temperature);
}

void MissionNode::call_light(){
    auto request = std::make_shared<seabot2_srvs::srv::Light::Request>();
    request->duration = flash_next_waypoint_time_;
    request->number_of_flash = flash_number_;

    client_light_->wait_for_service(500ms);
    if (!client_light_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Mission_node] Light service not available");
    }
    else {
        if(rclcpp::ok()) {
            auto result = client_light_->async_send_request(request);
            // Do not wait for the result.
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Mission_node] rclcpp not ok");
        }
    }
}

void MissionNode::call_log_params() const {
    const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
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

void MissionNode::call_restart_bag() const {
    const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
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

int MissionNode::load_mission(){
    // Call for a new ros2 bag
    call_restart_bag();
    rclcpp::sleep_for(3s); // Wait for the change of bag

    // Reload mission
    const int ret =  mission_.load_mission(mission_file_name_, mission_path_, this->now());

    // Call for log of parameters
    call_log_params();

    return ret;
}

void MissionNode::timer_callback() {
    if(mission_.is_new_mission_file(mission_file_name_, mission_path_)){
        load_mission();
    }

    // Update the state of the mission
    const bool is_new_waypoint = mission_.update_state(this->now());

    // Publish depth control set point
    publisher_depth_control_set_point_->publish(mission_.get_depth_control_set_point());

    // Publish Temperature Keeping waypoint data (if it is activated)
    if(mission_.is_current_waypoint_of_type(Mission::WP_TEMPERATURE_KEEPING)) {
        shared_ptr<WaypointTemperatureKeeping> wtk = mission_.get_current_waypoint_temperature_keeping();
        if (wtk != nullptr) {
            seabot2_msgs::msg::TemperatureKeepingDebug msg_debug;
            msg_debug.temperature = wtk->temperature_;
            msg_debug.error = wtk->error_temperature_;
            msg_debug.header.stamp = this->now();
            publisher_temperature_keeping_debug_->publish(msg_debug);
        }
    }

    // Publish mission state
    seabot2_msgs::msg::MissionState state_msg;
    state_msg.mode = mission_.get_mission_mode();
    state_msg.state = mission_.get_mission_state();
    state_msg.waypoint_id = mission_.get_current_waypoint_id();
    state_msg.waypoint_length = mission_.get_number_waypoints();
    state_msg.time_to_next_waypoint = mission_.get_time_to_next_waypoint();
    state_msg.header.stamp = this->now();
    publisher_mission_state_->publish(state_msg);

    if(is_new_waypoint)
        call_light();
}

void MissionNode::service_mission_reload_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    if(const int error_code = load_mission(); error_code==EXIT_SUCCESS)
        response->success = true;
    else
        response->success = false;
}

void MissionNode::service_mission_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    mission_enable_ = request->data;
}

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
