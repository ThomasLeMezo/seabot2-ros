#include "seabot2_depth_filter/depth_pose_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

DepthPoseNode::DepthPoseNode()
        : Node("depth_pose_node"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Depth_pose_node] Start Ok");
}

void DepthPoseNode::init_parameters() {

    this->declare_parameter<double>("physics_rho_", rho_);
    this->declare_parameter<double>("physics_g_", g_);
    this->declare_parameter<double>("physics_velocity_limit", velocity_limit_);

    rho_ = this->get_parameter_or("physics_rho_", rho_);
    g_ = this->get_parameter_or("physics_g_", g_);
    velocity_limit_ = this->get_parameter_or("physics_velocity_limit", velocity_limit_);

    this->declare_parameter<long>("velocity_dt_gap_sample", velocity_dt_gap_sample_);
    this->declare_parameter<long>("filter_velocity_median_remove_side_samples", filter_velocity_median_remove_side_samples_);
    this->declare_parameter<long>("filter_window_size", filter_window_size_);
    this->declare_parameter<long>("filter_median_remove_side_samples", filter_median_remove_side_samples_);

    velocity_dt_gap_sample_ = this->get_parameter_or("velocity_dt_gap_sample", velocity_dt_gap_sample_);
    filter_velocity_median_remove_side_samples_ = this->get_parameter_or("filter_velocity_median_remove_side_samples", filter_velocity_median_remove_side_samples_);
    filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
    filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples", filter_median_remove_side_samples_);
}

void DepthPoseNode::service_zero_pressure_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    if(!pressure_zero_depth_deque_.empty()){
        zero_depth_ = std::accumulate(pressure_zero_depth_deque_.begin(), pressure_zero_depth_deque_.end(), 0.0);
        zero_depth_ /= (double)pressure_zero_depth_deque_.size();
        RCLCPP_INFO(this->get_logger(),"[Depth_pose_node] Zero_depth = %f", zero_depth_);
        response->success = true;
    }
    else
        response->success = false;
}

void DepthPoseNode::pressure_callback(const pressure_ms5803_driver::msg::PressureSensorData &msg) {
    seabot2_depth_filter::msg::DepthPose msg_pose;
    msg_pose.header.stamp = msg.header.stamp;
    msg_pose.zero_depth_pressure = zero_depth_;

    /// Add new data to deque for pressure
    pressure_deque_.push_front(msg.pressure);
    if(pressure_deque_.size()>filter_window_size_)
        pressure_deque_.pop_back();

    /// Add new data to deque for zero_pressure
    pressure_zero_depth_deque_.push_back(msg.pressure);
    if(pressure_zero_depth_deque_.size()>zero_depth_window_size_)
        pressure_zero_depth_deque_.pop_back();

    if(pressure_deque_.size()==filter_window_size_){
        seabot2_depth_filter::msg::DepthPose msg_depth;
        /// ************** Compute depth ************** //
        /// MEDIAN + MEAN FILTER
        /// Make a copy
        deque<double> pressure_deque_sort(pressure_deque_);
        /// Sort to take median
        sort(pressure_deque_sort.begin(), pressure_deque_sort.end());
        /// Remove side values
        deque<double> pressure_deque_median(pressure_deque_sort.begin()+filter_median_remove_side_samples_, pressure_deque_sort.end()-filter_median_remove_side_samples_);
        /// Sum elements
        double pressure_sum = std::accumulate(pressure_deque_median.begin(), pressure_deque_median.end(), 0.0);
        /// Compute mean value
        double pressure_mean = pressure_sum / (double)pressure_deque_median.size();

        double pressure = (pressure_mean - zero_depth_);
        double depth = pressure / (g_*rho_/1e5); /// 1e5 for Pa to Bar

        msg_pose.depth = depth;
        msg_pose.pressure = pressure;

        /// ************** Compute velocity ************** //
        depth_memory_.push_front(std::pair<double,rclcpp::Time>(depth, msg.header.stamp));
        if(depth_memory_.size()>(velocity_dt_gap_sample_+filter_velocity_window_size_))
            depth_memory_.pop_back();

        if(depth_memory_.size()==(velocity_dt_gap_sample_+filter_velocity_window_size_)){
            vector<double> velocities;
            for(size_t i=0; i<filter_velocity_window_size_; i++){
                // Delta_depth / Delta_dt
                rclcpp::Duration dt = (depth_memory_[i].second-depth_memory_[velocity_dt_gap_sample_+i].second);
                if(dt.seconds()!=0.)
                    velocities.push_back((depth_memory_[i].first-depth_memory_[velocity_dt_gap_sample_+i].first)/dt.seconds());
            }
            if(velocities.size()>(2*filter_velocity_median_remove_side_samples_+1)) {
                /// Sort to take median
                sort(velocities.begin(), velocities.end());
                /// Remove side values
                vector<double> velocity_median(velocities.begin() + filter_velocity_median_remove_side_samples_,
                                               velocities.end() - filter_velocity_median_remove_side_samples_);
                /// Sum elements and compute mean
                double velocity =
                        std::accumulate(velocity_median.begin(), velocity_median.end(), 0.0) / velocity_median.size();
                /// Saturate the possible values
                velocity = std::clamp(velocity, -velocity_limit_, velocity_limit_);
                msg_pose.velocity = velocity;
            }
        }
        publisher_depth_data_->publish(msg_pose);
    }
}

void DepthPoseNode::init_interfaces() {
    publisher_depth_data_ = this->create_publisher<seabot2_depth_filter::msg::DepthPose>("depth", 1);

    subscriber_pressure_data_ = this->create_subscription<pressure_ms5803_driver::msg::PressureSensorData>(
            "/driver/pressure_external", 10, std::bind(&DepthPoseNode::pressure_callback, this, _1));

    service_zero_depth_ = this->create_service<std_srvs::srv::Trigger>("zero_pressure",
                                                                            std::bind(&DepthPoseNode::service_zero_pressure_callback, this, _1, _2, _3));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthPoseNode>());
    rclcpp::shutdown();
    return 0;
}