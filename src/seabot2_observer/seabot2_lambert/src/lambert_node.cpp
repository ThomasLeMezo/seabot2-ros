#include "seabot2_lambert/lambert_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

LambertNode::LambertNode()
        : Node("lambert_node"){

    init_parameters();
    init_interfaces();

    C_ = proj_context_create();
    P_ = proj_create_crs_to_crs (C_, epsg_source_.c_str(), epsg_target_.c_str(), NULL);
    if (0 == P_) {
        RCLCPP_WARN(this->get_logger(), "[Lambert_node] Error Proj %s\n", proj_errno_string(proj_errno(P_)));
        exit(EXIT_FAILURE);
    }
    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    P_for_GIS_ = proj_normalize_for_visualization(C_, P_);
    if (0 == P_for_GIS_) {
        RCLCPP_WARN(this->get_logger(), "[Lambert_node] Failed to normalize transformation object %s\n",
                    proj_errno_string(proj_errno(P_)));
        proj_destroy(P_);
        exit(EXIT_FAILURE);
    }
    proj_destroy(P_);
    P_ = P_for_GIS_;

    RCLCPP_INFO(this->get_logger(), "[Lambert_node] Start Ok");
}

void LambertNode::init_parameters() {
    this->declare_parameter<string>("epsg_source", epsg_source_);
    this->declare_parameter<string>("epsg_target", epsg_target_);
    this->declare_parameter<int>("filter_position_mean", filter_position_mean_.count());
    this->declare_parameter<int>("filter_dt_heading_computation_", filter_dt_heading_computation_.count());

    epsg_source_ = this->get_parameter_or("epsg_source", epsg_source_);
    epsg_target_ = this->get_parameter_or("epsg_target", epsg_target_);
    filter_position_mean_ = std::chrono::seconds(this->get_parameter_or("filter_position_mean", filter_position_mean_.count()));
    filter_dt_heading_computation_ = std::chrono::seconds(this->get_parameter_or("filter_dt_heading_computation_", filter_dt_heading_computation_.count()));
}

void LambertNode::compute_mean(){
    seabot2_lambert::msg::GnssPose msg_data;
    double east_start_mean=0., north_start_mean=0.;
    double east_end_mean=0., north_end_mean=0.;
    size_t start_pose_max_idx = lower_bound(time_memory_.begin(), time_memory_.end(),
                                            time_memory_.front()+filter_position_mean_) - time_memory_.begin();
    size_t end_pose_min_idx = lower_bound(time_memory_.begin(), time_memory_.end(),
                                            time_memory_.back()-filter_position_mean_) - time_memory_.begin();

    if(start_pose_max_idx!=time_memory_.size() && end_pose_min_idx!=0) {
        east_start_mean = std::accumulate(east_memory_.begin(), east_memory_.begin()+start_pose_max_idx, 0.);
        east_start_mean/=static_cast<double>(start_pose_max_idx);

        north_start_mean = std::accumulate(north_memory_.begin(), north_memory_.begin()+start_pose_max_idx, 0.);
        north_start_mean/=static_cast<double>(start_pose_max_idx);

        east_end_mean = std::accumulate(east_memory_.end()-end_pose_min_idx, east_memory_.end(), 0.);
        east_end_mean/=static_cast<double>(end_pose_min_idx);

        north_end_mean = std::accumulate(north_memory_.end()-end_pose_min_idx, north_memory_.end(), 0.);
        north_end_mean/=static_cast<double>(end_pose_min_idx);

        msg_data.east = east_end_mean;
        msg_data.north = north_end_mean;

        double heading_rad = atan2(east_end_mean-east_start_mean, north_end_mean-north_start_mean);
        msg_data.heading = (heading_rad > 0 ? heading_rad : (2*M_PI + heading_rad)) * 360. / (2.*M_PI);
    }
    else{
        msg_data.north = north_memory_.back();
        msg_data.east = east_memory_.back();
        msg_data.heading = 0.;
    }
    publisher_lambert_mean_data_->publish(msg_data);
}

void LambertNode::gnss_callback(const gpsd_client::msg::GpsFix &msg) {
    if(msg.longitude != 0. && msg.latitude != 0. && msg.status>=msg.MODE_2D){

        PJ_COORD coord_source, coord_target;
        coord_source.lpzt.z = 0.0;
        coord_source.lpzt.t = HUGE_VAL;

        coord_source.lpzt.lam = msg.longitude;
        coord_source.lpzt.phi = msg.latitude;
        coord_target = proj_trans(P_, PJ_FWD, coord_source);

        seabot2_lambert::msg::GnssPose msg_data;
        msg_data.east = coord_target.xyz.x;
        msg_data.north = coord_target.xyz.y;
        msg_data.heading = msg.track;
        msg_data.header.stamp = msg.header.stamp;

        publisher_lambert_data_->publish(msg_data);

        /// Compute mean
        east_memory_.push_back(coord_target.xyz.x);
        north_memory_.push_back(coord_target.xyz.y);
        time_memory_.emplace_back(msg.header.stamp);

        if(time_memory_.back()-time_memory_.front() > filter_dt_heading_computation_){
            east_memory_.pop_front();
            north_memory_.pop_front();
            time_memory_.pop_front();
        }
        compute_mean();
    }
    else{
        east_memory_.clear();
        north_memory_.clear();
        time_memory_.clear();
    }
}

void LambertNode::init_interfaces() {
    publisher_lambert_data_ = this->create_publisher<seabot2_lambert::msg::GnssPose>("pose", 1);
    publisher_lambert_mean_data_ = this->create_publisher<seabot2_lambert::msg::GnssPose>("pose_mean", 1);

    subscriber_gnss_data_ = this->create_subscription<gpsd_client::msg::GpsFix>(
            "/driver/fix", 10, std::bind(&LambertNode::gnss_callback, this, _1));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LambertNode>());
    rclcpp::shutdown();
    return 0;
}