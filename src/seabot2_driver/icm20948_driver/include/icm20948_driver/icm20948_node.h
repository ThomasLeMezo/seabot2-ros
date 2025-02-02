//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_ICM20948_NODE_H
#define BUILD_ICM20948_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "icm20948_driver/icm20948.h"
#include "Fusion.h"

#include "seabot2_msgs/msg/raw_data.hpp"
#include "seabot2_msgs/msg/rpy.hpp"
#include "seabot2_msgs/msg/debug_fusion.hpp"


using namespace std::chrono_literals;
using namespace std;

class ICM20948Node : public rclcpp::Node {
public:
    ICM20948Node();

private:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 20ms; /// loop dt

    ICM20948 icm20948_;

    /// Topics
    rclcpp::Publisher<seabot2_msgs::msg::RawData>::SharedPtr publisher_raw_data_;
    rclcpp::Publisher<seabot2_msgs::msg::RawData>::SharedPtr publisher_calibrated_data_;
    rclcpp::Publisher<seabot2_msgs::msg::RPY>::SharedPtr publisher_rpy_;
    rclcpp::Publisher<seabot2_msgs::msg::DebugFusion>::SharedPtr publisher_debug_fusion_;

    bool publish_raw_data_ = true;
    bool publish_calibrated_data_ = false;
    bool publish_rpy_ = true;
    bool publish_debug_fusion_ = false;

    /// Parameters

    // Fusion
    // Rotation of 90° over z-axis
    FusionMatrix gyroscopeMisalignment_ = {0.0f, 1.0f, 0.0f,
                                           -1.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f};
    FusionVector gyroscopeSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector gyroscopeOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix accelerometerMisalignment_ = gyroscopeMisalignment_;
    FusionVector accelerometerSensitivity_ = {1.0f, 1.0f, 1.0f};
    FusionVector accelerometerOffset_ = {0.0f, 0.0f, 0.0f};
    FusionMatrix softIronMatrix_ = {1.0909389005666026, 0.00012587571038681422, -0.005999255022037946,
                                    0.00012587571038681278, 1.04726279047246, -0.013739953259287032,
                                    -0.005999255022037951, -0.013739953259287034, 0.8754869909015943 };

    FusionVector hardIronOffset_ = {-25.90120562612343, 81.872262863885, -73.24716722906496};
    FusionOffset offset_{};
    FusionAhrs ahrs_{};
    FusionConvention convention_ = FusionConventionNed;
    float fusion_gain_ = 0.5f;
    unsigned int sample_rate_ = 25;
    rclcpp::Time last_time_fusion_ = rclcpp::Time(0., RCL_STEADY_TIME);
    rclcpp::Clock steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

    void compute_ahrs();

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    /**
     * Timer callback
     */
    void timer_callback();

};

#endif //BUILD_ICM20948_NODE_H
