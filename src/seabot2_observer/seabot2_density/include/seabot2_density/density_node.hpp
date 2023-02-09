#ifndef BUILD_DENSITY_NODE_HPP
#define BUILD_DENSITY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "seabot2_density/msg/density.hpp"
#include "seabot2_depth_filter/msg/depth_pose.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "TeosSea.h"

using namespace std::chrono_literals;
using namespace std;

class DensityNode : public rclcpp::Node {
public:
    DensityNode();

private:
    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

    /// Variable
    double sea_pressure_ = 0.;
    double temperature_ = 12.0;
    double salinity_ = 0.;
    double water_density_ = 1000.0;
    double water_sound_speed_ = 1500.0;

    TeosSea ts;

    /// Interfaces
    rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
    rclcpp::Subscription<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_data_;

    rclcpp::Publisher<seabot2_density::msg::Density>::SharedPtr publisher_density_;

    /// Functions

    /**
     * Callback
     */
    void timer_callback();

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
    void temperature_callback(const temperature_tsys01_driver::msg::TemperatureSensorData &msg);

    /**
     * Callback of depth
     * @param msg
     */
    void pressure_callback(const seabot2_depth_filter::msg::DepthPose &msg);

private:

};
#endif //BUILD_DENSITY_NODE_HPP
