#ifndef BUILD_BME280_NODE_HPP
#define BUILD_BME280_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"
#include <chrono>
#include <functional>
#include <cstring>

extern "C" {
#include "pressure_bme280_driver/bme280.h"
#include "pressure_bme280_driver/bme280_defs.h"
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

using namespace std::chrono_literals;
using namespace std;

int file;

class Bme280Node : public rclcpp::Node {
public:
    Bme280Node()
            : Node("bme280_node") {

        init_parameters();
        init_interfaces();

        sensor_init();

        timer_ = this->create_wall_timer(
                loop_dt_, std::bind(&Bme280Node::timer_callback, this));
    }

private:

    /// Variables
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds  loop_dt_ = 200ms; // loop dt

    rclcpp::Publisher<pressure_bme280_driver::msg::Bme280Data>::SharedPtr publisher_sensor_;

    double pressure_ = 0.0;
    double temperature_ = 0.0;
    double humidity_ = 0.0;

    string i2c_periph_ = "/dev/i2c-0";
    bool primary_i2c_address_ = false;
    struct bme280_dev dev_;

    /// Functions

    /**
     *
     */
    void timer_callback();

    /**
     *
     */
    void print_sensor_mode();

    /**
     *
     */
    void sensor_init();

    /**
     *
     */
    void print_calib_settings();

    /**
     *
     */
    void print_settings();

    /**
     *
     */
    void init_parameters();

    /**
     *
     */
    void init_interfaces();

};

#endif //BUILD_BME280_NODE_HPP
