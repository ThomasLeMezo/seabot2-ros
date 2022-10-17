#include "rclcpp/rclcpp.hpp"
#include "pressure_ms5803_driver/msg/pressure_sensor_data.hpp"
#include "pressure_ms5803_driver/pressure_ms5803.h"

using namespace std::chrono_literals;
using namespace std;

class PressureMS5803Node : public rclcpp::Node {
public:
    PressureMS5803Node()
            : Node("pressure_ms5803_node"), pressure_sensor_(this){

        init_parameters();
        init_interfaces();

        pressure_sensor_.init_sensor();
        timer_ = this->create_wall_timer(
                loop_dt_, std::bind(&PressureMS5803Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "[Pressure_ms5803] Start Ok");
    }

private:

    /// Variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pressure_ms5803_driver::msg::PressureSensorData>::SharedPtr publisher_sensor_;

    Pressure_ms5803 pressure_sensor_;

    std::chrono::milliseconds  loop_dt_ = 200ms;

    /// Functions

    /**
     *
     */
    void timer_callback();

    /**
     *
     */
    void init_parameters();

    /**
     *
     */
    void init_interfaces();
};

void PressureMS5803Node::timer_callback() {
    if(pressure_sensor_.measure()){ /// Process the measurement
        pressure_ms5803_driver::msg::PressureSensorData msg;
        msg.temperature = pressure_sensor_.get_temperature();
        msg.pressure = pressure_sensor_.get_pression();
        msg.header.stamp = this->get_clock()->now();

        publisher_sensor_->publish(msg);
    }
}

void PressureMS5803Node::init_parameters() {
    this->declare_parameter<std::string>("i2c_periph", pressure_sensor_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", pressure_sensor_.getI2CAddr());
    this->declare_parameter<long>("loop_dt", loop_dt_.count());

    pressure_sensor_.setI2CPeriph(this->get_parameter_or("i2c_periph", pressure_sensor_.getI2CPeriph()));
    pressure_sensor_.setI2CAddr(this->get_parameter_or("i2c_address", pressure_sensor_.getI2CAddr()));
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void PressureMS5803Node::init_interfaces() {
    publisher_sensor_ = this->create_publisher<pressure_ms5803_driver::msg::PressureSensorData>("sensor_external", 10);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureMS5803Node>());
    rclcpp::shutdown();
    return 0;
}
