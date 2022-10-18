#include "rclcpp/rclcpp.hpp"
#include "temperature_tsys01_driver/msg/temperature_sensor_data.hpp"
#include "temperature_tsys01_driver/temperature_tsys01.hpp"

using namespace std::chrono_literals;
using namespace std;

class TemperatureTSYS01Node : public rclcpp::Node {
public:
    TemperatureTSYS01Node()
            : Node("temperature_tsys01_node"), temperature_sensor_(this){

        init_parameters();
        init_interfaces();

        temperature_sensor_.init_sensor();
        timer_ = this->create_wall_timer(
                loop_dt_, std::bind(&TemperatureTSYS01Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "[Temperature_Tsys01] Start Ok");
    }

private:

    /// Variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<temperature_tsys01_driver::msg::TemperatureSensorData>::SharedPtr publisher_sensor_;

    Temperature_TSYS01 temperature_sensor_;

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

void TemperatureTSYS01Node::timer_callback() {
    if(temperature_sensor_.measure()){ /// Process the measurement
        temperature_tsys01_driver::msg::TemperatureSensorData msg;
        msg.temperature = temperature_sensor_.get_temperature();
        msg.header.stamp = this->get_clock()->now();

        publisher_sensor_->publish(msg);
    }
}

void TemperatureTSYS01Node::init_parameters() {
    this->declare_parameter<std::string>("i2c_periph", temperature_sensor_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", temperature_sensor_.getI2CAddr());
    this->declare_parameter<long>("loop_dt", loop_dt_.count());

    temperature_sensor_.setI2CPeriph(this->get_parameter_or("i2c_periph", temperature_sensor_.getI2CPeriph()));
    temperature_sensor_.setI2CAddr(this->get_parameter_or("i2c_address", temperature_sensor_.getI2CAddr()));
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));
}

void TemperatureTSYS01Node::init_interfaces() {
    publisher_sensor_ = this->create_publisher<temperature_tsys01_driver::msg::TemperatureSensorData>("temperature", 10);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureTSYS01Node>());
    rclcpp::shutdown();
    return 0;
}
