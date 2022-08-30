#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>

#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "pressure_bme280_driver/msg/bme280_data.hpp"

extern "C" {
    #include "pressure_bme280_driver/bme280.h"
    #include "pressure_bme280_driver/bme280_defs.h"
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

using namespace std::chrono_literals;
using namespace std;

int file;

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    return i2c_smbus_read_i2c_block_data(file, reg_addr, len, reg_data) != len;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    return i2c_smbus_write_i2c_block_data(file, reg_addr, len, reg_data);
}

void user_delay_ms(uint32_t period) {
    usleep(period * 1000);
}

class Bme280Node : public rclcpp::Node {
public:
    Bme280Node()
            : Node("bme280_node") {

        this->declare_parameter<std::string>("i2c_periph", i2c_periph_);
        this->declare_parameter<bool>("primary_i2c_address", primary_i2c_address_);

        this->get_parameter("i2c_periph", i2c_periph_);
        this->get_parameter("primary_i2c_address", primary_i2c_address_);

        publisher_sensor_ = this->create_publisher<pressure_bme280_driver::msg::Bme280Data>("sensor_internal", 10);
        timer_ = this->create_wall_timer(
                200ms, std::bind(&Bme280Node::timer_callback, this));

        sensor_init();
    }

private:

    /// Variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pressure_bme280_driver::msg::Bme280Data>::SharedPtr publisher_sensor_;

    double pressure_ = 0.0;
    double temperature_ = 0.0;
    double humidity_ = 0.0;

    string i2c_periph_ = "/dev/i2c-0";
    bool primary_i2c_address_ = false;
    struct bme280_dev dev_;

    /// Functions
    void timer_callback();

    void print_sensor_mode();

    void sensor_init();
    void print_calib_settings();
    void print_settings();

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bme280Node>());
    rclcpp::shutdown();
    return 0;
}

void Bme280Node::timer_callback() {
    auto msg = pressure_bme280_driver::msg::Bme280Data();
    struct bme280_data comp_data{};
    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev_);
    if(rslt==0) {
        pressure_ = comp_data.pressure / 100.0;
        humidity_ = comp_data.humidity;
        temperature_ = comp_data.temperature;

        msg.temperature = static_cast<float>(temperature_);
        msg.pressure = static_cast<float>(pressure_);
        msg.humidity = static_cast<float>(humidity_);
        publisher_sensor_->publish(msg);
    }
    else
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Error reading data (%i)", rslt);
}

void Bme280Node::sensor_init() {
// Sensor initialization
    int8_t rslt = BME280_OK;

    if (primary_i2c_address_)
        dev_.dev_id = BME280_I2C_ADDR_PRIM;
    else
        dev_.dev_id = BME280_I2C_ADDR_SEC;

    dev_.intf = BME280_I2C_INTF;

    if ((file = open(i2c_periph_.c_str(), O_RDWR)) < 0) {
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Failed to open the I2C bus (%s)", i2c_periph_.c_str());
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, dev_.dev_id) < 0) {
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }

    dev_.read = user_i2c_read;
    dev_.write = user_i2c_write;
    dev_.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev_); // Get Calib data
    if(rslt!=0)
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Error init the sensor : wrong device add ?");
    print_calib_settings();

/** Recommended mode of operation: Indoor navigation **/
    dev_.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev_.settings.osr_p = BME280_OVERSAMPLING_2X; /// 16X
    dev_.settings.osr_t = BME280_OVERSAMPLING_2X; /// 2X
    dev_.settings.filter = BME280_FILTER_COEFF_2; /// 16
    dev_.settings.standby_time = BME280_STANDBY_TIME_125_MS;

    uint8_t settings_sel;
    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev_);
    if(rslt!=0)
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Error reading the settings");
    print_settings();

    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_);
    if(rslt!=0)
        RCLCPP_WARN(this->get_logger(), "[Pressure_BME280] Error setting the sensor mode");
    print_sensor_mode();

    user_delay_ms(1500);
    if(rslt==0)
        RCLCPP_INFO(this->get_logger(), "[Pressure_BME280] Start Ok");
}

void Bme280Node::print_calib_settings() {
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_T1 = %i", dev_.calib_data.dig_T1);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_T2 = %i", dev_.calib_data.dig_T2);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_T3 = %i", dev_.calib_data.dig_T3);

    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P1 = %i", dev_.calib_data.dig_P1);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P2 = %i", dev_.calib_data.dig_P2);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P3 = %i", dev_.calib_data.dig_P3);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P4 = %i", dev_.calib_data.dig_P4);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P5 = %i", dev_.calib_data.dig_P5);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P6 = %i", dev_.calib_data.dig_P6);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P7 = %i", dev_.calib_data.dig_P7);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P8 = %i", dev_.calib_data.dig_P8);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_P9 = %i", dev_.calib_data.dig_P9);

    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H1 = %i", dev_.calib_data.dig_H1);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H2 = %i", dev_.calib_data.dig_H2);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H3 = %i", dev_.calib_data.dig_H3);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H4 = %i", dev_.calib_data.dig_H4);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H5 = %i", dev_.calib_data.dig_H5);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dig_H6 = %i", dev_.calib_data.dig_H6);
}

void Bme280Node::print_settings() {
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] dev__id = %i", dev_.dev_id);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] chip_id = %i", dev_.chip_id);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] settings.filter = %i", dev_.settings.filter);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] settings.osr_h = %i", dev_.settings.osr_h);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] settings.osr_p = %i", dev_.settings.osr_p);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] settings.osr_t = %i", dev_.settings.osr_t);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] settings.standby_time = %i", dev_.settings.standby_time);
}

void Bme280Node::print_sensor_mode() {
    uint8_t sensor_mode;
    bme280_get_sensor_mode(&sensor_mode, &dev_);
    RCLCPP_DEBUG(this->get_logger(), "[Pressure_BME280] Sensor Mode = %i", sensor_mode);
}
