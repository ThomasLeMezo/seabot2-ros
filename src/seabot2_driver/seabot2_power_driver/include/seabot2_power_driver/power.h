#ifndef POWER_H
#define POWER_H

#include <rclcpp/rclcpp.hpp>

#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#define REGISTER_DATA_READ 0x00
#define REGISTER_DATA_SIZE 11
#define CONVERT_BRIDGE_BATTERY (3.3/1024.)
#define CONVERT_BRIDGE_CURRENT (3.3/1024.)
#define CONVERT_CURRENT_V_to_A (1./66000.)

class Power
{
public:
    /**
     * @brief Power
     */
    Power(rclcpp::Node *n){
        n_ = n;
    }

    ~Power();

    /**
     * @brief Open the I2C device
     * @return
     */
    int i2c_open();

    /**
     * get the version of the pic software
     * @return version
     */
    uint8_t& get_version();

    /**
     *
     * @return
     */
    int getI2CAddr() const;

    void setI2CAddr(int i2CAddr);

    const std::string &getI2CPeriph() const;

    void setI2CPeriph(const std::string &i2CPeriph);

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    std::string i2c_periph_ = "/dev/i2c-1";
    int i2c_addr_ = 0x39;
    const int code_version_ = 0x06; /// Code version of the expected hardware
    uint8_t pic_code_version_=0; /// Code version read from the hardware

    std::array<double, 2> R1_= {180, 180};
    std::array<double, 2> R2_= {820, 820};

public:
    /// Variables
    std::array<float, 4> cell_volt_{};
    double battery_volt_;
    std::array<float, 2> esc_current_{};
    float motor_current_ = 0.0;
    int power_state_=0;

public:

    /**
     * Get all the data from the pic
     * @return
     */
    int get_all_data();

    /**
     * Set the seabot2 to sleep
     * (default delay of 60s before going to sleep)
     * @return
     */
    int set_sleep();

};

#endif // POWER_H
