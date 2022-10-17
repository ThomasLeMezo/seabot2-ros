#ifndef THRUSTER_H
#define THRUSTER_H

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

#define MOTOR_PWM_STOP 150
#define MAX_PWM 190
#define MIN_PWM 110

#define ENABLE_MOTOR 0x10

class Thruster
{
public:
    /**
     * @brief Thruster
     */
    Thruster(rclcpp::Node *n){
        n_ = n;
        i2c_open();

    }

    ~Thruster();

    /**
     * @brief Open the I2C device
     * @return
     */
    int i2c_open();

    /**
     * @brief Send the pwm value to the thrusters \n
     * Max values \n
     *  - Stopped         151 \n
     *  - Max forward     191 \n
     *  - Max reverse     111 \n
     * @param left
     * @param right
     * @return 0 if success
     */
    int write_cmd(const uint8_t &left, const uint8_t &right) const;

    /**
     * Enable/Disable the left & right motor
     * @param enable
     */
    void write_enable_motors(bool enable);

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
    int i2c_addr_ = 0x20;
    const int code_version_ = 0x01; /// Code version of the expected hardware
    uint8_t pic_code_version_=0; /// Code version read from the hardware

private:
    std::string i2c_periph_ = "/dev/i2c-1";

    bool reverse_thruster_order_ = false;


};

#endif // THRUSTER_H
