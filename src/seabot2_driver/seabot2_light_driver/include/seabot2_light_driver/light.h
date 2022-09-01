#ifndef LIGHT_H
#define LIGHT_H

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

#define REGISTER_LIGHT_ENABLE 0x00
#define REGISTER_LIGHT_POWER 0x01
#define REGISTER_LIGHT_PATTERN 0x02
#define NB_PATTERN 10

class Light
{
public:
    /**
     * @brief Light
     */
    Light(rclcpp::Node *n){
        n_ = n;
        i2c_open();
        if(get_version()!=code_version_)
            RCLCPP_WARN(n->get_logger(), "[Light_driver] Wrong PIC code version");
    }

    ~Light();

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
    std::string i2c_periph_ = "/dev/i2c-0";
    int i2c_addr_ = 0x28;
    const int code_version_ = 0x01; /// Code version of the expected hardware
    uint8_t pic_code_version_=0; /// Code version read from the hardware

public:
    int flash_duration_ = 1;
    int flash_pause_end_ = 40;
    int flash_pause_between_flash_ = 5;

public:

    /**
     * Enable the light of the seabot2
     * @param enable
     */
    int set_light_enable(const bool &enable) const;

    /**
     * Set Power of the led
     * @param val in 0 to 199
     */
    void set_power(const __u8 &val) const;

    /**
     * Set the pattern of the light
     * @param pattern
     */
    void set_pattern(const std::array<__u8, NB_PATTERN> &pattern) const;

    /**
     * Set a pattern with nb_flash
     * @param nb_flash
     */
    void set_flash_number(const unsigned int &nb_flash) const;
};

#endif // LIGHT_H
