#ifndef DSPIC_ACOUSTIC_H
#define DSPIC_ACOUSTIC_H

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

class DspicAcoustic
{
public:
    /**
     * @brief DspicAcoustic
     */
    DspicAcoustic(rclcpp::Node *n){
        n_ = n;
    }

    ~DspicAcoustic();

    /**
     * @brief Open the I2C device
     * @return
     */
    int i2c_open();

    /**
     *
     * @return
     */
    int getI2CAddr() const;

    void setI2CAddr(int i2CAddr);

    const std::string &getI2CPeriph() const;

    void setI2CPeriph(const std::string &i2CPeriph);

    int sync_pps();

    int enable_chirp(bool enable=true);

    int set_duration_between_shoot(uint16_t duration_seconds);

    int set_shoot_offset_from_posix_zero(uint16_t offset_seconds);

    int recompute_chirp(const uint16_t &frequency_middle, const uint16_t &frequency_range);

    int set_robot_code(const uint8_t &robot_code);

    /**
     *
     * @return
     */
    uint8_t get_pps_value() const;

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    std::string i2c_periph_ = "/dev/i2c-0";
    int i2c_addr_ = 0x1A;
    int code_version = 0x05;


};

#endif // DSPIC_ACOUSTIC_H
