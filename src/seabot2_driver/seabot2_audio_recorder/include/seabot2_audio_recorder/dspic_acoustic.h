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

    int set_pps_sync_chirp_id(uint8_t chirp_id);

    int enable_chirp();

    /**
     *
     * @return
     */
    uint8_t get_pps_value();

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    std::string i2c_periph_ = "/dev/i2c-0";
    int i2c_addr_ = 0x1A;

public:

};

#endif // DSPIC_ACOUSTIC_H
