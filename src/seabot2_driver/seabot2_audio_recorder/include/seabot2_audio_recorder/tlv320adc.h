#ifndef TLV320ADC_H
#define TLV320ADC_H

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

class TLV320ADC
{
public:
    /**
     * @brief TLV320ADC
     */
    TLV320ADC(rclcpp::Node *n){
        n_ = n;
    }

    ~TLV320ADC();

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

    /*
     * gain values: 0dB=0x00, 10dB=0x28, 20dB=0x50, 30dB=78, 40dB=A0
     */
    enum class AdcGain {
        GAIN_0dB = 0,
        GAIN_10dB = 0x28,
        GAIN_20dB = 0x50,
        GAIN_30dB = 0x78,
        GAIN_40dB = 0xA0,
    };

    /**
     *
     * @return
     */
    int set_adc_gain(uint8_t gain_ch1, uint8_t gain_ch2);

    /**
     * @brief Set the signal id
     * @param signal_id
     * @return
     */
    int set_signal_id(uint8_t signal_id);

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    std::string i2c_periph_ = "/dev/i2c-0";
    int i2c_addr_ = 0x4E;

public:

};

#endif // TLV320ADC_H
