#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"

#define CMD_RESET 0x1E // reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x48 // ADC conversion command
#define CMD_PROM 0xA0 // Coefficient location

#define CONVERSION_TIME 10000 // 10ms

class Temperature_TSYS01
{
public:
    /**
     * Constructor of the object
     * @param n
     */
    Temperature_TSYS01(rclcpp::Node *n){
        n_ = n;
    }

    ~Temperature_TSYS01();

    int i2c_open();
    int init_sensor();
    int reset();

    bool measure();

    double get_temperature();

    /**
 *
 * @return
 */
    int getI2CAddr() const;

    /**
     *
     * @param i2CAddr
     */
    void setI2CAddr(int i2CAddr);

    /**
     *
     * @return
     */
    const std::string &getI2CPeriph() const;

    /**
     *
     * @param i2CPeriph
     */
    void setI2CPeriph(const std::string &i2CPeriph);

private:

    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0;
    int i2c_addr_ = 0x77;
    std::string i2c_periph_ = "/dev/i2c-1";

    u_int16_t k_[5];
    bool valid_data_ = false;

    double temperature_;

};

inline double Temperature_TSYS01::get_temperature(){
    return temperature_;
}

#endif // TEMPERATURE_H
