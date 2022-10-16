#ifndef PRESSURE_H
#define PRESSURE_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include <deque>

#define CMD_RESET 0x1E /// reset command
#define CMD_ADC_READ 0x00 /// ADC read command

#define CMD_ADC_CONV_D1_4096 0x48 /// ADC conversion command
#define CMD_ADC_CONV_D2_4096 0x58 /// ADC conversion command

#define CMD_PROM 0xA1 // Coefficient location

class Pressure_ms5803
{
public:
    /**
     * Constructor of the object
     * @param node Pointer to the rclcpp node for RCLCPP_INFO
     */
    Pressure_ms5803(rclcpp::Node *n){
        n_ = n;
    }

    /**
     *
     */
    ~Pressure_ms5803();

    /**
     * Initialize the sensor
     * @return true if successful
     */
    int init_sensor();

    /**
     * Ask to sensor to measure pressure and temperature
     * @return true if successful
     */
    bool measure();

    /**
     * Ask the sensor to reset
     * @return
     */
    int reset();

    /**
     * Getter to the pressure measured by measure()
     * @return
     */
    float get_pression();

    /**
     * Getter to the temperature measured by measure()
     * @return
     */
    float get_temperature();

    /**
     * Compute the pressure and temperature from the measured values
     * @return
     */
    bool compute();

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

    void setCoefficient(const u_int16_t val, const long unsigned int index){
        if(index<C_.size())
            C_[index] = val;
    }

    /**
     *
     * @param numericalPressure Set D1 value
     */
    void setD1(u_int32_t d1);

    /**
     *
     * @param numericalTemperature Set D2 value
     */
    void setD2(u_int32_t d2);

private:
    int i2c_open();
    bool measure_D1();
    bool measure_D2();

    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port

    int i2c_addr_ = 0x76; /// I2C add of the sensor
    std::string i2c_periph_ = "/dev/i2c-0";

    /// I2C port

    const float p_min_out_range_ = 0.7; /// minimum pression allowed in bar
    const float p_max_out_range_ = 15.0; /// maximum pression allowed in bar
    const float t_min_out_range_ = 0.0; /// minimum temperature allowed in degree
    const float t_max_out_range_ = 80.0; /// maximum temperature allowed degree

    std::array<u_int16_t, 6>  C_; /// Factory calibration coefficients

    u_int32_t D1_, D2_; /// Digital pressure and temperature value

    float pressure_ = 1.0; /// in bar
    float temperature_ = 10.0; // in degree

};

/**
 * Read Digital Pressure
 * @return
 */
inline bool Pressure_ms5803::measure_D1(){
    i2c_smbus_write_byte(file_, CMD_ADC_CONV_D1_4096);
    usleep(10000);
    unsigned char buff[3] = {0, 0, 0};
    if (i2c_smbus_read_i2c_block_data(file_, CMD_ADC_READ, 3, buff) != 3){
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Error Reading D1");
        return false;
    }
    D1_ = (buff[0] << 16) | (buff[1] << 8) | buff[2];
    return true;
}

/**
 * Read Digital Temperature
 * @return
 */
inline bool Pressure_ms5803::measure_D2(){
    i2c_smbus_write_byte(file_, CMD_ADC_CONV_D2_4096);
    usleep(10000);
    unsigned char buff[3] = {0, 0, 0};
    if (i2c_smbus_read_i2c_block_data(file_, CMD_ADC_READ, 3, buff) != 3){
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Error Reading D2");
        return false;
    }
    D2_ = (buff[0] << 16) | (buff[1] << 8) | buff[2];
    return true;
}

/**
 * Getter to the pression measured by the sensor
 * @return pressure in bar
 */
inline float Pressure_ms5803::get_pression(){
    return pressure_;
}

/**
 * Getter to the temperature measured by the sensor
 * @return temperature in degree
 */
inline float Pressure_ms5803::get_temperature(){
    return temperature_;
}

#endif // PRESSURE_H
