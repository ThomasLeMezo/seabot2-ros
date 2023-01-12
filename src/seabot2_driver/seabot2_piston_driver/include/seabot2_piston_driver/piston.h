#ifndef PISTON_H
#define PISTON_H

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

#define REGISTER_STATE 0x05
#define REGISTER_REGULATION_DEAD_ZONE 0x30
#define REGISTER_REGULATION_PROPORTIONAL 0x32
#define REGISTER_DATA_READ 0x00
#define REGISTER_DATA_SIZE 18
#define REGISTER_SET_POINT 0x00

#define CONVERSION_BRIDGE (3.3/4096.)
#define CONVERSION_CURRENT CONVERSION_BRIDGE /*(CONVERSION_BRIDGE * 1000.0/264.0)*/

class Piston
{
public:
    /**
     * @brief Piston
     */
    Piston(rclcpp::Node *n){
        n_ = n;
    }

    ~Piston();

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

    enum state_piston { PISTON_SEARCH_SWITCH_BOTTOM=0,
                        PISTON_RELEASE_SWITCH_BOTTOM=1,
                        PISTON_BACK_SWITCH_BOTTOM=2,
                        PISTON_REGULATION=3,
                        PISTON_EXIT=4,
                        PISTON_LOW_BATT=5
    };

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    std::string i2c_periph_ = "/dev/i2c-1";
    int i2c_addr_ = 0x1E;
    const int code_version_ = 0x05; /// Code version of the expected hardware
    uint8_t pic_code_version_=0; /// Code version read from the hardware

public:
    /// Bridge
    double R1_ = 3.9; // kOhms
    double R2_ = 18.0; // kOhms

public:
    int position_ = 0;
    int position_last_ = 0;
    bool switch_top_ = false;
    bool switch_bottom_ = false;
    int state_ = PISTON_SEARCH_SWITCH_BOTTOM;
    bool enable_ = false;
    int position_set_point_ = 0;
    float battery_voltage_ = 0;
    float motor_current_ = 0;
    int motor_set_point_ = 0;
    int motor_cmd_ = 0;
    bool motor_sens_ = false;

public:
    /**
     * Reset the piston to position 0
     */
    int set_piston_reset() const;

    /**
     * Set piston exit mode
     * @return
     */
    int set_piston_exit() const;

    /**
     *
     * @return
     */
    int set_piston_regulation() const;

    /**
     * Set the regulation dead zone
     * @param val interval of ticks
     */
    void set_regulation_dead_zone(const __u16 &val) const;

    /**
     * Set the coefficient Kp of the regulator
     * @param val
     */
    void set_regulation_proportional(const __u16 &val) const;

    /**
     * Set the position of the piston
     * @param val
     * @return
     */
    int set_position(const int32_t &val) const;

    /**
     * Get data from dspic
     * @return
     */
    int get_all_data();

};

#endif // PISTON_H
