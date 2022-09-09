#ifndef SCREEN_H
#define SCREEN_H

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

#define REGISTER_WRITE_SCREEN 0
#define REGISTER_ROBOT_NAME 1
#define REGISTER_IP 2
#define REGISTER_PRESSURE 3
#define REGISTER_TEMPERATURE 4
#define REGISTER_VOLTAGE 5
#define REGISTER_HYGRO 6
#define REGISTER_MISSION_NAME 7
#define REGISTER_WAYPOINT_ID 9
#define REGISTER_NB_WAYPOINT 10
#define REGISTER_TIME 11
#define REGISTER_TIME_REMAINING 12
#define REGISTER_STATUS 13
#define REGISTER_CODE_VERSION 14

#define SCREEN_MISSION_NAME_SIZE 14
#define SCREEN_ROBOT_NAME_SIZE 14

#define DELAY_SLEEP_US 1000

class Screen
{
public:
    /**
     * @brief Screen
     */
    Screen(rclcpp::Node *n){
        n_ = n;
    }

    ~Screen();

    /**
     * @brief Open the I2C device
     * @return
     */
    int i2c_open();

    /**
     * Write the IP to the screen
     * @param data the four value of the IP
     */
    void write_ip(const std::array<unsigned char, 4> &data);

    /**
     * Write the pression
     * @param pressure in mbar [xxxx] mbar
     */
    void write_pressure(const short &pressure);

    /**
     * Write the temperature
     * @param temperature in tenth of degree (205 for 20.5 deg) [xx.x] deg
     */
    void write_temperature(const short &temperature);

    /**
     * Write the hygrometer
     * @param hygro in percentage [xx]%
     */
    void write_hygro(const short &hygro);

    /**
     * Write the voltage of the battery
     * @param volt in tenth of volt (125 for 12.5V) [xx.x] V
     */
    void write_voltage(const char &volt);

    /**
     * Write the name of the robot
     * @param name (max of 16 characters)
     */
    void write_robot_name(const std::string &name);

    /**
     * Write the name of the mission
     * @param mission_name (max of 16 characters)
     */
    void write_mission_name(const std::string &mission_name);

    /**
     * Write the id of the current waypoint
     * @param wp_id [0, 255]
     */
    void write_current_waypoint(const unsigned char &wp_id);

    /**
     * Write the number of waypoint in the mission
     * @param id_max [0, 255]
     */
    void write_number_waypoints(const unsigned char &id_max);

    /**
     * Write the Time
     * @param hour
     * @param minute
     */
    void write_time(const char &hour, const char &minute);

    /**
     * Write the remaining time before next waypoint
     * @param minute
     * @param second
     */
    void write_remaining_time(const char &minute, const char &second);

    /**
     * Status of the robot
     */
    enum Robot_Status { ERROR=0, WARNING=1, OK=3 };

    /**
     * Write the robot status
     * @param status
     */
    void write_robot_status(const Robot_Status &status);

    /**
     * Write the screen with data sent
     */
    void write_screen();

    /**
     *
     * @return
     */
    [[nodiscard]] int getI2CAddr() const;

    void setI2CAddr(int i2CAddr);

    [[nodiscard]] const std::string &getI2CPeriph() const;

    void setI2CPeriph(const std::string &i2CPeriph);

private:
    rclcpp::Node* n_= nullptr; /// Pointer to rclcpp Node

    int file_ = 0; /// File to the i2c port
    int i2c_addr_ = 0x3C;

    std::string i2c_periph_ = "/dev/i2c-0";

};

#endif // SCREEN_H
