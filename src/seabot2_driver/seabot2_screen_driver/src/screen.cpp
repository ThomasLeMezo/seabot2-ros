#include "seabot2_screen_driver/screen.h"
#include "sys/ioctl.h"
#include <iterator>
using namespace std;

Screen::~Screen(){
    close(file_);
}

int Screen::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Piston_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[Piston_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }
    usleep(100000);
    return 0;
}

int Screen::getI2CAddr() const {
    return i2c_addr_;
}

void Screen::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &Screen::getI2CPeriph() const {
    return i2c_periph_;
}

void Screen::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}

void Screen::write_ip(const std::array<unsigned char, 4> &data) {
    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_IP, 4, data.data())<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write IP");
}

void Screen::write_pressure(const short &pressure) {
    if(i2c_smbus_write_word_data(file_, REGISTER_PRESSURE, pressure)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write pressure");
}

void Screen::write_temperature(const short &temperature) {
    if(i2c_smbus_write_word_data(file_, REGISTER_TEMPERATURE, temperature)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write temperature");
}

void Screen::write_hygro(const short &hygro) {
    if(i2c_smbus_write_word_data(file_, REGISTER_HYGRO, hygro)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write hygro");
}

void Screen::write_voltage(const char &volt) {
    if(i2c_smbus_write_byte_data(file_, REGISTER_VOLTAGE, volt)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write voltage");
}

void Screen::write_robot_name(const std::string &name) {
    unsigned char name_c[SCREEN_ROBOT_NAME_SIZE];
    fill(begin(name_c), end(name_c), ' ');
    for(int i=0; i<min((int)name.length(), SCREEN_MISSION_NAME_SIZE); i++)
        name_c[i] = name[i];
    name_c[15] = '\n';

    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_ROBOT_NAME, SCREEN_ROBOT_NAME_SIZE, name_c)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write name");
}

void Screen::write_mission_name(const std::string &mission_name) {
    unsigned char mission_name_c[SCREEN_MISSION_NAME_SIZE];
    fill(begin(mission_name_c), end(mission_name_c), ' ');
    for(int i=0; i<min((int)mission_name.length(), SCREEN_MISSION_NAME_SIZE); i++)
        mission_name_c[i] = mission_name[i];
    mission_name_c[15] = '\n';

    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_MISSION_NAME, SCREEN_MISSION_NAME_SIZE,
                                      mission_name_c)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write name");
}

void Screen::write_current_waypoint(const unsigned char &wp_id) {
    if(i2c_smbus_write_byte_data(file_, REGISTER_WAYPOINT_ID, wp_id)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write waypoint id");
}

void Screen::write_number_waypoints(const unsigned char &id_max) {
    if(i2c_smbus_write_byte_data(file_, REGISTER_NB_WAYPOINT, id_max)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write waypoint max id");
}

void Screen::write_time(const char &hour, const char &minute) {
    uint16_t data = ((uint16_t)hour<<8) + ((uint16_t)minute); /// ToDo : check lsb or msb ?
    if(i2c_smbus_write_word_data(file_, REGISTER_TIME, data)<0)
    RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write time");
}

void Screen::write_remaining_time(const char &minute, const char &second) {
    uint16_t data = ((uint16_t)second<<8) + ((uint16_t)minute); /// ToDo : check lsb or msb ?
    if(i2c_smbus_write_word_data(file_, REGISTER_TIME_REMAINING, data)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write remaining time");
}

void Screen::write_robot_status(const Screen::Robot_Status &status) {
    if(i2c_smbus_write_byte_data(file_, REGISTER_STATUS, status)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write status");
}

void Screen::write_screen() {
    if(i2c_smbus_write_byte_data(file_, REGISTER_WRITE_SCREEN, 0x00) < 0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write reset screen");
}
