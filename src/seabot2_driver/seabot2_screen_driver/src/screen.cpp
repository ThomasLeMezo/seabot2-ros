#include "seabot2_screen_driver/screen.h"
#include "sys/ioctl.h"

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
    size_t size_name = std::min(name.length(), static_cast<size_t>(16));
    unsigned char name_c[16];
    std::copy( name.begin(), name.end(), name_c );
    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_ROBOT_NAME, size_name, name_c)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write name");
}


void Screen::write_mission_name(const std::string &mission_name) {
    size_t size_mission_name = std::min(mission_name.length(), static_cast<size_t>(16));
    unsigned char mission_name_c[16];
    std::copy( mission_name.begin(), mission_name.end(), mission_name_c );
    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_MISSION_NAME, size_mission_name,
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
    uint16_t data = (uint16_t)hour + (((uint16_t)minute)<<8); /// ToDo : check lsb or msb ?
    if(i2c_smbus_write_word_data(file_, REGISTER_TIME, data)<0)
    RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write time");
}

void Screen::write_remaining_time(const char &minute, const char &second) {
    uint16_t data = minute + (second<<8); /// ToDo : check lsb or msb ?
    if(i2c_smbus_write_word_data(file_, REGISTER_TIME_REMAINING, data)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write remaining time");
}

void Screen::write_robot_status(const Screen::Robot_Status &status) {
    if(i2c_smbus_write_byte_data(file_, REGISTER_STATUS, status)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write status");
}

void Screen::write_reset_screen() {
    if(i2c_smbus_write_byte_data(file_, REGISTER_RESET_SCREEN, 0x00)<0)
        RCLCPP_WARN(n_->get_logger(),"[Screen_driver] I2C Bus Failure - Write reset screen");
}
