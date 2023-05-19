#include "seabot2_thruster_driver/thruster.h"
#include "sys/ioctl.h"

Thruster::~Thruster(){
    write_cmd((uint8_t)MOTOR_PWM_STOP, (uint8_t)MOTOR_PWM_STOP);
    close(file_);
}

int Thruster::i2c_open(){
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

    if(get_version()!=code_version_)
        RCLCPP_WARN(n_->get_logger(), "[Thruster_driver] Wrong PIC code version");

    usleep(100000);
    return 0;
}

int Thruster::write_cmd(const uint8_t &left, const uint8_t &right) const{
    uint8_t data[2] = {right, left};

    int r = i2c_smbus_write_i2c_block_data(file_, 0x00, 2, data);
    if(r < 0)
        RCLCPP_WARN(n_->get_logger(),"[Thruster_driver] I2C Bus Failure - Write cmd");
    return r;
}

uint8_t& Thruster::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    usleep(100);
    return pic_code_version_;
}

void Thruster::write_enable_motors(const bool enable) {
    if(i2c_smbus_write_word_data(file_, ENABLE_MOTOR, enable?0x0101:0x0000)<0)
        RCLCPP_WARN(n_->get_logger(),"[Thruster_driver] I2C Bus Failure - enable motor");
}

int Thruster::getI2CAddr() const {
    return i2c_addr_;
}

void Thruster::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &Thruster::getI2CPeriph() const {
    return i2c_periph_;
}

void Thruster::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
