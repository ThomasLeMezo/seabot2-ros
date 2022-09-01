#include "seabot2_light_driver/light.h"
#include "sys/ioctl.h"

Light::~Light(){
    close(file_);
}

int Light::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Light_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[Light_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }
    usleep(100000);
    return 0;
}

int Light::set_light_enable(const bool &enable) const{
    if(i2c_smbus_write_byte_data(file_, REGISTER_LIGHT_ENABLE, enable)<0) {
        RCLCPP_WARN(n_->get_logger(), "[Light_driver] I2C bus Failure - Set light enable");
        return EXIT_FAILURE;
    }
    else
        return EXIT_SUCCESS;
}

void Light::set_power(const __u8 &val) const {
    if(val<199) {
        if (i2c_smbus_write_byte_data(file_, REGISTER_LIGHT_POWER, val) < 0)
            RCLCPP_WARN(n_->get_logger(), "[Light_driver] I2C bus Failure - Set light power");
    }
}

void Light::set_pattern(const std::array<__u8, NB_PATTERN> &pattern) const {
    if (i2c_smbus_write_i2c_block_data(file_, REGISTER_LIGHT_PATTERN, pattern.size(), pattern.data()) < 0)
        RCLCPP_WARN(n_->get_logger(), "[Light_driver] I2C bus Failure - Set light pattern");
}

void Light::set_flash_number(const unsigned int &nb_flash) const {
    std::array<__u8, NB_PATTERN> pattern{};
    for(unsigned int i=0; i<std::min(nb_flash*2, static_cast<unsigned int>(NB_PATTERN)); i+=2){
        pattern[i] = flash_duration_;
        if(i/2==nb_flash)
            pattern[i+1] = flash_pause_end_; /// in 0.1s ?
        else
            pattern[i+1] = flash_pause_between_flash_;
    }
    set_pattern(pattern);
}

uint8_t& Light::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    return pic_code_version_;
}

int Light::getI2CAddr() const {
    return i2c_addr_;
}

void Light::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &Light::getI2CPeriph() const {
    return i2c_periph_;
}

void Light::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
