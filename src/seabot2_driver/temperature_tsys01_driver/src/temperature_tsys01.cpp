#include "temperature_tsys01_driver/temperature_tsys01.hpp"
#include "sys/ioctl.h"

using namespace std;

Temperature_TSYS01::~Temperature_TSYS01(){
    if(file_ !=0)
        close(file_);
}

int Temperature_TSYS01::reset(){
    int res = i2c_smbus_write_byte(file_, CMD_RESET);
    usleep(30000); // 28ms reload for the sensor (?)
    //  usleep(30000);
    if (res < 0)
        RCLCPP_WARN(n_->get_logger(), "[Temperature_TSYS01] Error reseting sensor");
    else
        RCLCPP_INFO(n_->get_logger(), "[Temperature_TSYS01] Reset ok");
    return 0;
}

int Temperature_TSYS01::i2c_open(){
    if ((file_ = open(i2c_periph_.c_str(),O_RDWR)) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Temperature_TSYS01] Failed to open the I2C bus (%s)", i2c_periph_.c_str());
        exit(1);
    }

    if (ioctl(file_,I2C_SLAVE,i2c_addr_) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Temperature_TSYS01] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

int Temperature_TSYS01::init_sensor(){
    i2c_open();

    RCLCPP_INFO(n_->get_logger(), "[Temperature_TSYS01] Sensor initialization");
    reset();
    int return_val = 0;

    unsigned char buff[2] = {0, 0};
    for(int i=0; i<5; i++){
        __u8 add = CMD_PROM + (char) 2*(i+1); // Start at 0xA2
        if (i2c_smbus_read_i2c_block_data(file_, add, 2, buff)!=2){
            RCLCPP_WARN(n_->get_logger(), "[Temperature_TSYS01] Error Reading 0x%X", add);
            return_val = 1;
        }
        k_[4-i] = (buff[0] << 8) | buff[1] << 0;
    }
    if(return_val==0)
        RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] Sensor Read PROM OK");

    RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] k0 = %d", k_[0]);
    RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] k1 = %d", k_[1]);
    RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] k2 = %d", k_[2]);
    RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] k3 = %d", k_[3]);
    RCLCPP_DEBUG(n_->get_logger(), "[Temperature_TSYS01] k4 = %d", k_[4]);

    return return_val;
}

bool Temperature_TSYS01::measure(){
    i2c_smbus_write_byte(file_, CMD_ADC_CONV);
    usleep(60000);
    unsigned char buff[3] = {0, 0, 0};
    if (i2c_smbus_read_i2c_block_data(file_, CMD_ADC_READ, 3, buff)!=3){
        RCLCPP_WARN(n_->get_logger(), "[Temperature_TSYS01] Error Reading T at device \\\\x%02x", i2c_addr_);
        valid_data_ = false;
        return false;
    }

    double adc24 = (buff[0] << 16) | (buff[1] << 8) | buff[2];
    if(adc24==0)
        valid_data_ = false;
    else
        valid_data_ = true;

    double adc16 = adc24/256.0;
    temperature_ = -2.0*k_[4]*1e-21*pow(adc16, 4)
                    +4.0*k_[3]*1e-16*pow(adc16, 3)
                    -2.0*k_[2]*1e-11*pow(adc16, 2)
                    +1.0*k_[1]*1e-6*adc16
                    -1.5*k_[0]*1e-2;

    if(temperature_ < 0.0 || temperature_ >100.0)
        valid_data_ = false;

    return valid_data_;
}

int Temperature_TSYS01::getI2CAddr() const {
    return i2c_addr_;
}

void Temperature_TSYS01::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const string &Temperature_TSYS01::getI2CPeriph() const {
    return i2c_periph_;
}

void Temperature_TSYS01::setI2CPeriph(const string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
