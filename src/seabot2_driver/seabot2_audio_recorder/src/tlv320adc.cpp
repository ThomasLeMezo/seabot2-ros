#include "seabot2_audio_recorder/tlv320adc.h"
#include "sys/ioctl.h"

TLV320ADC::~TLV320ADC(){
    close(file_);
}

int TLV320ADC::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[TLV320ADC_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }

    usleep(100000);
    return 0;
}

int TLV320ADC::set_adc_gain(uint8_t gain_ch1, uint8_t gain_ch2) {
    if(i2c_smbus_write_byte_data(file_, 0x3D, (gain_ch1<<1))<0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set light enable");
    }
    if(i2c_smbus_write_byte_data(file_, 0x42, (gain_ch2<<1))<0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set light enable");
    }

    return EXIT_FAILURE;
}

int TLV320ADC::set_signal_id(uint8_t signal_id) {
    if(i2c_smbus_write_byte_data(file_, 0x0D, signal_id)<0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set signal ID");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set signal ID to %d", signal_id);
        return EXIT_SUCCESS;
    }
}


int TLV320ADC::getI2CAddr() const {
    return i2c_addr_;
}

void TLV320ADC::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &TLV320ADC::getI2CPeriph() const {
    return i2c_periph_;
}

void TLV320ADC::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
