#include "seabot2_audio_recorder/tlv320adc.h"
#include "sys/ioctl.h"

TLV320ADC::~TLV320ADC() {
    close(file_);
}

int TLV320ADC::i2c_open() {
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(),
                    strerror(file_));
        exit(1);
    }

    if (const int result = ioctl(file_, I2C_SLAVE, i2c_addr_); result < 0) {
        RCLCPP_WARN(n_->get_logger(),
                    "[TLV320ADC_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE,
                    strerror(result));
        exit(1);
    }

    usleep(100000);
    return 0;
}

int TLV320ADC::configure(const uint8_t gain) const {
    constexpr __useconds_t delay_short = 1000; // 0.1s
    if (set_wake_up() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_reset() != 0) {
        return EXIT_FAILURE;
    }
    usleep(100000); // 0.1s
    if (set_page(0) != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_wake_up() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_i2s_32bit() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_channel_slot() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_channel_enable() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_channel_parameters() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_adc_gain(0, 0) != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    if (set_power_up() != 0) {
        return EXIT_FAILURE;
    }
    usleep(delay_short); // 0.1s
    return EXIT_SUCCESS;
}

int TLV320ADC::set_adc_gain(const uint8_t gain_ch1, const uint8_t gain_ch2) const {
    if (i2c_smbus_write_byte_data(file_, 0x3D, gain_ch1) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set light enable");
        return EXIT_FAILURE;
    }
    if (i2c_smbus_write_byte_data(file_, 0x42, gain_ch2) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set light enable");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


int TLV320ADC::set_wake_up() const {
    if (i2c_smbus_write_byte_data(file_, 0x02, 0x81) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set wake up");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set wake up");
    return EXIT_SUCCESS;
}

int TLV320ADC::set_reset() const {
    if (i2c_smbus_write_byte_data(file_, 0x01, 0x01) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set reset");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set reset");
    return EXIT_SUCCESS;
}

int TLV320ADC::set_page(const uint8_t page) const {
    if (i2c_smbus_write_byte_data(file_, 0x00, page) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set page");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set page to %d", page);
    return EXIT_SUCCESS;
}

int TLV320ADC::set_i2s_32bit() const {
    if (i2c_smbus_write_byte_data(file_, 0x07, 0x70) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set I2S");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set I2S to 32 bit");
    return EXIT_SUCCESS;
}

int TLV320ADC::set_channel_slot() const {
    if (i2c_smbus_write_byte_data(file_, 0x0B, 0x00) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set Channel slot (ch1)");
        return EXIT_FAILURE;
    }
    if (i2c_smbus_write_byte_data(file_, 0x0C, 0x00) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set Channel slot (ch2)");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set channel slot");
    return EXIT_SUCCESS;
}

int TLV320ADC::set_channel_enable() const {
    if (i2c_smbus_write_byte_data(file_, 0x73, 0xC0) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable input");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set enable input");
    if (i2c_smbus_write_byte_data(file_, 0x74, 0xC0) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable output");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int TLV320ADC::set_channel_parameters() const {
    // i2cset -y 0 0x4E 0x3C 0x08 # Set Channel 1 input type (Mic=08,Line=88) + input impedance 20 kOhms
    if (i2c_smbus_write_byte_data(file_, 0x3C, 0x08) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable ch1 input");
        return EXIT_FAILURE;
    }
    if (i2c_smbus_write_byte_data(file_, 0x41, 0x08) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable ch2 input");
        return EXIT_FAILURE;
    }
    // #i2cset -y 0 0x4E 0x3C 0x48 # Set Channel 1 input type (Mic=48,Line=C8) + single-ended output
    if (i2c_smbus_write_byte_data(file_, 0x3C, 0x48) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable ch1 type");
        return EXIT_FAILURE;
    }
    if (i2c_smbus_write_byte_data(file_, 0x41, 0x48) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set enable ch2 type");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int TLV320ADC::set_power_up() const {
    if (i2c_smbus_write_byte_data(file_, 0x75, 0xE0) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[TLV320ADC_driver] I2C bus Failure - Set power up");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[TLV320ADC_driver] Set power up");
    return EXIT_SUCCESS;
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
