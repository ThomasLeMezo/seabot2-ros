#include "seabot2_audio_recorder/dspic_acoustic.h"
#include "sys/ioctl.h"

DspicAcoustic::~DspicAcoustic() {
    close(file_);
}

int DspicAcoustic::i2c_open() {
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DspicAcoustic_driver] Failed to open the I2C bus (%s) - %s",
                    i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),
                    "[DspicAcoustic_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE,
                    strerror(result));
        exit(1);
    }

    usleep(100000);
    return 0;
}

int DspicAcoustic::wait_recompute_signal() const {
    while (i2c_smbus_read_byte_data(file_, 0x05) != 0) {
        usleep(100000); // 100 ms
    }
    return EXIT_SUCCESS;
}

void DspicAcoustic::sync_pps() const {
    bool pps_sync = false;

    while (!pps_sync) {
        const double t = n_->now().seconds();
        if (const double dt = t - floor(t); dt < 0.5) {
            const unsigned int posix_seconds = static_cast<int>(ceil(t));
            uint8_t posix_seconds_bytes[4];
            for (int i = 0; i < 4; i++) {
                posix_seconds_bytes[i] = (posix_seconds >> (8 * i)) & 0xFF;
            }

            if (i2c_smbus_write_i2c_block_data(file_, 0xB0, 4, posix_seconds_bytes)) {
                RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - sync pps");
            } else {
                RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Synchronized PPS");
            }
            pps_sync = true;
        } else {
            // Sleep for 10 ms
            usleep(10000);
        }
    }
}

int DspicAcoustic::set_duration_between_shoot(const uint16_t duration_seconds) const {
    // shoot_duration_between
    if (i2c_smbus_write_word_data(file_, 0xB4, duration_seconds) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set duration between shoot");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set duration between shoot");
    return EXIT_SUCCESS;
}

int DspicAcoustic::set_shoot_offset_from_posix_zero(const uint16_t offset_seconds) const {
    // shoot_offset_from_posix_zero
    if (i2c_smbus_write_word_data(file_, 0xB6, offset_seconds) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
    return EXIT_SUCCESS;
}

int DspicAcoustic::set_robot_data(const int &data_size, const uint64_t &data) const {
    if (i2c_smbus_write_byte_data(file_, 0xD0, data_size) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set robot data size");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set robot data size");

    const uint8_t byte_size = ceil(data_size / 8.0);
    if (const int ret = i2c_smbus_write_i2c_block_data(file_,
                                                       0xD1,
                                                       byte_size,
                                                       reinterpret_cast<const uint8_t *>(&data)); ret < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set robot data");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set robot data");

    return EXIT_SUCCESS;
}

int DspicAcoustic::recompute_chirp(const uint16_t &frequency_middle, const uint16_t &frequency_range) const {
    // Set frequency middle
    if (i2c_smbus_write_word_data(file_, 0x01, frequency_middle) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set frequency_middle");
        return EXIT_FAILURE;
    } else {
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set frequency_middle");
    }

    // Set frequency range
    if (i2c_smbus_write_word_data(file_, 0x03, frequency_range) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set frequency_range");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set frequency_range");

    // Set chirp function
    if (i2c_smbus_write_byte_data(file_, 0x0D, 0x00) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set signal function");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set signal function");

    // recompute_signal
    if (i2c_smbus_write_byte_data(file_, 0x05, 0x01) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Recompute signal");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Recompute signal");

    return EXIT_SUCCESS;
}

int DspicAcoustic::set_robot_code(const uint8_t &robot_code) const {
    if (i2c_smbus_write_byte_data(file_, 0x00, robot_code) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set robot code = %i", robot_code);
    return EXIT_SUCCESS;
}

int DspicAcoustic::get_robot_code() const {
    return i2c_smbus_read_byte_data(file_, 0x00);
}

uint32_t DspicAcoustic::get_posix_time() const {
    uint32_t posix_time = 0;
    if (const int ret = i2c_smbus_read_i2c_block_data(file_, 0xB0, 4, reinterpret_cast<uint8_t *>(&posix_time));
        ret < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Get posix time");
        return 0;
    }
    return posix_time;
}

uint16_t DspicAcoustic::get_signal_number() const {
    return i2c_smbus_read_word_data(file_, 0x0A);
}

int DspicAcoustic::getI2CAddr() const {
    return i2c_addr_;
}

void DspicAcoustic::setI2CAddr(const int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &DspicAcoustic::getI2CPeriph() const {
    return i2c_periph_;
}

void DspicAcoustic::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}

int DspicAcoustic::enable_chirp(const bool enable) const {
    while (i2c_smbus_read_byte_data(file_, 0x08) != enable) {
        usleep(1000);
        if (i2c_smbus_write_byte_data(file_, 0x08, enable ? 0x01 : 0x00) < 0) {
            RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Enable chirp");
            break;
        } else {
            RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Chirp enable");
        }
        usleep(1000);
    }
    return EXIT_FAILURE;
}
