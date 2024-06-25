#include "seabot2_audio_recorder/dspic_acoustic.h"
#include "sys/ioctl.h"

DspicAcoustic::~DspicAcoustic(){
    close(file_);
}

int DspicAcoustic::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DspicAcoustic_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[DspicAcoustic_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }

    usleep(100000);
    return 0;
}

int DspicAcoustic::sync_pps() {
    bool pps_sync = false;

    while (!pps_sync){
        double t = n_->now().seconds();
        double dt = t - ceil(t);
        if (dt < 0.2) {
            int modulo = (int) ceil(t) % 15;
            if (i2c_smbus_write_byte_data(file_, 0x00, modulo) < 0) {
                RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - sync pps");
            }
            else {
                RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Synchronized PPS");
                break;
            }
        } else {
            // Sleep for 10 ms
            usleep(10000);
        }
    }
    return EXIT_SUCCESS;
}

int DspicAcoustic::set_pps_sync_chirp_id(uint8_t chirp_id) {
    if (i2c_smbus_write_byte_data(file_, 0x02, chirp_id) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Set PPS sync chirp id to %d", chirp_id);
    }
    else
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set PPS sync chirp id to %d", chirp_id);
    return 0;
}

uint8_t DspicAcoustic::get_pps_value() const{
    return i2c_smbus_read_byte_data(file_, 0x00);
}

int DspicAcoustic::getI2CAddr() const {
    return i2c_addr_;
}

void DspicAcoustic::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &DspicAcoustic::getI2CPeriph() const {
    return i2c_periph_;
}

void DspicAcoustic::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}

int DspicAcoustic::enable_chirp(bool enable){
    if (i2c_smbus_write_byte_data(file_, 0x08, enable?0x01:0x00) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - Enable chirp");
        return EXIT_FAILURE;
    }
    else {
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Chirp enable");
        return EXIT_SUCCESS;
    }
}