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
        double dt = t - floor(t);
        if (dt < 0.5) {
            unsigned int posix_seconds = (int) ceil(t);
            RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] %i", posix_seconds);
            uint8_t posix_seconds_bytes[4];
            for (int i = 0; i < 4; i++) {
                posix_seconds_bytes[i] = (posix_seconds >> (8 * i)) & 0xFF;
                RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] %i", posix_seconds_bytes[i]);
            }

            if (i2c_smbus_write_i2c_block_data(file_, 0xB0, 4, posix_seconds_bytes)) {
                RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - sync pps");
            }
            else {
                RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Synchronized PPS");
            }
            pps_sync = true;
        } else {
            // Sleep for 10 ms
            usleep(10000);
        }
    }
    return EXIT_SUCCESS;
}

int DspicAcoustic::set_duration_between_shoot(uint16_t duration_seconds){
    if (i2c_smbus_write_word_data(file_, 0xB4, duration_seconds) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set duration between shoot");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set duration between shoot");
        return EXIT_SUCCESS;
    }
}

int DspicAcoustic::set_shoot_offset_from_posix_zero(uint16_t offset_seconds){
    if (i2c_smbus_write_word_data(file_, 0xB6, offset_seconds) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
        return EXIT_SUCCESS;
    }
}

int DspicAcoustic::recompute_chirp(const uint16_t &frequency_middle, const uint16_t &frequency_range){

    // Set frequency middle
    if (i2c_smbus_write_word_data(file_, 0x01, frequency_middle) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
    }

    // Set frequency range
    if (i2c_smbus_write_word_data(file_, 0x03, frequency_range) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
    }

    // Set chirp function
    if (i2c_smbus_write_byte_data(file_, 0x0D, 0x00) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
    }

    // recompute_signal
    if (i2c_smbus_write_byte_data(file_, 0x01, 0x01) < 0){
        RCLCPP_WARN(n_->get_logger(), "[DSPIC_ACOUSTIC] I2C bus Failure - set shoot offset from posix zero");
        return EXIT_FAILURE;
    }
    else{
        RCLCPP_INFO(n_->get_logger(), "[DSPIC_ACOUSTIC] Set shoot offset from posix zero");
    }

    return EXIT_SUCCESS;
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