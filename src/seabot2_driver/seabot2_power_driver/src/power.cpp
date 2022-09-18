#include "seabot2_power_driver/power.h"
#include "sys/ioctl.h"

Power::~Power(){
    close(file_);
}

int Power::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Power_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(file_));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[Power_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(result));
        exit(1);
    }

    if(get_version()!=code_version_)
        RCLCPP_WARN(n_->get_logger(), "[Power_driver] Wrong PIC code version");

    usleep(100000);
    return 0;
}

int Power::get_all_data(){
    __u8 buff[REGISTER_DATA_SIZE];
    if (i2c_smbus_read_i2c_block_data(file_, REGISTER_DATA_READ, REGISTER_DATA_SIZE, buff) != REGISTER_DATA_SIZE) {
        RCLCPP_WARN(n_->get_logger(), "[Power_driver] Error Reading data");
        return EXIT_FAILURE;
    }
    else{
        /// ToDo : check if read by small block avoid errors ?
        for(int i=0; i<4; i++){
            cell_volt_[i] = ((int)buff[2*i] + ((int)buff[2*i+1]<<8)) * CONVERT_BRIDGE_BATTERY * ((R1_[i]+R2_[i])/R1_[i]);
        }

        /// Cells are a cumulative sum, restore here the value of each cell
        battery_volt_ = cell_volt_.back();
        for(size_t i=3; i>0; i--)
            cell_volt_[i] -= cell_volt_[i-1];

        for(int i=4; i<6; i++){
            esc_current_[i - 4] = ((int)buff[2 * i] + ((int)buff[2 * i + 1] << 8)) * CONVERT_BRIDGE_CURRENT;
        }
        motor_current_ = ((int)buff[12] + ((int)buff[13] << 8)) * CONVERT_BRIDGE_CURRENT;
        power_state_ = buff[14];

        return EXIT_SUCCESS;
    }
}

int Power::set_sleep(){
    if (i2c_smbus_write_byte_data(file_, 0x00, 0x02)) {
        RCLCPP_WARN(n_->get_logger(), "[Power_driver] Error sending sleep cmd");
        return EXIT_FAILURE;
    }
    else
        return EXIT_SUCCESS;
}

uint8_t& Power::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    return pic_code_version_;
}

int Power::getI2CAddr() const {
    return i2c_addr_;
}

void Power::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &Power::getI2CPeriph() const {
    return i2c_periph_;
}

void Power::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
