#include "seabot2_piston_driver/piston.h"
#include "sys/ioctl.h"

Piston::~Piston(){
    close(file_);
}

int Piston::i2c_open(){
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
        RCLCPP_WARN(n_->get_logger(), "[Piston_driver] Wrong PIC code version");

    usleep(100000);
    return 0;
}

void Piston::set_piston_reset() const{
    RCLCPP_INFO(n_->get_logger(),"[Piston_driver] Start resting piston");
    if(i2c_smbus_write_byte_data(file_, REGISTER_RESET, 0x01)<0)
        RCLCPP_WARN(n_->get_logger(),"[Piston_driver] I2C bus Failure - Piston Reset");
}

void Piston::set_regulation_dead_zone(const __u16 &val) const{
    if(i2c_smbus_write_word_data(file_, REGISTER_REGULATION_DEAD_ZONE, val)<0)
        RCLCPP_WARN(n_->get_logger(),"[Piston_driver] I2C bus Failure - Set dead zone");
}

void Piston::set_regulation_proportional(const __u16 &val) const{
    if(i2c_smbus_write_word_data(file_, REGISTER_REGULATION_PROPORTIONAL, val)<0)
        RCLCPP_WARN(n_->get_logger(),"[Piston_driver] I2C bus Failure - Set proportional");
}

int Piston::set_position(const int32_t &val) const{
    __u8 data[4];
    for(int i=0; i<4; i++){
        data[i] = val>>(8*i); /// ToDo : check LSB/MSB ?
    }
    if(i2c_smbus_write_i2c_block_data(file_, REGISTER_SET_POINT, 4, data)<0) {
        RCLCPP_WARN(n_->get_logger(), "[Piston_driver] I2C bus Failure - Set position");
        return EXIT_FAILURE;
    }
    else
        return EXIT_SUCCESS;
}

int Piston::get_all_data(){
    __u8 buff[REGISTER_DATA_SIZE];
    if (i2c_smbus_read_i2c_block_data(file_, REGISTER_DATA_READ, REGISTER_DATA_SIZE, buff) != REGISTER_DATA_SIZE) {
        RCLCPP_WARN(n_->get_logger(), "[Piston_driver] Error Reading data");
        return EXIT_FAILURE;
    }
    else{
        unsigned int u_position=0, u_position_set_point = 0;
        for(int i=0; i<=3; i++)
            u_position |= buff[i]<<(i*8);
        position_ = static_cast<int32_t>(u_position);

        switch_top_ = buff[4] & (0b1<<0);
        switch_bottom_ = buff[4] & (0b1 << 1);
        enable_ = buff[4] & (0b1<<2);
        motor_sens_ = buff[4] & (0b1<<3);
        state_ = buff[5];

        for(int i=6; i<=9; i++)
            u_position_set_point |= buff[i]<<((i-6)*8);
        position_set_point_ = static_cast<int32_t>(u_position_set_point);

        __u16 measured_battery_voltage = (__u16)buff[10] + ((__u16)buff[11]<<8);
        __u16 measured_motor_current = (__u16)buff[12] + ((__u16)buff[13] << 8);

        battery_voltage_ = measured_battery_voltage; //((float)(measured_battery_voltage)*3.3/4096.0) / (180.0/(180.0+820.0));
        motor_current_ = measured_motor_current; // ((float)(measured_motor_current)-2048.0)*(1.65/2048*0.264);
        motor_set_point_ = (__u16)buff[14] + ((__u16)buff[15] << 8);
        motor_cmd_ = (__u16)buff[16] + ((__u16)buff[17] << 8);
        return EXIT_SUCCESS;
    }
}

uint8_t& Piston::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    return pic_code_version_;
}

int Piston::getI2CAddr() const {
    return i2c_addr_;
}

void Piston::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const std::string &Piston::getI2CPeriph() const {
    return i2c_periph_;
}

void Piston::setI2CPeriph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
