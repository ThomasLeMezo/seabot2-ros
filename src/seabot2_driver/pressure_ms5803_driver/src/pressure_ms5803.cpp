#include "pressure_ms5803_driver/pressure_ms5803.h"
#include <sys/ioctl.h>


#define T_MIN 0.0
#define T_MAX 50.0
#define T_REF 20.0
#define P_MAX 14.0
#define P_MIN 0.0

using namespace std;

Pressure_ms5803::~Pressure_ms5803(){
    if(file_ != 0)
        close(file_);
}

int Pressure_ms5803::reset(){
    int res = i2c_smbus_write_byte(file_, CMD_RESET);
    usleep(30000); // 28ms reload for the sensor (?)
    //  usleep(30000);
    if (res < 0)
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Error reseting sensor");
    else
        RCLCPP_INFO(n_->get_logger(), "[Pressure_ms5803] Reset ok");
    return 0;
}

int Pressure_ms5803::i2c_open(){
    if ((file_ = open(i2c_periph_.c_str(), O_RDWR)) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Failed to open the I2C bus (%s)", i2c_periph_.c_str());
        exit(1);
    }

    if (ioctl(file_, I2C_SLAVE, i2c_addr_) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

int Pressure_ms5803::init_sensor(){
    i2c_open();

    RCLCPP_DEBUG(n_->get_logger(), "[Pressure_ms5803] Sensor initialization");
    reset();
    int return_val = 0;

    unsigned char buff[2] = {0, 0};
    for(size_t i=0; i<C_.size(); i++){
        __u8 add = CMD_PROM + (char) 2*(i+1);
        if (i2c_smbus_read_i2c_block_data(file_, add, 2, buff) != 2){
            RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Error Reading 0x%X", add);
            return_val = 1;
        }
        C_[i] = (buff[0] << 8) | buff[1] << 0;
        string text = "[Pressure_ms5803] C" + to_string(i) + " = "+ to_string(C_[i]);
        RCLCPP_DEBUG(n_->get_logger(), text.c_str());
    }
    if(return_val==0)
        RCLCPP_DEBUG(n_->get_logger(), "[Pressure_ms5803] Sensor Read PROM OK");

    return return_val;
}

bool Pressure_ms5803::measure(){
    if(measure_D1() && measure_D2()){
       return compute();
    }
    else
        return false;
}

bool Pressure_ms5803::compute() {
    int64_t dT = (int32_t)D2_ - ((int32_t)C_[4] << 8);
    int64_t TEMP = 2000+((dT * (int32_t)C_[5]) >> 23);

    int64_t OFF = ((int64_t)C_[1] << 16) + (((int64_t)C_[3] * dT) >> 7);
    int64_t SENS = ((int64_t)C_[0] << 15) + (((int64_t)C_[2] * dT) >> 8);

    /// Accurate temperature
    int64_t T2;
    int64_t OFF2, SENS2;
    if(TEMP < 2000){
        T2 = (3 * dT * dT ) >> 33;
        OFF2 = (3 * (TEMP-2000) * (TEMP-2000)) >> 1;
        SENS2 = (5 * (TEMP-2000) * (TEMP-2000)) >> 3;
        if(TEMP < -1500){
            OFF2 += 7 * (TEMP + 1500) * (TEMP+1500);
            SENS2 += 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    }
    else{
        T2 = (7 * dT * dT) >> 37;
        OFF2 = (1 * (TEMP-2000) * (TEMP-2000)) >> 4;
        SENS2 = 0;
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = (((D1_ * SENS) >> 21) - OFF) >> 15;

    temperature_ = static_cast<double>(TEMP) / 100.;
    pressure_ = static_cast<double>(P) / 10000.; /// in bar (/10 if in mbar)

    if(temperature_ < t_min_out_range_ || temperature_ > t_max_out_range_ || pressure_ < p_min_out_range_ || pressure_ > p_max_out_range_){
        if(n_ != nullptr)
            RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5803] Data out of range (p=%f t=%f)", pressure_, temperature_);
        if(pressure_ < p_min_out_range_ || pressure_ > p_max_out_range_) /// detection of overpressure (>50m)
            return false;
    }
    return true;
}

int Pressure_ms5803::getI2CAddr() const {
    return i2c_addr_;
}

void Pressure_ms5803::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const string &Pressure_ms5803::getI2CPeriph() const {
    return i2c_periph_;
}

void Pressure_ms5803::setI2CPeriph(const string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}

void Pressure_ms5803::setD1(u_int32_t d1) {
    D1_ = d1;
}

void Pressure_ms5803::setD2(u_int32_t d2) {
    D2_ = d2;
}
