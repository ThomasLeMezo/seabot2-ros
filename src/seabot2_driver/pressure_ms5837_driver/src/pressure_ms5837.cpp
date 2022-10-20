#include "pressure_ms5837_driver/pressure_ms5837.h"
#include <sys/ioctl.h>


#define T_MIN 0.0
#define T_MAX 50.0
#define T_REF 20.0
#define P_MAX 14.0
#define P_MIN 0.0

using namespace std;

Pressure_ms5837::~Pressure_ms5837(){
    if(file_ != 0)
        close(file_);
}

int Pressure_ms5837::reset(){
    int res = i2c_smbus_write_byte(file_, CMD_RESET);
    usleep(30000); // 28ms reload for the sensor (?)
    //  usleep(30000);
    if (res < 0)
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Error reseting sensor");
    else
        RCLCPP_INFO(n_->get_logger(), "[Pressure_ms5837] Reset ok");
    return 0;
}

int Pressure_ms5837::i2c_open(){
    if ((file_ = open(i2c_periph_.c_str(), O_RDWR)) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Failed to open the I2C bus (%s)", i2c_periph_.c_str());
        exit(1);
    }

    if (ioctl(file_, I2C_SLAVE, i2c_addr_) < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

int Pressure_ms5837::init_sensor(){
    i2c_open();

    RCLCPP_DEBUG(n_->get_logger(), "[Pressure_ms5837] Sensor initialization");
    reset();
    int return_val = 0;

    unsigned char buff[2] = {0, 0};
    for(size_t i=0; i<C_.size(); i++){
        __u8 add = CMD_PROM + (char) 2*(i+1);
        if (i2c_smbus_read_i2c_block_data(file_, add, 2, buff) != 2){
            RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Error Reading 0x%X", add);
            return_val = 1;
        }
        C_[i] = (buff[0] << 8) | buff[1] << 0;
        string text = "[Pressure_ms5837] C" + to_string(i) + " = "+ to_string(C_[i]);
        RCLCPP_DEBUG(n_->get_logger(), text.c_str());
    }
    if(return_val==0)
        RCLCPP_DEBUG(n_->get_logger(), "[Pressure_ms5837] Sensor Read PROM OK");

    return return_val;
}

bool Pressure_ms5837::measure(){
    if(measure_D1() && measure_D2()){
       return compute();
    }
    else
        return false;
}

bool Pressure_ms5837::compute() {
    int64_t dT = (int32_t)D2_ - ((int32_t)C_[5] << 8);
    int64_t TEMP = 2000+((dT * (int32_t)C_[6]) >> 23);

    int64_t OFF = ((int64_t)C_[2] << 16) + (((int64_t)C_[4] * dT) >> 7);
    int64_t SENS = ((int64_t)C_[1] << 15) + (((int64_t)C_[3] * dT) >> 8);

    /// Accurate temperature
    int64_t Ti;
    int64_t OFFi, SENSi;
    if(TEMP < 2000){
        Ti = (3 * dT * dT ) >> 33;
        OFFi = (3 * (TEMP-2000) * (TEMP-2000)) >> 1;
        SENSi = (5 * (TEMP-2000) * (TEMP-2000)) >> 3;
        if(TEMP < -1500){
            OFFi += 7 * (TEMP + 1500) * (TEMP+1500);
            SENSi += 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    }
    else{
        Ti = (2 * dT * dT) >> 37;
        OFFi = (1 * (TEMP-2000) * (TEMP-2000)) >> 4;
        SENSi = 0;
    }

    TEMP -= Ti;
    OFF -= OFFi;
    SENS -= SENSi;

    int64_t P = (((D1_ * SENS) >> 21) - OFF) >> 13;

    temperature_ = static_cast<double>(TEMP) / 100.;
    pressure_ = static_cast<double>(P) / 10000.; /// in bar (/10 if in mbar)

    if(temperature_ < t_min_out_range_ || temperature_ > t_max_out_range_ || pressure_ < p_min_out_range_ || pressure_ > p_max_out_range_){
        if(n_ != nullptr)
            RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Data out of range (p=%f t=%f)", pressure_, temperature_);
        if(pressure_ < p_min_out_range_ || pressure_ > p_max_out_range_) /// detection of overpressure (>50m)
            return false;
    }
    return true;
}

int Pressure_ms5837::getI2CAddr() const {
    return i2c_addr_;
}

void Pressure_ms5837::setI2CAddr(int i2CAddr) {
    i2c_addr_ = i2CAddr;
}

const string &Pressure_ms5837::getI2CPeriph() const {
    return i2c_periph_;
}

void Pressure_ms5837::setI2CPeriph(const string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}

void Pressure_ms5837::setD1(u_int32_t d1) {
    D1_ = d1;
}

void Pressure_ms5837::setD2(u_int32_t d2) {
    D2_ = d2;
}
