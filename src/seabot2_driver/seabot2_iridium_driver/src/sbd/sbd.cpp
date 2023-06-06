#include "seabot2_iridium_driver/sbd/sbd.h"
#include <string>
#include <vector>
#include <cmath>

#include <boost/algorithm/string.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <chrono>

using namespace boost;
using namespace std;

const std::string SBD::SBD_TOKEN_OK = "\r\nOK\r\n";
const std::string SBD::SBD_TOKEN_READY = "\r\nREADY\r\n";
const std::string SBD::SBD_TOKEN_SEPARATOR = "\r\n";

SBD::SBD(){
  omp_init_lock(&lock_data);
}

void SBD::init(const string &serial_port_name, const unsigned int &baud_rate){
  try{
    serial_.open(serial_port_name, baud_rate);
  }
  catch(boost::system::system_error& e){
    cout<<"Error: "<<e.what()<<endl;
  }

  disable_echo(); // Disable echo from SBD
}

SBD::~SBD(){
  serial_.close();
  omp_destroy_lock(&lock_data);
}

void SBD::read(){
  string result = serial_.readStringUntil(SBD_TOKEN_SEPARATOR);

  if(!result.empty()){
    if(debug_)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium_raw] %s", result.c_str());

    if(boost::starts_with(result, "OK")){
      omp_set_lock(&lock_data);
      OK_ = true;
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "READY")){
      omp_set_lock(&lock_data);
      READY_ = true;
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "ERROR")){
      omp_set_lock(&lock_data);
      ERROR_ = true;
      READY_ = false;
      OK_ = false;
      in_session_ = false;
      omp_unset_lock(&lock_data);
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "[Iridium] Received ERROR");
    }
    else if(boost::starts_with(result, "SBDRING")){
      omp_set_lock(&lock_data);
      ring_alert_ = true;
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+SBDREG:")){
      vector<string> fields;
      string result0 = result.substr(8);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      reg_status_ = stoi(fields[0]);
      reg_error_code_ = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CRIS:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      ring_alert_code_ = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CIEV:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      int indicator = stoi(fields[0]);
      omp_set_lock(&lock_data);
      switch(indicator){
      case 0:
        indicator_signal_ = stoi(fields[1]);
        //        cout << "=> Signal = " << indicator_signal_ << endl;
        break;
      case 1:
        indicator_service_ = stoi(fields[1]);
        //        cout << "=> Service = " << indicator_service_ << endl;
        break;
      case 2:
        indicator_antenna_ = stoi(fields[1]);
        //        cout << "=> Antenna = " << (indicator_antenna_?"Fault":"OK") << endl;
        break;
      default:
        break;
      }
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "300234")){
      omp_set_lock(&lock_data);
      imei_ = stoll(result);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CSQ:")){
      omp_set_lock(&lock_data);
      CSQ_ = stoi(result.substr(5));
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CSQF:")){
      omp_set_lock(&lock_data);
      CSQ_ = stoi(result.substr(6));
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "SBDTC")){
      omp_set_lock(&lock_data);
      copy_MO_MT_size_ = stoi(result.substr(49));
      omp_unset_lock(&lock_data);
    }
    else if(READY_){
      // result after a READY
      omp_set_lock(&lock_data);
      ready_return_ = stoi(result);
      READY_ = false;
      omp_unset_lock(&lock_data);

    }
    else if(boost::starts_with(result, "+SBDSX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      if(fields.size()>=6){
        omp_set_lock(&lock_data);
        STATUS_MO_ = stoi(fields[0]);
        STATUS_MOMSN_ = stoi(fields[1]);
        STATUS_MT_ = stoi(fields[2]);
        STATUS_MTMSN_ = stoi(fields[3]);
        STATUS_RA_ = stoi(fields[4]);
        waiting_ = stoi(fields[5]);

        if(ring_alert_ && STATUS_MT_==1)
          ring_alert_ = false;
        omp_unset_lock(&lock_data);
      }
    }
    else if(boost::starts_with(result, "+SBDLOE:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      trafic_management_status_ = stoi(fields[0]);
      trafic_management_time_ = stoi(fields[1]); // Check format
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+SBDIX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);

      omp_set_lock(&lock_data);
      if(fields.size()>=6){
        SESSION_MO_ = stoi(fields[0]);
        SESSION_MOMSN_ = stoi(fields[1]);
        SESSION_MT_ = stoi(fields[2]);
        SESSION_MTMSN_ = stoi(fields[3]);
        waiting_ = stoi(fields[5]);
      }
      in_session_ = false;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium] Session result %i", SESSION_MO_);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+AREG:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      areg_new_event_ = true;
      areg_event_ = stoi(fields[0]);
      areg_error_code_ = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "0")){
        omp_set_lock(&lock_data);
        valid_flush_ = true;
        return_flush_ = true;
        omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "1")){
        omp_set_lock(&lock_data);
        valid_flush_ = false;
        return_flush_ = true;
        omp_unset_lock(&lock_data);
    }
    else if(read_msg_){ // Keep at the end
      omp_set_lock(&lock_data);
      read_msg_data_ = result.substr(2, result.size()-4); // checksum + ending token
      read_msg_ = false;
      omp_unset_lock(&lock_data);
    }
  }

//  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/**
 * Note : each line fillful this format \r\n *** \r\n
 */
void SBD::write(const std::string &at_cmd){
  omp_set_lock(&lock_data);
  OK_ = false;
  omp_unset_lock(&lock_data);

  string cmd = at_cmd + '\r';
  serial_.writeString(cmd);
  if(debug_)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridiuraw_] send = %s", cmd.c_str());
}

void SBD::disable_echo(){
  write("ATE0");
}

int SBD::cmd_CSQ(bool fast){
  omp_set_lock(&lock_data);
  CSQ_ = -1;
  omp_unset_lock(&lock_data);

  write("AT+CSQ" + string(fast?"F":""));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  omp_set_lock(&lock_data);
  int result = CSQ_;
  omp_unset_lock(&lock_data);
  return result;
}

long long SBD::cmd_get_imei(){
  write("AT+CGSN");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  omp_set_lock(&lock_data);
  long long result = imei_;
  omp_unset_lock(&lock_data);
  return result;
}

int SBD::cmd_copy_MO_MT(){
  omp_set_lock(&lock_data);
  copy_MO_MT_size_ = -1;
  omp_unset_lock(&lock_data);
  write("AT+SBDTC");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  omp_set_lock(&lock_data);
  int result = copy_MO_MT_size_;
  omp_unset_lock(&lock_data);
  return result;
}

int SBD::cmd_write_message(const std::string &data){
  set_ready(false);

  if(data.size()>340)
    return 4;
  string cmd = "AT+SBDWB=" + std::to_string(data.size());
  write(cmd);

  string data_checksum(data);
  // Compute checksum
  uint16_t checksum = 0;
  for(const char &c:data)
    checksum += static_cast<unsigned char>(c);
  data_checksum += static_cast<char>(checksum>>8);
  data_checksum += static_cast<char>(checksum);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  write(data_checksum);

  //  for(int i=0; i<data_checksum.size(); i++){
  //    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium_sbdwb_] Send = %x", data_checksum.c_str()[i]);
  //  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool valid = false;
  for(size_t i=0; i<20; i++){
    if(is_ready()){
      valid = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return (valid)?EXIT_SUCCESS:EXIT_FAILURE;
}

std::string SBD::cmd_read_message(){
  string cmd = "AT+SBDRB";
  omp_set_lock(&lock_data);
  read_msg_ = true;
  omp_unset_lock(&lock_data);

  write(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  omp_set_lock(&lock_data);
  string result = read_msg_data_;
  omp_unset_lock(&lock_data);

  return result;
}

int SBD::cmd_flush_message(const bool &MO, const bool &MT){
  string cmd = "AT+SBDD";
  if(MO && MT)
    cmd += to_string(2);
  else if(MO)
    cmd += to_string(0);
  else
    cmd += to_string(1);
  write(cmd);

  for(size_t i=0; i<4*30; i++){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(is_flush_return()){
        if(is_flush_valid()){
            return 0;
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium] Error flushing device");
            return 1;
        }
    }
  }

  return 0;
}

int SBD::cmd_status(){
  string cmd = "AT+SBDSX";
  write(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 0;
}

int SBD::cmd_session(){
  omp_set_lock(&lock_data);
  bool answer = ring_alert_;
  SESSION_MO_ = -2;
  SESSION_MT_ = -2;
  omp_unset_lock(&lock_data);

  omp_set_lock(&lock_data);
  in_session_ = true;
  omp_unset_lock(&lock_data);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium] Start a session");
  string cmd = "AT+SBDIX" + string(answer?"A":"");
  if(valid_gnss_){
    int lat_deg = int(latitude_);
    double lat_min = int(abs(latitude_ - lat_deg)*60000.)/1000.;
    int lon_deg = int(longitude_);
    double lon_min = int(abs(longitude_ - lon_deg)*60000.)/1000.;
    cmd += "=";

    std::stringstream lat_string, lon_string;
    if(lat_deg<0)
      lat_string << "-";
    lat_string << setfill('0') << setw(2) << abs(lat_deg);
    //std::fesetround(FE_TONEAREST);
    lat_string << setw(5) << std::fixed << std::setprecision(3) << lat_min;
    cmd += lat_string.str();

    cmd += ",";

    if(lon_deg<0)
      lon_string << "-";
    lon_string << setfill('0') << setw(3) << abs(lon_deg);
    lon_string << setw(5) << std::fixed << std::setprecision(3) << lon_min;
    cmd += lon_string.str();
  }

  write(cmd);

  for(size_t i=0; i<4*30; i++){
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if(!is_in_session()){
      return 0;
    }
  }

  omp_set_lock(&lock_data);
  in_session_ = false;
  omp_unset_lock(&lock_data);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Iridium] End of session not received");

  exit(1);
}

int SBD::cmd_enable_alert(const bool &enable){
  string cmd = "AT+SBDMTA=";
  cmd += string(enable?"1":"0");
  write(cmd);
  return 0;
}

int SBD::cmd_enable_indicator_reporting(const bool &enable){
  string cmd = "AT+CIER=";
  cmd += string(enable?"1":"0");
  cmd += ",1,1,1,0";
  write(cmd);
  return 0;
}

int SBD::cmd_trafic_management_status(){
  string cmd = "AT+SBDLOE";
  write(cmd);
  return 0;
}

int SBD::cmd_set_registration_mode(const int &mode){
  string cmd = "AT+SBDAREG=";
  cmd += to_string(mode);
  write(cmd);
  return 0;
}

bool SBD::sbd_power(const bool &enable){
  if(enable != iridium_power_state_){

    string gpio_file = "/sys/class/gpio/gpio" + to_string(gpio_power_) + "/value";

    ofstream setvalgpio(gpio_file.c_str()); // open value file for gpio
    if (!setvalgpio.is_open()){
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "[IRIDIUM] Unable to write power on on GPIO %u", gpio_power_);
      return false;
    }

    setvalgpio << (enable?1:0);//write value to value file
    setvalgpio.close();// close value file

    iridium_power_state_ = enable;
  }
  return true;
}


