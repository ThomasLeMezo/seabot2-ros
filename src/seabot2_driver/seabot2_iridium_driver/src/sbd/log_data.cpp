#include "seabot2_iridium_driver/sbd/log_data.h"

#include "boost/filesystem.hpp"
#include <boost/multiprecision/cpp_int.hpp>
#include <iostream>
#include <iterator>

#include <fstream>
#include <string>
#include <chrono>

using namespace std;
using boost::multiprecision::cpp_int;

bool LogData::deserialize_log_CMD_sleep(const string &message){
  cout << "Deserialize log sleep" << endl;

  uint_cmd_sleep_t data = (uint_cmd_sleep_t(1) << NB_BITS_CMD_SLEEP) - 1;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  unsigned int bit_position = 4;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 12, bit_position, sleep_time_);
  return true;
}

bool LogData::deserialize_log_CMD_parameters(const string &message){
  cout << "Deserialize log parameters" << endl;

  uint_cmd_parameters_t data;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  unsigned int bit_position = 4;
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, enable_mission_);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, enable_flash_);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, enable_depth_);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, enable_engine_);

  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 8, bit_position, period_message_);

  return true;
}

unsigned int LogData::deserialize_log_CMD_waypoint(const std::string &message, const unsigned int &bit_position){
  string message_wp_type = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_TYPE/8));
  uint_cmd_waypoint_type_t data_type= (uint_cmd_waypoint_type_t(1)<<NB_BITS_CMD_WAYPOINT_TYPE) -1;
  memcpy(data_type.backend().limbs(), message_wp_type.c_str(), message_wp_type.size());

   unsigned int bit_position_local = 1;
   unsigned int duration;

  if((data_type & 0x1) == 1){
      string message_wp_data = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_TRAJ/8));
      uint_cmd_waypoint_traj_t data_traj = (uint_cmd_waypoint_traj_t(1)<<NB_BITS_CMD_WAYPOINT_TRAJ) -1;
      memcpy(data_traj.backend().limbs(), message_wp_data.c_str(), message_wp_data.size());

      int east, north;
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 9, bit_position_local, duration);
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 15, bit_position_local, east);
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 15, bit_position_local, north);

      Waypoint w(duration*60.0, 0.0, east*4.0 + gnss_mean_east_, north * 4.0 + gnss_mean_north_, true);
      waypoint_list_.push_back(w);
  }
  else{
      string message_wp_data = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_DEPTH/8));
      uint_cmd_waypoint_depth_t data_depth = (uint_cmd_waypoint_depth_t(1)<<NB_BITS_CMD_WAYPOINT_DEPTH) -1;
      memcpy(data_depth.backend().limbs(), message_wp_data.c_str(), message_wp_data.size());

      unsigned int depth;
      bool seafloor_landing=false;
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 9, bit_position_local, duration);
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 11, bit_position_local, depth);
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 1, bit_position_local, seafloor_landing);
      bit_position_local += 2;

      Waypoint w(duration*60.0,depth/4.0, 0.0, 0.0, false, seafloor_landing);
      waypoint_list_.push_back(w);
  }

  return bit_position_local;
}

bool LogData::deserialize_log_CMD_mission(const string &message){
  cout << "Deserialize log mission" << endl;
  waypoint_list_.clear();

  uint_cmd_mission_header_t data = (uint_cmd_mission_header_t(1) << NB_BITS_CMD_MISSION_HEADER) - 1;
  memcpy(data.backend().limbs(), message.c_str(), NB_BITS_CMD_MISSION_HEADER/8);

  unsigned int bit_position = 4;
  unsigned int nb_waypoints, start_time;

  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 8, bit_position, nb_waypoints);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 22, bit_position, start_time);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 15, bit_position, gnss_mean_east_, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 15, bit_position, gnss_mean_north_, L93_NORTH_MIN, L93_NORTH_MAX);

    start_time_ = REF_POSIX_TIME + start_time * 60;

  cout << "nb_waypoints = " << nb_waypoints << endl;
  cout << "m_start_time time = " << start_time_ << endl;
  cout << "start_time = " << start_time << endl;
  cout << "m_mean_north = " << gnss_mean_north_ << endl;
  cout << "m_mean_east = " << gnss_mean_east_ << endl;

  // Deserialize waypoint list and add to list
  for(size_t i=0; i<nb_waypoints; i++)
      bit_position += deserialize_log_CMD_waypoint(message, bit_position);

  return true;
}

std::string LogData::serialize_log_state(const long long &time){
  // Set all bits to 1 (enable add option)
  uint_log1_t data = (uint_log1_t(1) << NB_BITS_LOG1) - 1;

  unsigned int bit_position = 0;
  bit_position += serialize_data<uint_log1_t>(data, 4, bit_position, LOG_STATE);

  // Get timestamped of the day
  struct tm * timeinfo;
  time_t t1 = time; // current timestamp
  timeinfo = gmtime(&t1);
  timeinfo->tm_hour = 0;
  timeinfo->tm_min = 0;
  timeinfo->tm_sec = 0;
  time_t t2 = mktime(timeinfo); // timestamp at 00:00:00 of the day

  long time_sec_day = t1-t2; // number of seconds since 00:00:00 of the day
  int time_LQ = static_cast<int>(round(time_sec_day/3.0));
  /// Todo : change to serailize lat/long positioning instead of L93
  /// message size in SBD0 = 30 bytes = 240 bits => message can be x2 in size
  /// 24bit for lat and long (about 1m positioning) => +6 bit
  bit_position += serialize_data<uint_log1_t>(data, 14, bit_position, time_LQ, 0, ((1<<14) -1));
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, gnss_mean_east_, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, gnss_mean_north_, L93_NORTH_MIN, L93_NORTH_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, gnss_speed_, 0, 5.0);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, gnss_mean_heading_, 0, 359.0);

  unsigned char state = 0;
  state |= (safety_global_safety_valid_ & 0x1) << 0;
  state |= (safety_published_frequency_ & 0x1) << 1;
  state |= (safety_depth_limit_ & 0x1) << 2;
  state |= (safety_batteries_limit_ & 0x1) << 3;
  state |= (safety_depressurization_ & 0x1) << 4;
  state |= (safety_seafloor_ & 0x1) << 5;
  state |= (safety_piston_ & 0x1) << 6;
  state |= (safety_zero_depth_ & 0x1) << 7;

  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, state);

  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, battery_, BATT_MIN, BATT_MAX);

  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, internal_pressure_, 680.0, 800.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, internal_temperature_, 8.0, 50.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, internal_humidity_, 50.0, 100.0);

  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, current_waypoint_);
  bit_position += serialize_data<uint_log1_t>(data, 4, bit_position, last_cmd_received_);
  return {(char*)data.backend().limbs(), NB_BITS_LOG1/8};
}

bool LogData::write_file(const string &file_name, const string &data, const unsigned int nb_bits){
  ofstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cerr << "Unable to open " << file_name << " new log file : " << errno << endl;
    return false;
  }

  save_file.write(data.c_str(), nb_bits/8);
  save_file.close();

  return true;
}

std::string LogData::read_file(const std::string &file_name){
  std::ifstream save_file;
  save_file.open(file_name);
  if(!save_file.is_open()){
    cout << "File cannot be open" << strerror(errno) << endl;
    cout << "file_name = " << file_name << endl;
    return "";
  }
  std::string data((std::istreambuf_iterator<char>(save_file)),
                   std::istreambuf_iterator<char>());

  save_file.close();
  return data;
}

bool LogData::deserialize_log_CMD(const string &raw_data){
  unsigned char message_type = raw_data[0] & 0xF;

  switch(message_type){
  case CMD_SLEEP:
      msg_type_ = CMD_SLEEP;
    cout << "CMD Sleep" << endl;
    deserialize_log_CMD_sleep(raw_data);
    break;
  case CMD_PARAMETERS:
    cout << "CMD Parameters" << endl;
          msg_type_ = CMD_PARAMETERS;
    deserialize_log_CMD_parameters(raw_data);
      break;
  case CMD_MISSION_NEW:
    cout << "CMD Mission NEW" << endl;
          msg_type_ = CMD_MISSION_NEW;
    deserialize_log_CMD_mission(raw_data);
    break;
  case CMD_MISSION_KEEP:
    cout << "CMD Mission" << endl;
          msg_type_ = CMD_MISSION_KEEP;
    deserialize_log_CMD_mission(raw_data);
    break;

  default:
      cout << "MESSAGE TYPE NOT FOUND : " << static_cast<int>(message_type) << endl;
    break;
  }
  return true;
}
