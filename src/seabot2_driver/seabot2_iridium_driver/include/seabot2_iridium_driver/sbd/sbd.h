#ifndef SBD_H
#define SBD_H

#include <string>
#include "seabot2_iridium_driver/serial/BufferedAsyncSerial.h"
#include "omp.h"

//#define SBD_TOKEN_OK "\r\nOK\r\n"
//#define SBD_TOKEN_READY "\r\nREADY\r\n"
//#define SBD_TOKEN_SEPARATOR "\r\n"

class SBD
{
public:

  /**
   * @brief SBD
   * @param serial_port
   * @param baud_rate
   * @param timeout (in ms)
   */
  SBD();

  ~SBD();

    const static std::string SBD_TOKEN_OK;
    const static std::string SBD_TOKEN_READY;
    const static std::string SBD_TOKEN_SEPARATOR;

  /**
   * @brief init
   */
  int init(const std::string &serial_port_name="/dev/ttyAMA0", const unsigned int &baud_rate=19200);

  /**
   * @brief disable_echo
   */
  void disable_echo();

  /**
   * @brief cmd_CSQ
   */
  int cmd_CSQ(bool fast=true);

  /**
   * @brief cmd_get_imei
   * @return
   */
  long long cmd_get_imei();

  /**
   * @brief cmd_copy_MO_MT
   */
  int cmd_copy_MO_MT();

  /**
   * @brief cmd_write_message
   * @param data
   * @param data_size
   * @return
   */
  int cmd_write_message(const std::string &data);

  /**
   * @brief cmd_read_message
   * @param data
   * @return
   */
  std::string cmd_read_message();

  /**
   * @brief SBD::cmd_status
   */
  int cmd_status();

  /**
   * @brief cmd_flush_message
   * @param MO
   * @param MT
   * @return
   */
  int cmd_flush_message(const bool &MO, const bool &MT);

  /**
   * @brief SBD::cmd_session
   * @param answer
   * @return
   */
  int cmd_session();

  /**
   * @brief cmd_enable_alert
   * @param enable
   * @return
   */
  int cmd_enable_alert(const bool &enable=true);

  /**
   * @brief cmd_enable_indicator_reporting
   * @param enable
   * @return
   */
  int cmd_enable_indicator_reporting(const bool &enable=true);

  /**
   * @brief cmd_trafic_management_status
   * @return
   */
  int cmd_trafic_management_status();

  /**
   * @brief set_gnss
   * @param latitude
   * @param longitude
   */
  void set_gnss(const double &latitude, const double &longitude);

  /**
   * @brief cmd_set_registration_mode
   * @param mode
   * @return
   */
  int cmd_set_registration_mode(const int &mode);

  /**
   * @brief decode_sentence
   */
  void read();

  /**
   * @brief sbd_power
   * @param enable
   * @return
   */
  bool sbd_power(const bool &enable);

  /**
   * @brief set_debug
   * @param val
   * @return
   */
  void set_debug(const bool &val);

private:

  /**
   * @brief write_AT
   * @param cmd
   * @return
   */
  void write(const std::string &at_cmd);

private:
  BufferedAsyncSerial serial_;
  bool valid_gnss_ = false;
  double latitude_ = 48.39475416;
  double longitude_ = -4.48271749;

  bool OK_ = false;
  bool ERROR_ = false;
  bool READY_ = false;
  bool read_msg_ = false;

  bool return_flush_ = false;
  bool valid_flush_ = false;

  bool in_session_ = false;

  omp_lock_t lock_data{};


  int STATUS_MO_ = -2;
  int STATUS_MOMSN_ = -2;
  int STATUS_MT_ = -2;
  int STATUS_MTMSN_ = -2;
  int STATUS_RA_ = -2;
  int waiting_ = -2;

  int SESSION_MO_ = -2;
  int SESSION_MOMSN_ = -2;
  int SESSION_MT_ = -2;
  int SESSION_MTMSN_ = -2;

  int CSQ_ = -1;
  int copy_MO_MT_size_ = -1;

  long long imei_ = 0;

  int ready_return_ = -1;

  std::string read_msg_data_;

  int indicator_signal_ = -1;
  int indicator_service_ = -1;
  int indicator_antenna_ = -1;

  bool ring_alert_ = false;
  int ring_alert_code_ = -1;

  int trafic_management_status_ = -1;
  int trafic_management_time_ = -1;

  bool areg_new_event_ = false;
  int areg_event_ = -1;
  int areg_error_code_ = -1;

  unsigned int gpio_power_ = 5;
  bool iridium_power_state_ = false;

  int reg_status_ = -1;
  int reg_error_code_ = -1;

  bool debug_ = false;

public:
  int get_status_mo();
  int get_status_momsn();
  int get_status_mt();
  int get_status_mtmsn();
  int get_status_ra();
  int get_waiting();
  int get_session_mo();
  int get_session_momsn();
  int get_session_mt();
  int get_session_mtmsn();
  int get_csq();
  int get_copy_mo_mt_size();
  long long get_imei();
  int get_write_return();
  std::string get_read_msg_data();
  int get_indicator_signal();
  int get_indicator_service();
  int get_indicator_antenna();
  bool get_ring_alert();
  int get_ring_alert_code();
  int get_trafic_management_status();
  int get_trafic_management_time();
  bool get_areg_new_event();
  int get_areg_event();
  int get_areg_error_code();
  bool is_in_session();
  bool is_flush_valid();
  bool is_flush_return();
  bool is_ready();
  void set_ready(const bool &val);

};

inline void SBD::set_debug(const bool &val){
  debug_ = val;
}

inline void SBD::set_gnss(const double &latitude, const double &longitude){
  latitude_ = latitude;
  longitude_ = longitude;
  valid_gnss_ = true;
}

inline int SBD::get_status_mo(){
  omp_set_lock(&lock_data);
  int result = STATUS_MO_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_momsn(){
  omp_set_lock(&lock_data);
  int result = STATUS_MOMSN_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_mt(){
  omp_set_lock(&lock_data);
  int result = STATUS_MT_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_mtmsn(){
  omp_set_lock(&lock_data);
  int result = STATUS_MTMSN_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_ra(){
  omp_set_lock(&lock_data);
  int result = STATUS_RA_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_waiting(){
  omp_set_lock(&lock_data);
  int result = waiting_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mo(){
  omp_set_lock(&lock_data);
  int result = SESSION_MO_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_momsn(){
  omp_set_lock(&lock_data);
  int result = SESSION_MOMSN_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mt(){
  omp_set_lock(&lock_data);
  int result = SESSION_MT_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mtmsn(){
  omp_set_lock(&lock_data);
  int result = SESSION_MTMSN_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_csq(){
  omp_set_lock(&lock_data);
  int result = CSQ_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_copy_mo_mt_size(){
  omp_set_lock(&lock_data);
  int result = copy_MO_MT_size_;
  omp_unset_lock(&lock_data);
  return result;
}

inline long long SBD::get_imei(){
  omp_set_lock(&lock_data);
  long result = imei_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_write_return(){
  omp_set_lock(&lock_data);
  int result = ready_return_;
  omp_unset_lock(&lock_data);
  return result;
}

inline std::string SBD::get_read_msg_data(){
  omp_set_lock(&lock_data);
  std::string result = read_msg_data_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_signal(){
  omp_set_lock(&lock_data);
  int result = indicator_signal_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_service(){
  omp_set_lock(&lock_data);
  int result = indicator_service_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_antenna(){
  omp_set_lock(&lock_data);
  int result = indicator_antenna_;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::get_ring_alert(){
  omp_set_lock(&lock_data);
  bool result = ring_alert_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_ring_alert_code(){
  omp_set_lock(&lock_data);
  int result = ring_alert_code_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_trafic_management_status(){
  omp_set_lock(&lock_data);
  int result = trafic_management_status_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_trafic_management_time(){
  omp_set_lock(&lock_data);
  int result = trafic_management_time_;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::get_areg_new_event(){
  omp_set_lock(&lock_data);
  bool result = areg_new_event_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_areg_event(){
  omp_set_lock(&lock_data);
  int result = areg_event_;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_areg_error_code(){
  omp_set_lock(&lock_data);
  int result = areg_error_code_;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::is_in_session(){
  omp_set_lock(&lock_data);
  bool result = in_session_;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::is_flush_valid(){
    omp_set_lock(&lock_data);
    bool result = valid_flush_;
    omp_unset_lock(&lock_data);
    return result;
}

inline bool SBD::is_flush_return(){
    omp_set_lock(&lock_data);
    bool result = return_flush_;
    omp_unset_lock(&lock_data);
    return result;
}

inline bool SBD::is_ready(){
  omp_set_lock(&lock_data);
  bool result = READY_;
  omp_unset_lock(&lock_data);
  return result;
}

inline void SBD::set_ready(const bool &val){
  omp_set_lock(&lock_data);
  READY_ = val;
  omp_unset_lock(&lock_data);
}


#endif // SBD_H
