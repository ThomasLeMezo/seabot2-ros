#ifndef LOGDATA_H
#define LOGDATA_H

#include <string>
#include <vector>
#include <boost/multiprecision/cpp_int.hpp>

using boost::multiprecision::cpp_int;

#define L93_EAST_MIN 0.0
#define L93_EAST_MAX 1300000.0
#define L93_NORTH_MIN 6000000.0
#define L93_NORTH_MAX 7200000.0
#define REF_POSIX_TIME 1604874973 //To be update every 5 years !
#define BATT_MIN 12.0
#define BATT_MAX 16.8

//enum MSG_TYPE:unsigned int {LOG_STATE=0, CMD_SLEEP=1, CMD_PARAMETERS=2, CMD_MISSION_NEW=3, CMD_MISSION_KEEP=4};
// First octet (4 bits) : message_id

#define NB_BITS_LOG1 120
// 15 bytes
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_LOG1, NB_BITS_LOG1, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_log1_t;

#define NB_BITS_CMD_SLEEP 16
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_SLEEP, NB_BITS_CMD_SLEEP, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_sleep_t;

#define NB_BITS_CMD_PARAMETERS 16
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_PARAMETERS, NB_BITS_CMD_PARAMETERS, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_parameters_t;

#define NB_BITS_CMD_WAYPOINT_DEPTH 24
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_WAYPOINT_DEPTH, NB_BITS_CMD_WAYPOINT_DEPTH, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_waypoint_depth_t;

#define NB_BITS_CMD_WAYPOINT_TRAJ 40
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_WAYPOINT_TRAJ, NB_BITS_CMD_WAYPOINT_TRAJ, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_waypoint_traj_t;

#define NB_BITS_CMD_WAYPOINT_TYPE 8
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_WAYPOINT_TYPE, NB_BITS_CMD_WAYPOINT_TYPE, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_waypoint_type_t;

#define NB_BITS_CMD_MISSION_HEADER 64
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_MISSION_HEADER, NB_BITS_CMD_MISSION_HEADER, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_mission_header_t;

class Waypoint{

public:
    Waypoint()= default;

    Waypoint(const double &duration_param, const double &depth_param, const double &east_param, const double &north_param, const bool &enable_thrusters_param=false, const bool &seafloor_landing_param=false){
        duration = duration_param;
        depth = depth_param;
        east = east_param;
        north = north_param;
        enable_thrusters = enable_thrusters_param;
        seafloor_landing = seafloor_landing_param;
    }
public:
    double north = 0.0;
    double east = 0.0;
    double depth = 0.0;
    double duration = 0;
    bool enable_thrusters = false;
    bool seafloor_landing = false;
};

class LogData
{
public:
    /**
     * @brief LogData
     */
    LogData()= default;

    enum MSG_TYPE:unsigned int {LOG_STATE=0, CMD_SLEEP=1, CMD_PARAMETERS=2, CMD_MISSION_NEW=3, CMD_MISSION_KEEP=4};

private:
    /**
     * @brief serialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @param min
     * @param max
     */
    template<typename T>
    unsigned int serialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, const double &value, const double &value_min, const double&value_max);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @param value_min
     * @param value_max
     * @return
     */
    template<typename T>
    unsigned int deserialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, double &value, const double &value_min, const double &value_max);

    /**
     * @brief serialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    template<typename T>
    unsigned int serialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, const unsigned int &value);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    template<typename T>
    unsigned int deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, unsigned int &value);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    template<typename T>
    unsigned int deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, int &value);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    template<typename T>
    unsigned int deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, bool &value);

public:

    /**
     * @brief deserialize_log_CMD
     * @param file_name
     * @param type
     * @return
     */
    bool deserialize_log_CMD(const std::string &raw_data);

    /**
     * @brief serialize_log_state
     * @param file_name
     * @return
     */
    std::string serialize_log_state(const long long &time);

    /**
     * @brief deserialize_log_CMD_sleep
     * @param file_name
     * @return
     */
    bool deserialize_log_CMD_sleep(const std::string &data);

    /**
     * @brief deserialize_log_CMD_mission
     * @param file_name
     * @return
     */
    bool deserialize_log_CMD_mission(const std::string &data);

    /**
     * @brief deserialize_log_CMD_parameters
     * @param file_name
     * @return
     */
    bool deserialize_log_CMD_parameters(const std::string &message);

    /**
     * @brief write_file
     * @param file_name
     * @param data
     * @return
     */
    static bool write_file(const std::string &file_name, const std::string &data, const unsigned int nb_bits = NB_BITS_LOG1);

    /**
     * @brief read_file
     * @param file_name
     * @return
     */
    static std::string read_file(const std::string &file_name);

private:

//  /**
//   * @brief serialize_log_CMD_waypoint
//   * @param save_file
//   * @param w
//   * @return
//   */
//  std::string serialize_log_CMD_waypoint(const Waypoint &w);

    /**
     * @brief LogData::deserialize_log_CMD_waypoint
     * @param save_file
     * @return
     */
    unsigned int deserialize_log_CMD_waypoint(const std::string &message, const unsigned int &bit_position);

public:
    double time_now_ = 0.0;

    double gnss_east_ = 42.0; // 2^21-1
    double gnss_north_ = 6000042.0; // 2^21-1 + 6e6
    double gnss_speed_ = 4.2;
    double gnss_heading_ = 242.0;
    double battery_ = 10.0;

    double gnss_mean_east_ = 42.0;
    double gnss_mean_north_ = 6000042.0;
    double gnss_mean_heading_ = 242.0;
    unsigned long start_time_ = REF_POSIX_TIME;

    double internal_pressure_ = 742.0;
    double internal_temperature_ = 42.0;
    double internal_humidity_ = 0.0;

    unsigned int  current_waypoint_ = 42; // 0 to 255 max

    unsigned int sleep_time_ = 0; // sleep time in min

    std::vector<Waypoint> waypoint_list_;
    double offset_east_ = 0.;
    double offset_north_ = 0.;
//  long long offset_time_ = TIME_POSIX_START; // in sec

    bool enable_mission_ = true;
    bool enable_flash_ = true;
    bool enable_depth_ = true;
    bool enable_engine_ = true;

    unsigned int last_cmd_received_ = 0;
    unsigned int period_message_ = 15; // in 10*min

    bool safety_global_safety_valid_ = false;
    bool safety_published_frequency_ = false;
    bool safety_depth_limit_ = false;
    bool safety_batteries_limit_ = false;
    bool safety_depressurization_ = false;
    bool safety_seafloor_ = false;
    bool safety_piston_ = false;
    bool safety_zero_depth_ = false;

    MSG_TYPE msg_type_;
};

template<typename T>
unsigned int LogData::serialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, const double &value, const double &value_min, const double &value_max){
    double scale = ((1<<nb_bit)-1)/(value_max-value_min);
    double bit_max = (1<<nb_bit)-1;
    long unsigned int v = static_cast<long unsigned int>(std::min(std::max(round((value-value_min)*scale), 0.0), bit_max));
    auto v_max = static_cast<long unsigned int>(bit_max);
    v = std::min(v, v_max);

    T mask = ((T(1)<<nb_bit)-1);
    bits &= ~(mask << start_bit);
    bits |= (T(v) & mask) << start_bit;
    return nb_bit;
}

template<typename T>
unsigned int LogData::deserialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, double &value, const double &value_min, const double &value_max){
    double scale = ((1<<nb_bit)-1.0)/(value_max-value_min);

    T mask = ((T(1)<<nb_bit)-1) << start_bit;
    T v = (bits & mask)>>start_bit;
    value = static_cast<double>(v)/scale + value_min;

//  std::cout << "---" << std::endl;
//  std::cout << "scale = " << scale << std::endl;
//  std::cout << "value min = " << value_min << std::endl;
//  std::cout << "binary value = " << v << std::endl;
//  std::cout << "scaled value = " << static_cast<double>(v)/scale << std::endl;
//  std::cout << "value = " << value << std::endl << std::endl;
    return nb_bit;
}

template<typename T>
unsigned int LogData::serialize_data(T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, const unsigned int &value){
    T mask = ((T(1)<<nb_bit)-1);
    bits &= ~(mask << start_bit);
    bits |= (T(value) & mask) << start_bit;
    return nb_bit;
}

template<typename T>
unsigned int LogData::deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, unsigned int &value){
    T mask = ((T(1)<<nb_bit)-1) << start_bit;
    T v = (bits & mask)>>start_bit;

    value = static_cast<unsigned int>(v);
    return nb_bit;
}

template<typename T>
unsigned int LogData::deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, int &value){
    T mask = ((T(1)<<nb_bit)-1) << start_bit;
    T v = (bits & mask)>>start_bit;

    value = static_cast<int>(v);// Two bit complement is automaticaly done
    return nb_bit;
}

template<typename T>
unsigned int LogData::deserialize_data(const T &bits, const unsigned int &nb_bit, const unsigned int &start_bit, bool &value){
    T mask = ((T(1)<<nb_bit)-1) << start_bit;
    T v = (bits & mask)>>start_bit;

    value = static_cast<bool>(v);
    return nb_bit;
}

#endif // LOGDATA_H
