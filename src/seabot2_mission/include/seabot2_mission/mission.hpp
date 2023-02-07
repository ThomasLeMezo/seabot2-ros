#ifndef MISSION_H
#define MISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include <boost/property_tree/ptree.hpp>
#include <rclcpp/rclcpp.hpp>
#include "seabot2_mission/msg/waypoint.hpp"

#include <filesystem>
namespace fs = std::filesystem;

class Waypoint{

public:
    Waypoint(){}

    Waypoint(const rclcpp::Time &time_end_param, const double &depth_param, const double &north_param, const double &east_param, const double&limit_velocity_param, const double&approach_velocity_param, const bool &enable_thrusters_param=true, const bool &seafloor_landing_param=false){
        time_end = time_end_param;
        depth = depth_param;
        east = east_param;
        north = north_param;
        limit_velocity = limit_velocity_param;
        approach_velocity = approach_velocity_param;
        enable_thrusters = enable_thrusters_param;
        seafloor_landing = seafloor_landing_param;
    }
public:
    double north = 0.0;
    double east = 0.0;
    double depth = 0.0;
    double limit_velocity = 0.0;
    double approach_velocity = 1.0;
    bool  enable_thrusters = true;
    bool seafloor_landing = false;
    rclcpp::Time time_end;
};

class Mission
{
public:
    /**
       * @brief Mission
       */
    Mission(rclcpp::Node *n);

    //    /**
    //     * @brief load_mission
    //     * @param filename
    //     */
    //    void load_mission(const std::string &file_name);

    /**
       * @brief update_mission
       * @return
       */
    bool update_mission();

    /**
       * @brief compute_command
       * @param north
       * @param east
       * @param depth
       */
    bool compute_command(seabot2_mission::msg::Waypoint &wp);

    /**
     * Test if mission file was updated
     * @param file_xml
     * @param folder_path
     * @return
     */
    bool is_new_mission_file(const std::string &file_xml, const std::string &folder_path);

    /**
     * Load mission file
     * @param file_xml
     * @param folder_path
     * @return error code
     */
    int load_mission(const std::string &file_xml, const std::string &folder_path="");

    /**
     * @brief is_mission_enable
     * @return
     */
    bool is_mission_enable() const;

    /**
     * @brief get_current_waypoint
     * @return
     */
    size_t get_current_waypoint() const;

    /**
     * @brief get_time_to_next_waypoint
     * @return
     */
    double get_time_to_next_waypoint() const;

    /**
     * @brief set_limit_velocity_default
     * @param vel
     */
    void set_limit_velocity_default(const double &vel);

    /**
     * @brief set_approach_velocity_default
     * @param vel
     */
    void set_approach_velocity_default(const double &vel);

private:
    /**
     * Decode a waypoint
     * @param v
     * @param last_time
     * @param depth_offset
     * @return error code
     */
    int decode_waypoint(boost::property_tree::ptree::value_type &v, rclcpp::Time &last_time, const double &depth_offset);

private:
    std::string file_name_ = "mission_empty.xml";
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_ = 0;
    bool is_first_waypoint_ = true;
    bool mission_enable_ = false;
    bool update_mission_ = true;
    rclcpp::Duration duration_next_waypoint_ = rclcpp::Duration::from_seconds(0.);

    std::filesystem::file_time_type file_time_;

    rclcpp::Time time_start_;
    double offset_north_ = 0.0;
    double offset_east_ = 0.0;
    double limit_velocity_default_ = 0.02;
    double approach_velocity_default_ = 1.0;

    double default_time_to_start_ = 60.0;

    rclcpp::Node *n_= nullptr;

    void waypoint_end(seabot2_mission::msg::Waypoint &wp);

    void waypoint_wait_start(seabot2_mission::msg::Waypoint &wp, rclcpp::Time &t_now);

    void waypoint_current(seabot2_mission::msg::Waypoint &wp, rclcpp::Time &t_now);
};

inline bool Mission::is_mission_enable() const{
    return mission_enable_;
}

inline size_t Mission::get_current_waypoint() const{
    return current_waypoint_;
}

inline double Mission::get_time_to_next_waypoint() const{
    return duration_next_waypoint_.seconds();
}

inline void Mission::set_limit_velocity_default(const double &vel){
    limit_velocity_default_ = vel;
}

inline void Mission::set_approach_velocity_default(const double &vel){
    approach_velocity_default_ = vel;
}


#endif // MISSION_H
