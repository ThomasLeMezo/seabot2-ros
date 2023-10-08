#ifndef MISSION_H
#define MISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include <boost/property_tree/ptree.hpp>
#include <rclcpp/rclcpp.hpp>
#include "seabot2_mission/msg/depth_control_set_point.hpp"
#include "seabot2_mission/msg/mission_state.hpp"

#include "seabot2_mission/waypoint.hpp"

#include <filesystem>
namespace fs = std::filesystem;

class Waypoint;
class WaypointDepth;
class WaypointSeafloorLanding;
class WaypointTemperatureKeeping;
class WaypointTemperatureProfile;
class WaypointGnssProfile;


class Mission
{
public:
    /**
       * @brief Mission
       */
    Mission()= default;

    /**
       * @brief
       */
    bool update_state(const rclcpp::Time &t_now);

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
    int load_mission(const std::string &file_xml, const std::string &folder_path="",
                     const rclcpp::Time &start_time_if_not_given=rclcpp::Time(0., RCL_STEADY_TIME));

    /**
     * @brief is_mission_enable
     * @return
     */
    [[nodiscard]] bool is_mission_enable() const;

    /**
     * @brief get_velocity_list
     * @return
     */
    std::vector<float> get_velocity_list();

    /**
     * @brief get_start_time
     * @return
     */
    [[nodiscard]] rclcpp::Time get_start_time() const{
        return time_start_;
    }

    /**
     * @brief get_end_time
     * @return
     */
    [[nodiscard]] rclcpp::Time get_end_time() const{
        return time_end_;
    }

    /**
     * @brief get_number_waypoints
     */
    [[nodiscard]] size_t get_number_waypoints() const{
        return waypoints_.size();
    }

    /**
     * @brief get_current_waypoint_id
     * @return
     */
    [[nodiscard]]  size_t get_current_waypoint_id() const{
        return current_waypoint_id_;
    }

    /**
     * @brief get_time_to_next_waypoint
     * @return
     */
    [[nodiscard]] double get_time_to_next_waypoint() const{
        return duration_next_waypoint_.seconds();
    }

    /**
     * @brief get_time_to_next_waypoint
     * @param vel
     */
    void set_limit_velocity_default(const double &vel){
        limit_velocity_default_ = vel;
    }

    /**
     * @brief get_mission_mode
     * @return
     */
    [[nodiscard]] unsigned int get_mission_mode() const{
        return mission_mode_;
    }

    /**
     * @brief update depth
     * @param depth in meter
     */
    void update_depth(const double &depth){
        depth_ = depth;
    }

    /**
     * @brief update_temperature
     * @param temperature in degree
     */
    void update_temperature(const double &temperature){
        temeprature_ = temperature;
    }

    /**
     * @brief get_depth_control_set_point
     * @return
     */
    seabot2_mission::msg::DepthControlSetPoint& get_depth_control_set_point(){
        return dc_msg_;
    }

    /**
     * Set set point parameters to idle state
     */
    void idle_state_configuration(const rclcpp::Time &t_now);

private:
    /**
     *
     * @param w
     * @param v
     * @param last_time
     * @return
     */
    void decode_waypoint(std::shared_ptr<Waypoint> w, boost::property_tree::ptree::value_type &v, rclcpp::Time &last_time);

    /**
     * Decode a depth waypoint
     * @param v
     * @param last_time
     * @param depth_offset
     * @return
     */
    void decode_waypoint_depth(std::shared_ptr<WaypointDepth> w, boost::property_tree::ptree::value_type &v, const double &depth_offset);

    /**
     *
     * @param v
     * @param last_time
     * @param depth_offset
     * @return
     */
    int decode_paths(boost::property_tree::ptree::value_type &v, rclcpp::Time &last_time, const double &depth_offset);

private:
    std::string file_name_ = "mission_empty.xml";

    enum WAYPOINT_TYPE:unsigned int {WP_DEPTH=0,
        WP_SEAFLOOR_LANDING=1,
        WP_TEMPERATURE_KEEPING=2,
        WP_TEMPERATURE_PROFILE=3,
        WP_GNSS_PROFILE=4};
    std::vector<std::pair<std::shared_ptr<Waypoint>, WAYPOINT_TYPE>> waypoints_;

    size_t current_waypoint_id_ = 0;
    bool is_first_waypoint_ = true;
    bool mission_enable_ = false;
    rclcpp::Duration duration_next_waypoint_ = rclcpp::Duration::from_seconds(0.);

    std::filesystem::file_time_type file_time_;

    rclcpp::Time time_start_ = rclcpp::Time(0., RCL_STEADY_TIME);
    rclcpp::Time time_end_ = rclcpp::Time(0., RCL_STEADY_TIME);
    double offset_north_ = 0.0;
    double offset_east_ = 0.0;
    double limit_velocity_default_ = 0.02;

    double default_time_to_start_ = 60.0;

    // Mission mode
    unsigned int mission_mode_ = seabot2_mission::msg::MissionState::MODE_IDLE;

    // Mission state
    enum MISSION_STATE:unsigned int {NOT_STARTED=0, RUNNING=1, ENDING=2};
    MISSION_STATE mission_state_ = NOT_STARTED;

    // Control messages
    seabot2_mission::msg::DepthControlSetPoint dc_msg_;

    // Waypoint type
    const std::string XML_DEPTH = "waypoint_depth";
    const std::string XML_DEPTH_LEGACY = "waypoint";
    const std::string XML_SEAFLOOR_LANDING =   "seafloor_landing";
    const std::string XML_TEMPERATURE_KEEPING = "temperature_keeping";
    const std::string XML_TEMPERATURE_PROFILE = "temperature_profile";
    const std::string XML_GNSS_PROFILE =   "gnss_profile";
    const std::vector<std::string> XML_TYPE = {XML_DEPTH,
                                              XML_DEPTH_LEGACY,
                                              XML_SEAFLOOR_LANDING,
                                              XML_TEMPERATURE_KEEPING,
                                              XML_TEMPERATURE_PROFILE,
                                              XML_GNSS_PROFILE};

    // State
    double depth_ = 0.0;
    double temeprature_ = 15.0; // in degree

};


#endif // MISSION_H
