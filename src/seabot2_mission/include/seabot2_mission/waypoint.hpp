//
// Created by lemezoth on 08/10/23.
//

#ifndef BUILD_WAYPOINT_HPP
#define BUILD_WAYPOINT_HPP

#include <string>
#include <vector>
#include <array>
#include <rclcpp/rclcpp.hpp>

#include "seabot2_mission/msg/depth_control_set_point.hpp"
#include "seabot2_mission/msg/mission_state.hpp"
#include "seabot2_mission/waypoint.hpp"
#include "seabot2_mission/mission.hpp"

using namespace std::chrono_literals;
class Mission;

class Waypoint{

public:
    explicit Waypoint(Mission *mission):time_end(0., RCL_STEADY_TIME){
        mission_ = mission;
    }

    virtual ~Waypoint()= default;

    virtual void process(const rclcpp::Time &time) = 0;

public:
    double velocity = 0.0;
    unsigned int mode = seabot2_mission::msg::MissionState::MODE_IDLE;
    rclcpp::Time time_end;
    Mission *mission_ = nullptr;
};

class WaypointDepth: public Waypoint{
public:
    explicit WaypointDepth(Mission *mission):Waypoint(mission){}
    ~WaypointDepth() override = default;

    /**
     * @brief process
     */
    void process(const rclcpp::Time &time) override;

public:
    double depth = 0.0;
};

class WaypointSeafloorLanding: public Waypoint{
public:
    explicit WaypointSeafloorLanding(Mission *mission):Waypoint(mission){}
    ~WaypointSeafloorLanding() override = default;

    /**
     * @brief process
     */
    void process(const rclcpp::Time &time) override;

public:
};

class WaypointTemperatureKeeping: public Waypoint{
public:
    explicit WaypointTemperatureKeeping(Mission *mission):Waypoint(mission){}
    ~WaypointTemperatureKeeping() override = default;

    /**
     * @brief process
     */
    void process(const rclcpp::Time &time) override;

public:
    double temperature_ = 20.0;
    double depth_set_point_accumulator_ = 0.0;
    double coeff_K_ = 0.03;
    double error_temperature_ = 0.0;
};

class WaypointTemperatureProfile: public Waypoint{
public:
    explicit WaypointTemperatureProfile(Mission *mission):Waypoint(mission){}
    ~WaypointTemperatureProfile() override = default;

    /**
     * @brief process
     */
    void process(const rclcpp::Time &time) override;

public:
    double temperature_high_ = 20.0;
    double temperature_low_ = 20.0;
    double depth_min_ = 0.0;
    double depth_max_ = 0.0;
    rclcpp::Duration max_delay_ = 100s;
private:
    rclcpp::Time time_last_transition_;
    enum PROFILE_STATE{PROFILE_GO_DOWN, PROFILE_GO_UP};
    PROFILE_STATE state_ = PROFILE_GO_DOWN;
    bool first_init_ = true;
};

class WaypointGNSSProfile: public Waypoint{
public:
    explicit WaypointGNSSProfile(Mission *mission):Waypoint(mission){}

    ~WaypointGNSSProfile() override = default;

    /**
     * @brief process
     */
    void process(const rclcpp::Time &time) override;

public:
    double north = 0.0;
    double east = 0.0;
};


#endif //BUILD_WAYPOINT_HPP
