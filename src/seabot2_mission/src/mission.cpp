#include "seabot2_mission/mission.hpp"
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"

#include <utility>

#include <algorithm>

#include "seabot2_mission/waypoint.hpp"

using namespace std;
namespace pt = boost::property_tree;
namespace bt = boost::posix_time;
namespace gt = boost::gregorian;

bool Mission::update_state(const rclcpp::Time &t_now){

    bool is_new_waypoint = false;
    if(waypoints_.empty()){
        duration_next_waypoint_ = rclcpp::Duration::from_seconds(0.0);
        mission_mode_ = WP_IDLE;
        mission_state_ = NO_WP;
    }
    if(current_waypoint_id_ < waypoints_.size()){
        if(t_now < time_start_){ /// Wait before mission start
            duration_next_waypoint_ = time_start_ - t_now;
            mission_state_ = NOT_STARTED;
            mission_mode_ = WP_IDLE;
        }
        else{ /// Mission started
            mission_enable_ = true;
            if(is_first_waypoint_){ /// Tell first waypoint is new
                is_new_waypoint = true;
                is_first_waypoint_ = false;
            }
            while(current_waypoint_id_ < waypoints_.size() &&
                  t_now >= waypoints_[current_waypoint_id_].first->time_end){ /// Test if next waypoint is reached
                is_new_waypoint = true;
                current_waypoint_id_++;
            }

            if(current_waypoint_id_ < waypoints_.size()) { /// Verify end of waypoint list
                duration_next_waypoint_ = waypoints_[current_waypoint_id_].first->time_end - t_now;
                if(is_new_waypoint)
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Mission] Start following waypoint %lu (ending at %f)",
                                current_waypoint_id_, round(waypoints_[current_waypoint_id_].first->time_end.seconds()));

                // Process the current wp
                switch (waypoints_[current_waypoint_id_].second) {
                    case WP_DEPTH:
                        std::dynamic_pointer_cast<WaypointDepth>(waypoints_[current_waypoint_id_].first)->process(t_now);
                        break;
                    case WP_SEAFLOOR_LANDING:
                        break;
                    case WP_TEMPERATURE_KEEPING:
                        std::dynamic_pointer_cast<WaypointTemperatureKeeping>(waypoints_[current_waypoint_id_].first)->process(t_now);
                        break;
                    case WP_TEMPERATURE_PROFILE:
                        std::dynamic_pointer_cast<WaypointTemperatureProfile>(waypoints_[current_waypoint_id_].first)->process(t_now);
                        break;
                    case WP_GNSS_PROFILE:
                        break;
                }
                // ToDo set mission mode !
                mission_state_ = RUNNING;
                mission_mode_ = waypoints_[current_waypoint_id_].second;
            }
            else{
                mission_state_ = ENDING;
                mission_mode_ = WP_IDLE;
            }
        }
    }
    else{ /// Last waypoint was reached
        if(mission_state_==RUNNING)
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Mission] End of waypoints");
        mission_state_ = ENDING;
        duration_next_waypoint_ = waypoints_[waypoints_.size() - 1].first->time_end - t_now;
    }

    if(mission_state_==ENDING || mission_state_==NOT_STARTED){
        idle_state_configuration(t_now);
    }

    return is_new_waypoint;
}

bool Mission::is_current_waypoint_of_type(const WAYPOINT_TYPE &type){
    if(current_waypoint_id_ < waypoints_.size()){
        if(waypoints_[current_waypoint_id_].second == type)
            return true;
        else
            return false;
    }
    else{
        return false;
    }
}

std::shared_ptr<WaypointTemperatureKeeping> Mission::get_current_waypoint_temperature_keeping(){
    if(current_waypoint_id_ < waypoints_.size()){
        if(waypoints_[current_waypoint_id_].second == WP_TEMPERATURE_KEEPING){
            return std::dynamic_pointer_cast<WaypointTemperatureKeeping>(waypoints_[current_waypoint_id_].first);
        }
    }
    return nullptr;
}

void Mission::idle_state_configuration(const rclcpp::Time &t_now){
    dc_msg_.enable_control = true;
    dc_msg_.depth = 0.0;
    dc_msg_.limit_velocity = waypoints_[0].first->velocity;
    dc_msg_.header.stamp = t_now;
}

bool Mission::is_new_mission_file(const std::string &file_xml, const std::string &folder_path){
    try {
        fs::path p1 = folder_path + "/" + file_xml;
        std::filesystem::file_time_type ft = std::filesystem::last_write_time(p1);
        if ((ft.time_since_epoch() - file_time_.time_since_epoch()).count() != 0) {
            file_time_ = ft;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Seabot_Mission] New mission file detected");
            return true;
        } else {
            return false;
        }
    }
    catch (...){
        return false;
    }
}

int Mission::load_mission(const std::string &file_xml, const std::string &folder_path, const rclcpp::Time &start_time_if_not_given){
    if(folder_path.empty())
        file_name_ = file_xml;
    else
        file_name_ = folder_path + "/" + file_xml;
    pt::ptree tree;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Read xml file : %s", file_name_.c_str());
    try {
        pt::read_xml(file_name_, tree);
    } catch (std::exception const&  ex) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] %s", ex.what());
        return EXIT_FAILURE;
    }

    waypoints_.clear();

    try{
        offset_north_ = tree.get_child("mission.offset.north").get_value<double>();
    } catch (std::exception const&  ex){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] No north offset defined %s", ex.what());
    }

    try{
        offset_east_ = tree.get_child("mission.offset.east").get_value<double>();
    } catch (std::exception const&  ex){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] No east offset defined %s", ex.what());
    }

    time_start_ = start_time_if_not_given + rclcpp::Duration::from_seconds(default_time_to_start_);
    // Read special offset time
    try{
        const int year = tree.get_child("mission.offset.start_time_utc.year").get_value<int>();
        const int month = tree.get_child("mission.offset.start_time_utc.month").get_value<int>();
        const int day = tree.get_child("mission.offset.start_time_utc.day").get_value<int>();
        const int hour = tree.get_child("mission.offset.start_time_utc.hour").get_value<int>();
        const int min = tree.get_child("mission.offset.start_time_utc.min").get_value<int>();

        bt::ptime t1(gt::date(year,month,day),bt::time_duration(hour,min,0));
        time_start_ = rclcpp::Time(to_time_t(t1), 0, RCL_ROS_TIME);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Start time = %f", time_start_.seconds());
    } catch (std::exception const&  ex){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] No time offset defined %s - Set now + %f s", ex.what(), default_time_to_start_);
    }

    time_end_ = time_start_;
    current_waypoint_id_ = 0;
    is_first_waypoint_ = true;

    int return_code = EXIT_SUCCESS;
    BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("mission.paths")){
                    return_code &= decode_paths(v, time_end_, 0.0);
                    if(return_code == EXIT_FAILURE)
                        break;
                }
    return return_code;
}

void Mission::decode_waypoint(std::shared_ptr<Waypoint> w, pt::ptree::value_type &v, rclcpp::Time &last_time){
    // Duration
    boost::optional<double> t = v.second.get_optional<double>("duration_since_start");
    boost::optional<double> d = v.second.get_optional<double>("duration");
    if(t.is_initialized()) // End_time
        w->time_end = time_start_ + rclcpp::Duration::from_seconds(t.value());
    else if(d.is_initialized()){ // Duration
        w->time_end = last_time + rclcpp::Duration::from_seconds(d.value());
    }
    else
        throw(std::runtime_error("(No time or duration founded for a waypoint)"));

    // Regulation parameters
    boost::optional<double> vel = v.second.get_optional<double>("limit_velocity");
    if(vel.is_initialized())
        w->velocity = vel.value();
    else
        w->velocity = limit_velocity_default_;

    last_time = w->time_end;
}

void Mission::decode_waypoint_depth(std::shared_ptr<WaypointDepth> w, pt::ptree::value_type &v, const double &depth_offset){
    boost::optional<double> depth = v.second.get_optional<double>("depth");
    if(depth.is_initialized()){
        w->depth = depth.value() + depth_offset;
    }
    else{
        w->depth = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Load Depth Waypoint %zu (t_end=%li, d=%lf, vel=%f)",
                waypoints_.size(),
                (long int)w->time_end.seconds(),
                w->depth,
                w->velocity);
}

void Mission::decode_waypoint_temperature_keeping(const std::shared_ptr<WaypointTemperatureKeeping>& w, pt::ptree::value_type &v){
    boost::optional<double> temperature = v.second.get_optional<double>("temperature");
    if(temperature.is_initialized()){
        w->temperature_ = temperature.value();
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Load Temperature Keeping Waypoint %zu (t_end=%li, temp=%lf, vel=%f)",
                waypoints_.size(),
                (long int)w->time_end.seconds(),
                w->temperature_,
                w->velocity);
}

void Mission::decode_waypoint_temperature_profile(const std::shared_ptr<WaypointTemperatureProfile>& w, pt::ptree::value_type &v){
    boost::optional<double> temperature_high = v.second.get_optional<double>("temperature_high");
    if(temperature_high.is_initialized())
        w->temperature_high_ = temperature_high.value();

    boost::optional<double> temperature_low = v.second.get_optional<double>("temperature_low");
    if(temperature_low.is_initialized())
        w->temperature_low_ = temperature_low.value();

    boost::optional<double> depth_min = v.second.get_optional<double>("depth_min");
    if(depth_min.is_initialized())
        w->depth_min_ = depth_min.value();

    boost::optional<double> depth_max = v.second.get_optional<double>("depth_max");
    if(depth_max.is_initialized())
        w->depth_max_ = depth_max.value();

    boost::optional<double> max_delay = v.second.get_optional<double>("max_delay");
    if(max_delay.is_initialized())
        w->max_delay_ = rclcpp::Duration::from_seconds(max_delay.value());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Load Temperature Profiling Waypoint %zu (t_end=%li, vel=%f)",
                waypoints_.size(),
                (long int)w->time_end.seconds(),
                w->velocity);
}

int Mission::decode_paths(pt::ptree::value_type &v, rclcpp::Time &last_time, const double &depth_offset){
    try{
        if(std::find(std::begin(XML_TYPE), std::end(XML_TYPE), v.first) != std::end(XML_TYPE)){
            if(v.first == XML_DEPTH || v.first == XML_DEPTH_LEGACY){ // "waypoint for leagcy
                std::shared_ptr<WaypointDepth> w = std::make_shared<WaypointDepth>(this);
                decode_waypoint(w, v, last_time);
                decode_waypoint_depth(w, v, depth_offset);
                waypoints_.emplace_back(w, WP_DEPTH);
            }
            else if(v.first == XML_SEAFLOOR_LANDING){

            }
            else if(v.first == XML_TEMPERATURE_KEEPING){
                std::shared_ptr<WaypointTemperatureKeeping> w = std::make_shared<WaypointTemperatureKeeping>(this);
                decode_waypoint(w, v, last_time);
                decode_waypoint_temperature_keeping(w, v);
                waypoints_.emplace_back(w, WP_TEMPERATURE_KEEPING);
            }
            else if(v.first == XML_TEMPERATURE_PROFILE){
                std::shared_ptr<WaypointTemperatureProfile> w = std::make_shared<WaypointTemperatureProfile>(this);
                decode_waypoint(w, v, last_time);
                decode_waypoint_temperature_profile(w, v);
                waypoints_.emplace_back(w, WP_TEMPERATURE_PROFILE);
            }
            else if(v.first == XML_GNSS_PROFILE){

            }
        }
        else if(v.first == "loop"){
            const int nb_loop = v.second.get<int>("<xmlattr>.number", 1);
            const double depth_increment = v.second.get<double>("<xmlattr>.depth_increment", 0.0);

            double depth_offset_tmp = depth_offset;
            for(int i=0; i<nb_loop; i++){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Loop %i/%i", i+1, nb_loop);
                BOOST_FOREACH(pt::ptree::value_type &v_loop,v.second){
                                decode_paths(v_loop, last_time, depth_offset_tmp);
                            }
                depth_offset_tmp += depth_increment;
            }
        }
    }
    catch(std::exception const&  ex) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"[Seabot_Mission] Wrong xml file %s", ex.what());
        return EXIT_FAILURE;
    }


//            // North & East position
//            boost::optional<double> north_local = v.second.get_optional<double>("north");
//            boost::optional<double> east_local = v.second.get_optional<double>("east");
//            if(north_local.is_initialized() && east_local.is_initialized()){
//                w.north = north_local.value();
//                w.east = north_local.value();
//            }
//            else{
//                w.enable_thrusters = false;
//            }

    return EXIT_SUCCESS;
}

vector<float> Mission::get_velocity_list(){
    vector<float> velocity_list;
    for(auto & waypoint : waypoints_){
        if(waypoint.second!=WP_GNSS_PROFILE) {
            bool found = (find(velocity_list.begin(), velocity_list.end(), (float) waypoint.first->velocity) !=
                          velocity_list.end());
            if (!found)
                velocity_list.push_back(waypoint.first->velocity);
        }
    }
    return velocity_list;
}

