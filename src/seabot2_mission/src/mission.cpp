#include "seabot2_mission/mission.hpp"
#include <fstream>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"

#include <ctime>
#include <utility>

using namespace std;
namespace pt = boost::property_tree;
namespace bt = boost::posix_time;
namespace gt = boost::gregorian;

Mission::Mission(rclcpp::Node *n){
    n_ = n;
}

bool Mission::compute_command(seabot2_mission::msg::Waypoint &wp){

    bool is_new_waypoint = false;
    rclcpp::Time t_now = n_->now();
    if(current_waypoint_ < waypoints_.size()){
        if(t_now < time_start_){ /// Wait before mission start
            waypoint_wait_start(wp, t_now);
        }
        else{ /// Mission started
            mission_enable_ = true;
            if(is_first_waypoint_){ /// Tell first waypoint is new
                is_new_waypoint = true;
                is_first_waypoint_ = false;
            }
            while(current_waypoint_ < waypoints_.size() &&
                  t_now >= waypoints_[current_waypoint_].time_end){ /// Test if next waypoint is reached
                is_new_waypoint = true;
                current_waypoint_++;
            }

            if(current_waypoint_ < waypoints_.size()) { /// Verify end of waypoint list
                waypoint_current(wp, t_now);
                if(is_new_waypoint)
                    RCLCPP_INFO(n_->get_logger(), "[Mission] Start following waypoint %lu (ending at %f)",
                                current_waypoint_, round(waypoints_[current_waypoint_].time_end.seconds()));
            }
            else
                waypoint_end(wp, t_now);
        }
    }
    else{ /// Last waypoint was reached
        waypoint_end(wp, t_now);
    }

    wp.mission_enable = mission_enable_;
    wp.waypoint_id = current_waypoint_;
    wp.waypoint_length = waypoints_.size();
    wp.time_to_next_waypoint = duration_next_waypoint_.seconds();

    return is_new_waypoint;
}

void Mission::waypoint_current(seabot2_mission::msg::Waypoint &wp, rclcpp::Time &t_now){
    wp.depth = waypoints_[current_waypoint_].depth;

    wp.limit_velocity = waypoints_[current_waypoint_].limit_velocity;
    wp.enable_thrusters = waypoints_[current_waypoint_].enable_thrusters;
    wp.seafloor_landing = waypoints_[current_waypoint_].seafloor_landing;

    /// Compute the desired (north, east) set point
    rclcpp::Time t1;
    if(current_waypoint_ == 0) /// Case first waypoint
        t1 = time_start_;
    else /// After the first waypoint
        t1 = waypoints_[current_waypoint_ - 1].time_end;
    rclcpp::Time t2 = waypoints_[current_waypoint_].time_end;
    double ratio = (t_now-t1).seconds()/(t2-t1).seconds(); // Should be between [0, 1]

    if(current_waypoint_ >= 1){
        double d_north = waypoints_[current_waypoint_].north - waypoints_[current_waypoint_ - 1].north;
        double d_east = waypoints_[current_waypoint_].east - waypoints_[current_waypoint_ - 1].east;

        wp.north = waypoints_[current_waypoint_ - 1].north + d_north * ratio + offset_north_;;
        wp.east = waypoints_[current_waypoint_ - 1].east + d_east * ratio + offset_east_;
    }
    else{ /// Case first waypoint
        wp.north = waypoints_[current_waypoint_].north + offset_north_;;
        wp.east = waypoints_[current_waypoint_].east + offset_east_;
    }
    duration_next_waypoint_ = waypoints_[current_waypoint_].time_end - t_now;
}

void Mission::waypoint_end(seabot2_mission::msg::Waypoint &wp, rclcpp::Time &t_now){
    if(mission_enable_)
        RCLCPP_INFO(n_->get_logger(),"[Mission] End of waypoints");
    mission_enable_ = false;

    wp.depth = 0.0;
    wp.limit_velocity = waypoints_[waypoints_.size() - 1].limit_velocity;
    wp.north = waypoints_[waypoints_.size() - 1].north + offset_north_;
    wp.east = waypoints_[waypoints_.size() - 1].east + offset_east_;
    wp.enable_thrusters = false;
    wp.seafloor_landing = false;

    duration_next_waypoint_ = waypoints_[waypoints_.size() - 1].time_end - t_now;
}

void Mission::waypoint_wait_start(seabot2_mission::msg::Waypoint &wp, rclcpp::Time &t_now){
    mission_enable_ = false;
    wp.depth = 0.0;
    wp.north = waypoints_[current_waypoint_].north + offset_north_;
    wp.east = waypoints_[current_waypoint_].east + offset_east_;

    wp.limit_velocity = waypoints_[current_waypoint_].limit_velocity;
    wp.enable_thrusters = waypoints_[current_waypoint_].enable_thrusters;
    wp.seafloor_landing = waypoints_[current_waypoint_].seafloor_landing;
    duration_next_waypoint_ = time_start_ - t_now;
}

bool Mission::is_new_mission_file(const std::string &file_xml, const std::string &folder_path){
    try {
        fs::path p1 = folder_path + "/" + file_xml;
        std::filesystem::file_time_type ft = std::filesystem::last_write_time(p1);
        if ((ft.time_since_epoch() - file_time_.time_since_epoch()).count() != 0) {
            file_time_ = ft;
            RCLCPP_INFO(n_->get_logger(), "[Seabot_Mission] New mission file detected");
            return true;
        } else {
            return false;
        }
    }
    catch (...){
        return false;
    }
}

int Mission::load_mission(const std::string &file_xml, const std::string &folder_path){
    if(folder_path.empty())
        file_name_ = file_xml;
    else
        file_name_ = folder_path + "/" + file_xml;
    pt::ptree tree;
    RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] Read xml file : %s", file_name_.c_str());
    try {
        pt::read_xml(file_name_, tree);
    } catch (std::exception const&  ex) {
        RCLCPP_FATAL(n_->get_logger(),"[Seabot_Mission] %s", ex.what());
        return EXIT_FAILURE;
    }

    waypoints_.clear();

    try{
        offset_north_ = tree.get_child("mission.offset.north").get_value<double>();
    } catch (std::exception const&  ex){
        RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] No north offset defined %s", ex.what());
    }

    try{
        offset_east_ = tree.get_child("mission.offset.east").get_value<double>();
    } catch (std::exception const&  ex){
        RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] No east offset defined %s", ex.what());
    }

    time_start_ = n_->now() + rclcpp::Duration::from_seconds(default_time_to_start_);
    // Read special offset time
    try{
        const int year = tree.get_child("mission.offset.start_time_utc.year").get_value<int>();
        const int month = tree.get_child("mission.offset.start_time_utc.month").get_value<int>();
        const int day = tree.get_child("mission.offset.start_time_utc.day").get_value<int>();
        const int hour = tree.get_child("mission.offset.start_time_utc.hour").get_value<int>();
        const int min = tree.get_child("mission.offset.start_time_utc.min").get_value<int>();

        bt::ptime t1(gt::date(year,month,day),bt::time_duration(hour,min,0));
        time_start_ = rclcpp::Time(to_time_t(t1), 0, RCL_ROS_TIME);

        RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] Start time = %f", time_start_.seconds());
    } catch (std::exception const&  ex){
        RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] No time offset defined %s - Set now + %f s", ex.what(), default_time_to_start_);
    }

    rclcpp::Time last_time = time_start_;
    current_waypoint_ = 0;
    is_first_waypoint_ = true;

    int return_code = EXIT_SUCCESS;
    BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("mission.paths")){
                    return_code &= decode_waypoint(v, last_time, 0.0);
                    if(return_code == EXIT_FAILURE)
                        break;
                }
    return return_code;
}

int Mission::decode_waypoint(pt::ptree::value_type &v, rclcpp::Time &last_time, const double &depth_offset){
    if(v.first == "waypoint"){
        Waypoint w;
        try{

            // North & East position
            boost::optional<double> north_local = v.second.get_optional<double>("north");
            boost::optional<double> east_local = v.second.get_optional<double>("east");
            if(north_local.is_initialized() && east_local.is_initialized()){
                w.north = north_local.value();
                w.east = north_local.value();
            }
            else{
                w.enable_thrusters = false;
            }

            boost::optional<double> depth = v.second.get_optional<double>("depth");
            if(depth.is_initialized()){
                w.depth = depth.value() + depth_offset;
            }
            else{
                w.depth = 0.0;
            }

            boost::optional<bool> landing = v.second.get_optional<bool>("seafloor_landing");
            if(landing.is_initialized()){
                w.seafloor_landing = landing.value();
            }
            else{
                w.seafloor_landing = false;
            }


            // Duration
            boost::optional<double> t = v.second.get_optional<double>("duration_since_start");
            boost::optional<double> d = v.second.get_optional<double>("duration");
            if(t.is_initialized()) // End_time
                w.time_end = time_start_ + rclcpp::Duration::from_seconds(t.value());
            else if(d.is_initialized()){ // Duration
                w.time_end = last_time + rclcpp::Duration::from_seconds(d.value());
            }
            else
                throw(std::runtime_error("(No time or duration founded for a waypoint)"));

            // Regulation parameters
            boost::optional<double> vel = v.second.get_optional<double>("limit_velocity");
            if(vel.is_initialized())
                w.limit_velocity = vel.value();
            else
                w.limit_velocity = limit_velocity_default_;

            boost::optional<bool> enable_thrusters = v.second.get_optional<bool>("enable_thrusters");
            if(enable_thrusters.is_initialized())
                w.enable_thrusters = enable_thrusters.value();
        }
        catch(std::exception const&  ex) {
            RCLCPP_FATAL(n_->get_logger(),"[Seabot_Mission] Wrong xml file %s", ex.what());
            return EXIT_FAILURE;
        }
        last_time = w.time_end;
        waypoints_.push_back(w);

        RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] Load Waypoint %zu (t_end=%li, d=%lf, E=%lf, N=%lf, vel=%f)", waypoints_.size(), (long int)w.time_end.seconds(), w.depth, w.east, w.north, w.limit_velocity);
    }
    else if(v.first == "loop"){
        const int nb_loop = v.second.get<int>("<xmlattr>.number", 1);
        const double depth_increment = v.second.get<double>("<xmlattr>.depth_increment", 0.0);

        double depth_offset_tmp = depth_offset;
        for(int i=0; i<nb_loop; i++){
            RCLCPP_INFO(n_->get_logger(),"[Seabot_Mission] Loop %i/%i", i+1, nb_loop);
            BOOST_FOREACH(pt::ptree::value_type &v_loop,v.second){
                            decode_waypoint(v_loop, last_time, depth_offset_tmp);
                        }
            depth_offset_tmp += depth_increment;
        }
    }
    return EXIT_SUCCESS;
}

vector<float> Mission::get_velocity_list(){
    vector<float> velocity_list;
    for(auto & waypoint : waypoints_){
        bool found = (find(velocity_list.begin(), velocity_list.end(), (float)waypoint.limit_velocity) != velocity_list.end());
        if(!found)
            velocity_list.push_back(waypoint.limit_velocity);
    }
    return velocity_list;
}

