#include "seabot2_screen_driver/screen_node.h"
#include <sys/socket.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <unistd.h>

using namespace placeholders;

ScreenNode::ScreenNode()
        : Node("screen_node"), screen_(this){
    get_ip();
    get_hostname();
    init_parameters();
    init_topics();

    screen_.i2c_open();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&ScreenNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Screen_node] Start Ok");
}

void ScreenNode::timer_callback() {

    if(depth_<depth_no_update_) {
        get_ip();

        screen_.write_ip(ip_);
        screen_.write_pressure(round(pressure_)); /// in mbar
        screen_.write_temperature(round(temperature_*10.));
        screen_.write_hygro(round(hygro_));
        screen_.write_voltage(round(voltage_*10.));

        screen_.write_current_waypoint(wp_id_);
        screen_.write_number_waypoints(wp_max_);

        int t_remain = (time_next_wp - this->now()).seconds();
        if(t_remain > 99*60)
            screen_.write_remaining_time(99,59);
        else if(t_remain <0)
            screen_.write_remaining_time(0, 0);
        else
            screen_.write_remaining_time(floor(t_remain / 60.0), t_remain % 60);

        screen_.write_robot_status(status_);

        std::time_t t = std::time(0);   // get time now
        std::tm *now = std::localtime(&t);
        screen_.write_time(now->tm_hour, now->tm_min);

        screen_.write_mission_name(mission_name_);
        screen_.write_robot_name(robot_name_);

        screen_.write_screen();
    }
}

void ScreenNode::init_parameters() {
    /// I2C
    this->declare_parameter<std::string>("i2c_periph", screen_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", screen_.getI2CAddr());
    screen_.setI2CPeriph(this->get_parameter_or("i2c_periph", screen_.getI2CPeriph()));
    screen_.setI2CAddr(this->get_parameter_or("i2c_address", screen_.getI2CAddr()));

    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));

    this->declare_parameter<double>("depth_no_update", depth_no_update_);
    depth_no_update_ = this->get_parameter_or("depth_no_update", depth_no_update_);
}

void ScreenNode::init_topics() {

    subscriber_sensor_internal_ = this->create_subscription<pressure_bme280_driver::msg::Bme280Data>(
            "sensor_internal", 10, std::bind(&ScreenNode::topic_internal_pressure_callback, this, _1));
}

void ScreenNode::get_hostname() {
    char hostname[16];
    gethostname(hostname, 16);
    robot_name_ = hostname;
    RCLCPP_INFO(this->get_logger(), "[Screen_node] Hostname = %s", robot_name_.c_str());
}

#include <cstring>

void ScreenNode::get_ip() {
    /// Reset IP
    for (auto &n:ip_){
        n = 0;
    }

    /// https://stackoverflow.com/questions/2283494/get-ip-address-of-an-interface-on-linux
    struct ifaddrs *ifaddr, *ifa;
    int s;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1)
        return;

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next){
        if (ifa->ifa_addr == nullptr)
            continue;
        s=getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
        if((strcmp(ifa->ifa_name,"wlan0")==0)&&(ifa->ifa_addr->sa_family==AF_INET)){
            if (s != 0) {
                return;
            }

            /// Parse the string to extract ip numbers
            std::istringstream ss(string(host)+".");
            std::string val;
            for(int i=0; i<4; i++) {
                std::getline(ss, val, '.');
                ip_[i] = stoi(val);
            }
        }
    }
    freeifaddrs(ifaddr);
}

void ScreenNode::topic_internal_pressure_callback(const pressure_bme280_driver::msg::Bme280Data &msg) {
    pressure_ = msg.pressure;
    temperature_ = msg.temperature;
    hygro_ = msg.humidity;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScreenNode>());
    rclcpp::shutdown();
    return 0;
}
