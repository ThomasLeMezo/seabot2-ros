#include "seabot2_safety/safety_node.hpp"
#include "sys/sysinfo.h"

using namespace placeholders;

SafetyNode::SafetyNode()
        : Node("safety_node"){

    RCLCPP_INFO(this->get_logger(), "[Safety_node] Init node safety");
    init_parameters();
    init_interfaces();

    //rclcpp::sleep_for(3s);

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&SafetyNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "[Safety_node] Start Ok");
}

void SafetyNode::init_parameters() {
    this->declare_parameter<int>("loop_dt_", loop_dt_.count());
    loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("dt", loop_dt_.count()));

    this->declare_parameter<double>("internal_humidity_limit", internal_humidity_limit_);
    this->declare_parameter<double>("internal_pressure_limit", internal_pressure_limit_);
    this->declare_parameter<double>("battery_volt_limit", battery_volt_limit_);
    this->declare_parameter<double>("depth_flash_surface", depth_flash_surface_);
    this->declare_parameter<double>("depth_limit_max", depth_limit_max_);
    this->declare_parameter<int>("depth_no_data_warning", depth_no_data_warning_.count());
    this->declare_parameter<int>("battery_no_data_warning", battery_no_data_warning_.count());
    this->declare_parameter<int>("internal_no_data_warning", internal_no_data_warning_.count());
    this->declare_parameter<int>("piston_no_data_warning", piston_no_data_warning_.count());

    internal_humidity_limit_ = this->get_parameter_or("internal_humidity_limit", internal_humidity_limit_);
    internal_pressure_limit_ = this->get_parameter_or("internal_pressure_limit", internal_pressure_limit_);
    battery_volt_limit_ = this->get_parameter_or("battery_volt_limit", battery_volt_limit_);
    depth_flash_surface_ = this->get_parameter_or("depth_flash_surface", depth_flash_surface_);
    depth_limit_max_ = this->get_parameter_or("depth_limit_max", depth_limit_max_);
    depth_no_data_warning_ = std::chrono::milliseconds(this->get_parameter_or("depth_no_data_warning", depth_no_data_warning_.count()));
    battery_no_data_warning_ = std::chrono::milliseconds(this->get_parameter_or("battery_no_data_warning", battery_no_data_warning_.count()));
    internal_no_data_warning_ = std::chrono::milliseconds(this->get_parameter_or("internal_no_data_warning", internal_no_data_warning_.count()));
    piston_no_data_warning_ = std::chrono::milliseconds(this->get_parameter_or("piston_no_data_warning", piston_no_data_warning_.count()));
}

void SafetyNode::depth_callback(const seabot2_depth_filter::msg::DepthPose &msg){
    depth_ = msg.depth;
    velocity_ = msg.velocity;
    depth_last_received_ = msg.header.stamp;
}

void SafetyNode::internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg){
    internal_humidity_ = msg.humidity;
    internal_pressure_ = msg.pressure;
    internal_temperature_ = msg.temperature;
    internal_last_received_ = this->now();
}

void SafetyNode::power_callback(const seabot2_power_driver::msg::PowerState &msg){
    battery_volt_ = msg.battery_volt;
    power_state_ = msg.power_state;
    battery_last_received_ = msg.header.stamp;
}

void SafetyNode::piston_callback(const seabot2_piston_driver::msg::PistonState &msg){
    piston_position_ = msg.position;
    piston_last_received_ = msg.header.stamp;
    piston_state_ = msg.state;
}

void SafetyNode::init_interfaces() {
    publisher_safety_ =  this->create_publisher<seabot2_safety::msg::SafetyStatus>("safety", 1);

    subscriber_depth_data_ = this->create_subscription<seabot2_depth_filter::msg::DepthPose>(
            "/observer/depth", 10, std::bind(&SafetyNode::depth_callback, this, _1));

    subscriber_internal_sensor_filter_ = this->create_subscription<pressure_bme280_driver::msg::Bme280Data>(
            "/observer/pressure_internal", 10, std::bind(&SafetyNode::internal_sensor_callback, this, _1));

    subscriber_power_data_ = this->create_subscription<seabot2_power_driver::msg::PowerState>(
            "/observer/power", 10, std::bind(&SafetyNode::power_callback, this, _1));

    subscriber_piston_data_ = this->create_subscription<seabot2_piston_driver::msg::PistonState>(
            "/driver/piston", 10, std::bind(&SafetyNode::piston_callback, this, _1));

    client_zero_pressure_ = this->create_client<std_srvs::srv::Trigger>("/observer/zero_pressure");

    client_flash_surface_ = this->create_client<std_srvs::srv::SetBool>("/driver/surface");
}

bool SafetyNode::test_depth(){
    bool is_valid = true;

    if(this->now()-depth_last_received_ > depth_no_data_warning_) {
        is_valid = false;
        safety_published_frequency_ &= false;
        safety_depth_limit_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] No depth data received");
    }

    if(depth_ > depth_limit_max_){
        is_valid = false;
        safety_depth_limit_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] Depth limit reached");
    }

    return is_valid;
}

bool SafetyNode::test_internal_data() {
    bool is_valid = true;
    if(internal_humidity_>internal_humidity_limit_ || internal_pressure_>internal_pressure_limit_){
        is_valid &= false;
        safety_depressurization_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] Depressurization detected %f%%, %f mbar", internal_humidity_, internal_pressure_);
    }
    if(this->now()-internal_last_received_ > internal_no_data_warning_){
        is_valid &= false;
        safety_depressurization_ &= false;
        safety_published_frequency_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] No internal data received");
    }
    return is_valid;
}

bool SafetyNode::test_zero_pressure() {
    bool is_valid_conditions = false;
    if(piston_state_ == 4
       && depth_ < max_depth_reset_zero_
       && abs(velocity_) < max_velocity_reset_zero_){
        is_valid_conditions = true;
    }

    switch(reset_depth_status_){
        case IDLE:
            if(is_valid_conditions) {
                reset_depth_status_ = WAIT_RESET;
                depth_reset_time_wait_ = this->now();
            }
            break;
        case WAIT_RESET:
            if(is_valid_conditions
               && (this->now()-depth_reset_time_wait_) > depth_reset_delay_wait_){
                if(call_service_zero_depth() == EXIT_SUCCESS) {
                    is_zero_depth_once_ = true;
                    reset_depth_status_ = IDLE;
                    safety_zero_depth_ = true;
                    RCLCPP_INFO(this->get_logger(), "[Safety_node] Call zero depth");
                }
            }
            else if(!is_valid_conditions){
                reset_depth_status_ = IDLE;
            }
            break;
        default:
            break;
    }

    return is_zero_depth_once_;
}

bool SafetyNode::test_battery(){
    bool is_valid = true;
    if(battery_volt_<battery_volt_limit_){
        is_valid &= false;
        safety_batteries_limit_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] Battery volt limit detected");
    }
    if(this->now()-battery_last_received_ > battery_no_data_warning_){
        is_valid &= false;
        safety_batteries_limit_ &= false;
        safety_published_frequency_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] No battery data received");
    }
    if(power_state_ != static_cast<int>(POWER_STATE_STATUS::POWER_ON)){
        is_valid &= false;
        safety_batteries_limit_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] Power state not ok");
    }
    return is_valid;
}

bool SafetyNode::test_piston(){
    bool is_valid = true;
    if(this->now()-piston_last_received_ > piston_no_data_warning_){
        is_valid &= false;
        safety_published_frequency_ &= false;
        safety_piston_ &= false;
        RCLCPP_WARN(this->get_logger(), "[Safety_node] No piston data received, %f", (this->now()-piston_last_received_).seconds());
    }
    return is_valid;
}

int SafetyNode::call_service_zero_depth(){
    client_zero_pressure_->wait_for_service(500ms);
    if (!client_zero_pressure_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Safety_node] Zero depth service not available");
        return EXIT_FAILURE;
    }
    else {
        if(rclcpp::ok()) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = client_zero_pressure_->async_send_request(request);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Safety_node] rclcpp not ok");
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

int SafetyNode::call_service_flash_surface(const bool &is_surface){
    client_flash_surface_->wait_for_service(500ms);
    if (!client_flash_surface_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "[Safety_node] Flash surface service not available");
        return EXIT_FAILURE;
    }
    else {
        if(rclcpp::ok()) {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = is_surface;
            auto result = client_flash_surface_->async_send_request(request);
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Safety_node] rclcpp not ok");
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

void SafetyNode::flash_surface(){
    if(!flash_surface_enable_ && depth_ < depth_flash_surface_){
        if(call_service_flash_surface(true)==EXIT_SUCCESS)
            flash_surface_enable_ = true;
    }
//    else if(flash_surface_enable_ && depth_ > depth_flash_surface_){
//        if(call_service_flash_surface(false)==EXIT_SUCCESS)
//            flash_surface_enable_ = false;
//    }
}

void SafetyNode::get_ram_cpu(){
    /// CPU Usage (https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process)
    static unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
    double percent;
    FILE* file;
    unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
    fclose(file);

    if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
        totalSys < lastTotalSys || totalIdle < lastTotalIdle){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) + (totalSys - lastTotalSys);
        percent = total;
        total += (totalIdle - lastTotalIdle);
        percent /= total;
        percent *= 100;
    }

    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;

    cpu_ = percent;

    struct sysinfo memInfo;
    sysinfo(&memInfo);
    long long physMemUsed = memInfo.totalram - memInfo.freeram;
    //Multiply in next statement to avoid int overflow on right hand side...
    physMemUsed /= 1000000;
    ram_ = physMemUsed;
}

void SafetyNode::timer_callback() {

    /// ToDo :
    /// * Seafloor

    global_safety_ok_ = true;
    safety_published_frequency_ = true;
    safety_depth_limit_ = true;
    safety_batteries_limit_ = true;
    safety_depressurization_ = true;
    safety_seafloor_ = true;
    safety_piston_ = true;

    global_safety_ok_ &= test_depth();
    global_safety_ok_ &= test_internal_data();
    global_safety_ok_ &= test_piston();
    global_safety_ok_ &= test_zero_pressure();
    global_safety_ok_ &= test_battery();
    flash_surface();
    get_ram_cpu();

    seabot2_safety::msg::SafetyStatus msg;
    msg.global_safety_valid = global_safety_ok_;
    msg.published_frequency = safety_published_frequency_;
    msg.depth_limit = safety_depth_limit_;
    msg.batteries_limit = safety_batteries_limit_;
    msg.depressurization = safety_depressurization_;
    msg.seafloor = safety_seafloor_;
    msg.piston = safety_piston_;
    msg.zero_depth = safety_zero_depth_;
    msg.cpu = cpu_;
    msg.ram = ram_;
    publisher_safety_->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}