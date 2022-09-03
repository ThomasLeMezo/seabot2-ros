#include "seabot2_thruster_driver/thruster_node.h"

using namespace placeholders;

ThrusterNode::ThrusterNode()
        : Node("thruster_node"), thruster_(this){

    init_parameters();
    init_topics();

    thruster_.i2c_open();

    timer_ = this->create_wall_timer(
            200ms, std::bind(&ThrusterNode::timer_callback, this));

    last_regulation_time_ = this->now();
    manual_velocity_time_last_ = this->now();
    velocity_time_last = this->now();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds >(delay_stop_));
    RCLCPP_INFO(this->get_logger(), "[Thruster_node] Start Ok");
}

void ThrusterNode::timer_callback() {
    /// Measure difference of time
    rclcpp::Time new_regulation_time = this->now();
    double dt = (new_regulation_time - last_regulation_time_).seconds();
    last_regulation_time_ = new_regulation_time;

    double max_pwm_dt_change = max_velocity_pwm_ * dt;
    /// Choose input source
    double linear, angular;
    if((new_regulation_time - manual_velocity_time_last_)<delay_stop_){ /// Case manual cmd send in the last delay_stop_
        linear = manual_velocity_linear_;
        angular = manual_velocity_angular_;
    }
    else if((new_regulation_time - velocity_time_last)<delay_stop_){
        linear = velocity_linear_;
        angular = velocity_angular_;
    }
    else{
        linear = 0.0;
        angular = 0.0;
    }
    /// Thrusters allocation
    double left, right;
    left = linear + angular;
    right = linear - angular;

    if(!allow_backward_){ /// Remove backward propulsion if defined
        left = std::max(0., left);
        right = std::max(0., right);
    }

    /// Compute the translation to pwm
    uint8_t cmd_left = convert_to_pwm(left);
    uint8_t cmd_right = convert_to_pwm(right);

    /// Reverse cmd if defined in parameters
    if(reverse_left_)
        cmd_left = invert_cmd(cmd_left);
    if(reverse_right_)
        cmd_right = invert_cmd(cmd_right);

    if(cmd_left!=cmd_left_last_ || cmd_right!=cmd_right_last_){
        /// Saturate the velocity
        double delta_right = cmd_right - cmd_right_last_;
        double delta_left = cmd_left - cmd_left_last_;
        delta_right = clamp(delta_right, -max_pwm_dt_change, max_pwm_dt_change);
        delta_left = clamp(delta_left, -max_pwm_dt_change, max_pwm_dt_change);

        cmd_right = static_cast<int>(cmd_right_last_) + static_cast<int>(round(delta_right));
        cmd_left = static_cast<int>(cmd_left_last_) + static_cast<int>(round(delta_left));

        /// Write values to the microcontroller
        if(thruster_.write_cmd(cmd_left, cmd_right)==0){
            /// Save previous values
            cmd_left_last_ = cmd_left;
            cmd_right_last_ = cmd_right;

            /// Publish cmd for log
            seabot2_thruster_driver::msg::Engine msg;
            msg.left = cmd_left;
            msg.right = cmd_right;
            publisher_engine_->publish(msg);
        }
    }
}

uint8_t ThrusterNode::convert_to_pwm(const double &u) const{
    int cmd = static_cast<int>(round(u*coeff_cmd_to_pwm_ + MOTOR_PWM_STOP));
    return std::clamp(cmd, MIN_PWM, MAX_PWM);
}

void ThrusterNode::init_parameters() {
    this->declare_parameter<long>("loop_dt", loop_dt_.count());
    this->declare_parameter<double>("coeff_cmd_to_pwm", coeff_cmd_to_pwm_);
    this->declare_parameter<int>("delay_stop", delay_stop_.count());
    this->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
    this->declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    this->declare_parameter<double>("max_velocity_pwm", max_velocity_pwm_);
    this->declare_parameter<bool>("allow_backward", allow_backward_);
    this->declare_parameter<bool>("reverse_thruster_order", reverse_thruster_order_);
    this->declare_parameter<bool>("reverse_left", reverse_left_);
    this->declare_parameter<bool>("reverse_right", reverse_right_);

    loop_dt_ = std::chrono::milliseconds (this->get_parameter_or("loop_dt", loop_dt_.count()));
    this->get_parameter("coeff_cmd_to_pwm", coeff_cmd_to_pwm_);
    delay_stop_ = std::chrono::milliseconds (this->get_parameter_or("delay_stop", delay_stop_.count()));
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_velocity_pwm", max_velocity_pwm_);
    this->get_parameter("allow_backward", allow_backward_);
    this->get_parameter("reverse_thruster_order", reverse_thruster_order_);
    this->get_parameter("reverse_left", reverse_left_);
    this->get_parameter("reverse_right", reverse_right_);

    /// I2C
    this->declare_parameter<std::string>("i2c_periph", thruster_.getI2CPeriph());
    this->declare_parameter<int>("i2c_address", thruster_.getI2CAddr());

    thruster_.setI2CPeriph(this->get_parameter_or("i2c_periph", thruster_.getI2CPeriph()));
    thruster_.setI2CAddr(this->get_parameter_or("i2c_address", thruster_.getI2CAddr()));
}

void ThrusterNode::init_topics() {
    publisher_engine_ = this->create_publisher<seabot2_thruster_driver::msg::Engine>("engine", 1);

    subscription_velocity_ = this->create_subscription<seabot2_thruster_driver::msg::Velocity>(
            "cmd_engine", 10, std::bind(&ThrusterNode::topic_velocity_callback, this, _1));
    subscription_manual_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ThrusterNode::topic_manual_velocity_callback, this, _1));
}

void ThrusterNode::topic_velocity_callback(const seabot2_thruster_driver::msg::Velocity &msg) {
    velocity_linear_ = std::clamp(static_cast<double>(msg.linear), -max_linear_velocity_, max_linear_velocity_);
    velocity_angular_ = std::clamp(static_cast<double>(msg.angular), -max_linear_velocity_, max_linear_velocity_);
    velocity_time_last = this->get_clock()->now();
}

void ThrusterNode::topic_manual_velocity_callback(const geometry_msgs::msg::Twist &msg) {
    manual_velocity_linear_ = std::clamp(msg.linear.x, -max_linear_velocity_, max_linear_velocity_);
    manual_velocity_angular_ = std::clamp(msg.angular.z, -max_linear_velocity_, max_linear_velocity_);
    manual_velocity_time_last_ = this->get_clock()->now();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterNode>());
    rclcpp::shutdown();
    return 0;
}
