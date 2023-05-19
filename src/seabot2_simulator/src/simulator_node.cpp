#include "seabot2_simulator/simulator_node.hpp"

using namespace std::placeholders;

SimulatorNode::SimulatorNode()
        : Node("simulator_node"), s_(rclcpp::Time(0., RCL_ROS_TIME)){

    RCLCPP_INFO(this->get_logger(), "[Simulator_node] Init node simulation");
    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
            loop_dt_, std::bind(&SimulatorNode::timer_callback, this));

    s_.run_simulation();

    cout << "position piston" << '\t' << s_.x_(0)*s_.rad_to_tick_ << endl;
    cout << "piston velocity" << '\t' << s_.x_(1)*s_.rad_to_tick_ << endl;
    cout << "piston current" << '\t' << s_.x_(2) << endl;
    cout << "velocity" << '\t' << s_.x_(3) << endl;
    cout << "depth" << '\t' << s_.x_(4) << endl;
//    cout << "depth_kalman" << '\t' << s_.memory_kalman_depth[s_.memory_kalman_depth.size()-1] << endl;
    cout << "time end = " << s_.t_.seconds() << endl;

    //cout << s_.x_ << endl;
    cout << s_.motor_cmd_ << endl;
    cout << "steps = " << s_.nb_steps << endl;
    cout << "time end = " << (s_.t_-s_.start_time_).seconds() << endl;

    cout << "piston position = " << s_.piston_position_ << endl;
    cout << "piston switchs = " << s_.switch_bottom_ << '\t' << s_.switch_top_ << endl;
    cout << "piston set_point = " << s_.dc_.piston_set_point_ << endl;

    RCLCPP_INFO(this->get_logger(), "[Simulator_node] Simulation ended");
    exit(EXIT_SUCCESS);
}

void SimulatorNode::init_parameters() {
}

void SimulatorNode::init_interfaces() {

}

void SimulatorNode::timer_callback() {

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorNode>());
    rclcpp::shutdown();

    return 0;
}