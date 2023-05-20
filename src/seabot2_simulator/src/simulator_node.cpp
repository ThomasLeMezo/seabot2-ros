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