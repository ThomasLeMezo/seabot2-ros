#include "seabot2_kalmann/kalmann_node.hpp"
#include <algorithm>    // std::sort

using namespace placeholders;

KalmannNode::KalmannNode()
        : Node("kalmann_node"){

    init_parameters();
    init_interfaces();

    RCLCPP_INFO(this->get_logger(), "[Kalmann_node] Start Ok");
}

void KalmannNode::init_parameters() {

}

void KalmannNode::init_interfaces() {

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmannNode>());
    rclcpp::shutdown();
    return 0;
}