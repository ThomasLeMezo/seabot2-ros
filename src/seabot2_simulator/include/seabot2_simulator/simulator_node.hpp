#ifndef BUILD_SIMULATOR_NODE_HPP
#define BUILD_SIMULATOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "simulator.h"

using namespace std::chrono_literals;
using namespace std;

class SimulatorNode : public rclcpp::Node {
public:
    SimulatorNode();

private:
    /// Variable
    Simulator s_;

    /// Interfaces

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

};
#endif //BUILD_SIMULATOR_NODE_HPP
