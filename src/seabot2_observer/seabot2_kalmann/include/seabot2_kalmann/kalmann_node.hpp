#ifndef BUILD_KALMANN_NODE_HPP
#define BUILD_KALMANN_NODE_HPP

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std;

class KalmannNode : public rclcpp::Node {
public:
    KalmannNode();

private:

    /// Variable

    /// Interfaces

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

private:

};
#endif //BUILD_KALMANN_NODE_HPP
