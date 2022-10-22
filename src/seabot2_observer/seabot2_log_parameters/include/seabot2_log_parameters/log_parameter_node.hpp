#ifndef BUILD_LOG_PARAMETER_NODE_HPP
#define BUILD_LOG_PARAMETER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "seabot2_log_parameters/msg/log_parameter.hpp"

using namespace std::chrono_literals;
using namespace std;

class LogParameterNode : public rclcpp::Node {
public:
    LogParameterNode();

private:

    /// Interfaces
    rclcpp::Publisher<seabot2_log_parameters::msg::LogParameter>::SharedPtr publisher_parameters_;

    /// Functions

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces of this node
     */
    void init_interfaces();

    /**
     * Record all parameters
     */
    void record_parameters();

    /**
     *
     * @param node_name
     * @return
     */
    std::vector<std::string> get_param_list(const std::string &node_name);

    /**
     *
     * @param node_name
     * @param param_name
     */
    void get_param_values(const std::string &node_name,
                                            const std::vector<std::string> &param_name);

private:

};
#endif //BUILD_LOG_PARAMETER_NODE_HPP
