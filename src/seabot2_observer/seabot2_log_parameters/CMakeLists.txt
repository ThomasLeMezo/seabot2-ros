cmake_minimum_required(VERSION 3.8)
project(seabot2_log_parameters)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

### Executable
add_executable(log_parameter_node
        src/log_parameter_node.cpp
)

ament_target_dependencies(log_parameter_node
        rclcpp
        seabot2_msgs
        std_srvs
)

### Includes
target_include_directories(log_parameter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS log_parameter_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
