cmake_minimum_required(VERSION 3.8)
project(seabot2_thruster_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

### Executable
add_executable(thruster_node
        src/thruster_node.cpp
        src/thruster.cpp
)

ament_target_dependencies(thruster_node
        rclcpp
        seabot2_msgs
        geometry_msgs
)
target_link_libraries(thruster_node
        i2c
)

### Includes
target_include_directories(thruster_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(thruster_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

### Install
install(TARGETS thruster_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
