cmake_minimum_required(VERSION 3.8)
project(seabot2_temperature_filter)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Executable
add_executable(filter_temperature_node
        src/filter_temperature_node.cpp
)

ament_target_dependencies(filter_temperature_node
        rclcpp
        seabot2_msgs
)

target_link_libraries(filter_temperature_node
        "${cpp_typesupport_target}"
)

### Includes
target_include_directories(filter_temperature_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS filter_temperature_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
