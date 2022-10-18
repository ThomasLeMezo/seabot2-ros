cmake_minimum_required(VERSION 3.8)
project(bluerobotics_ping_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(ping-cpp/src/device)
include_directories(ping-cpp/src/hal)
include_directories(ping-cpp/src/hal/link)
include_directories(ping-cpp/src/hal/link/desktop)
include_directories(ping-cpp/src/time)
include_directories(ping-cpp/src/time/desktop)
include_directories(ping-cpp/src/message)
add_subdirectory(ping-cpp/src/)

#### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Profile.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Excutable
add_executable(bluerobotics_ping_node src/ping_node.cpp)

ament_target_dependencies(bluerobotics_ping_node
        rclcpp
        std_msgs
        std_srvs)

target_link_libraries(bluerobotics_ping_node "${cpp_typesupport_target}"
        DEVICE
        HAL
        PING_MESSAGES
        ${Boost_LIBRARIES})

### Includes

target_include_directories(bluerobotics_ping_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(bluerobotics_ping_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bluerobotics_ping_driver>)

target_include_directories(bluerobotics_ping_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bluerobotics_ping_driver>)

### Install
install(TARGETS bluerobotics_ping_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
