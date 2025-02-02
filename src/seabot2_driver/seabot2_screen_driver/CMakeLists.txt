cmake_minimum_required(VERSION 3.8)
project(seabot2_screen_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Executable
add_executable(screen_node
        src/screen_node.cpp
        src/screen.cpp
)

ament_target_dependencies(screen_node
        rclcpp
        seabot2_msgs
)

target_link_libraries(screen_node
        i2c
)

### Includes
target_include_directories(screen_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS screen_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
