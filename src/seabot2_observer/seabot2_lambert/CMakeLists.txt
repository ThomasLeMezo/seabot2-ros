cmake_minimum_required(VERSION 3.8)
project(seabot2_lambert)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Executable
add_executable(lambert_node src/lambert_node.cpp)

ament_target_dependencies(lambert_node
        rclcpp
        seabot2_msgs
)

target_link_libraries(lambert_node
        "${cpp_typesupport_target}"
        proj
)

### Includes
target_include_directories(lambert_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS lambert_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
