cmake_minimum_required(VERSION 3.8)
project(seabot2_wtf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Curses REQUIRED)

### Executable

add_executable(wtf_node
        src/wtf_node.cpp
)

include_directories(${CURSES_INCLUDE_DIR})

ament_target_dependencies(wtf_node
        rclcpp
        seabot2_msgs
        rcl_interfaces
        )

target_link_libraries(wtf_node
        "${cpp_typesupport_target}"
        ${CURSES_LIBRARIES}
)

### Includes

target_include_directories(wtf_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install

install(TARGETS wtf_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
