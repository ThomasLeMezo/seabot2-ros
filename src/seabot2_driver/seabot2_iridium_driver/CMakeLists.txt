cmake_minimum_required(VERSION 3.8)
project(seabot2_iridium_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenMP REQUIRED)

find_package(seabot2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

### Executable
add_executable(iridium_node
        src/iridium_node.cpp
        src/sbd/log_data.cpp
        src/sbd/sbd.cpp
        src/serial/AsyncSerial.cpp
        src/serial/BufferedAsyncSerial.cpp
        src/sbd/mission_xml.cpp
        )

ament_target_dependencies(iridium_node
        rclcpp
        seabot2_msgs
        std_msgs
        )

target_link_libraries(iridium_node
        "${cpp_typesupport_target}"
        OpenMP::OpenMP_CXX
        )

### Includes
target_include_directories(iridium_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serial>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sbd>
        $<INSTALL_INTERFACE:include>
        )

### Install
install(TARGETS iridium_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
