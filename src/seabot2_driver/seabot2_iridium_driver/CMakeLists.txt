cmake_minimum_required(VERSION 3.8)
project(seabot2_iridium_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenMP REQUIRED)

find_package(seabot2_depth_filter REQUIRED)
find_package(pressure_bme280_driver REQUIRED)
find_package(seabot2_power_driver REQUIRED)
find_package(seabot2_piston_driver REQUIRED)
find_package(seabot2_mission REQUIRED)
find_package(gpsd_client REQUIRED)
find_package(seabot2_safety REQUIRED)
find_package(seabot2_lambert REQUIRED)
find_package(std_msgs REQUIRED)

### Interfaces
### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/IridiumSession.msg"
        "msg/IridiumStatus.msg"
        )

rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp"
        )

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
        rosidl_default_generators
        seabot2_depth_filter
        pressure_bme280_driver
        seabot2_power_driver
        seabot2_piston_driver
        seabot2_mission
        gpsd_client
        seabot2_safety
        seabot2_lambert
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
