cmake_minimum_required(VERSION 3.8)
project(seabot2_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(seabot2_light_driver REQUIRED)
find_package(std_srvs REQUIRED)
find_package(seabot2_depth_filter REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)
find_package(seabot2_temperature_profile REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MissionState.msg"
        "srv/AlphaMission.srv"
        "msg/DepthControlSetPoint.msg"
        "msg/Waypoint.msg"
        "msg/TemperatureKeepingDebug.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(mission_node src/mission_node.cpp src/mission.cpp src/waypoint.cpp)
ament_target_dependencies(mission_node
        rclcpp
        rosidl_default_generators
        std_srvs
        seabot2_light_driver
        seabot2_depth_filter
        temperature_tsys01_driver
        seabot2_temperature_profile
)
target_link_libraries(mission_node "${cpp_typesupport_target}")
target_include_directories(mission_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

### Install
install(TARGETS mission_node
        DESTINATION lib/${PROJECT_NAME})

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")

  ### Export library
  include_directories(include/seabot2_mission)
  add_library(mission SHARED
          src/mission.cpp
          src/waypoint.cpp
          )

  target_include_directories(mission PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          include/seabot2_mission
          )

  set(MISSION_HEADERS
          include/seabot2_mission/mission.hpp
          include/seabot2_mission/waypoint.hpp
          )

  set_target_properties(mission PROPERTIES
          PUBLIC_HEADER "${MISSION_HEADERS}"
          )

  ament_target_dependencies(mission
          rclcpp
          )

  target_link_libraries(mission
          "${cpp_typesupport_target}"
          )

  install(TARGETS mission
          LIBRARY DESTINATION lib
          PUBLIC_HEADER DESTINATION include/seabot2_mission/seabot2_mission/
          )
endif()

ament_package()
