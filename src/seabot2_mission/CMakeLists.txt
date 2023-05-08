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
find_package(ibex REQUIRED)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
  message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Waypoint.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(mission_node src/mission_node.cpp src/mission.cpp)
ament_target_dependencies(mission_node
        rclcpp
        rosidl_default_generators
        std_srvs
        seabot2_light_driver
        ibex
        )
target_link_libraries(mission_node "${cpp_typesupport_target}")
target_include_directories(mission_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


# Ibex test


add_executable(test_ibex src/solver.cpp)
ament_target_dependencies(test_ibex
        rclcpp
        rosidl_default_generators
        std_srvs
        seabot2_light_driver
        ibex
        )
target_link_libraries(test_ibex "${cpp_typesupport_target}" Ibex::ibex)
target_include_directories(test_ibex PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        Ibex::ibex
        )

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS mission_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_ibex
        DESTINATION lib/${PROJECT_NAME})

ament_package()
