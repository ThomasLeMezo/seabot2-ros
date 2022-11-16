cmake_minimum_required(VERSION 3.8)
project(seabot2_kalman)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(seabot2_piston_driver REQUIRED)
find_package(seabot2_depth_filter REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(seabot2_density REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)
find_package(seabot2_depth_filter REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/KalmanState.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(kalman_node
        src/kalman_node.cpp
        src/kalman.cpp
        )

ament_target_dependencies(kalman_node
        rclcpp
        rosidl_default_generators
        seabot2_piston_driver
        seabot2_depth_filter
        std_srvs
        std_msgs
        seabot2_density
        temperature_tsys01_driver
        seabot2_depth_filter
        )
target_link_libraries(kalman_node
        "${cpp_typesupport_target}"
        )

### Includes
target_include_directories(kalman_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS kalman_node
        DESTINATION lib/${PROJECT_NAME})


### Export library
include_directories(include/seabot2_kalman)
add_library(kalman SHARED
        src/kalman.cpp
)
target_include_directories(kalman PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include/seabot2_kalman)
set_target_properties(kalman PROPERTIES PUBLIC_HEADER include/seabot2_kalman/kalman.h)

ament_target_dependencies(kalman
        rclcpp
        )
target_link_libraries(kalman "${cpp_typesupport_target}")

#ament_export_targets(kalman HAS_LIBRARY_TARGET)

install(TARGETS kalman
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/seabot2_kalman/seabot2_kalman/kalman
)

ament_package()
