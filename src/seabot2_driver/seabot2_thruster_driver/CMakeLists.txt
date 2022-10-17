cmake_minimum_required(VERSION 3.8)
project(seabot2_thruster_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Engine.msg"
        "msg/Velocity.msg"
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(thruster_node src/thruster_node.cpp src/thruster.cpp)
ament_target_dependencies(thruster_node
        rclcpp
        geometry_msgs
        rosidl_default_generators
)
target_link_libraries(thruster_node "${cpp_typesupport_target}" i2c)

### Includes
target_include_directories(thruster_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(thruster_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()

#  ament_add_gtest(test_thruster_driver //.cpp src/.cpp)
#  ament_target_dependencies(test_thruster_driver rclcpp)
#  target_link_libraries(test_thruster_driver ${rclcpp_LIBRARIES} i2c)
#  target_include_directories(test_thruster_driver PUBLIC
#          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:include>)
endif()

### Install
install(TARGETS thruster_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
