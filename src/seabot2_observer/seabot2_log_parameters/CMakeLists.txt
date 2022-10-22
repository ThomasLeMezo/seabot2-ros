cmake_minimum_required(VERSION 3.8)
project(seabot2_log_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)

### Interfaces
### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/LogParameter.msg"
        DEPENDENCIES rcl_interfaces
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(log_parameter_node src/log_parameter_node.cpp)
ament_target_dependencies(log_parameter_node
        rclcpp
        rosidl_default_generators
        rcl_interfaces
        )
target_link_libraries(log_parameter_node "${cpp_typesupport_target}" proj)

### Includes
target_include_directories(log_parameter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS log_parameter_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
