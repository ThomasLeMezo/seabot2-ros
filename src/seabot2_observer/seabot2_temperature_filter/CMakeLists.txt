cmake_minimum_required(VERSION 3.8)
project(seabot2_temperature_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)

### Interfaces
#rosidl_generate_interfaces(${PROJECT_NAME}
#        "msg/.msg"
#        DEPENDENCIES std_msgs
#        )
#rosidl_get_typesupport_target(cpp_typesupport_target
#        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(filter_temperature_node src/filter_temperature_node.cpp)
ament_target_dependencies(filter_temperature_node
        rclcpp
        rosidl_default_generators
        temperature_tsys01_driver
        )
target_link_libraries(filter_temperature_node "${cpp_typesupport_target}")

### Includes
target_include_directories(filter_temperature_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS filter_temperature_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
