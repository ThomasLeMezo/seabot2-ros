cmake_minimum_required(VERSION 3.8)
project(temperature_tsys01_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/TemperatureSensorData.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(temperature_tsys01_node src/temperature_tsys01_node.cpp src/temperature_tsys01.cpp)
ament_target_dependencies(temperature_tsys01_node
  rclcpp
  rosidl_default_generators
)
target_link_libraries(temperature_tsys01_node "${cpp_typesupport_target}" i2c)

### Includes
target_include_directories(temperature_tsys01_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

#  ament_add_gtest(ms5803_test test/temperature_tsys01_test.cpp src/temperature_tsys01.cpp)
#  ament_target_dependencies(ms5803_test rclcpp)
#  target_link_libraries(ms5803_test ${rclcpp_LIBRARIES} i2c)
#  target_include_directories(ms5803_test PUBLIC
#          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:include>)

endif()


### Install
install(TARGETS temperature_tsys01_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
