cmake_minimum_required(VERSION 3.8)
project(pressure_ms5837_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Interfaces

### Executable
add_executable(pressure_ms5837_node
        src/pressure_ms5837_node.cpp
        src/pressure_ms5837.cpp
)

ament_target_dependencies(pressure_ms5837_node
  rclcpp
  seabot2_msgs
)

target_link_libraries(pressure_ms5837_node
        i2c
)

### Includes
target_include_directories(pressure_ms5837_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
#  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(ms5837_test test/pressure_ms5837_test.cpp src/pressure_ms5837.cpp)
  ament_target_dependencies(ms5837_test rclcpp)
  target_link_libraries(ms5837_test ${rclcpp_LIBRARIES} i2c)
  target_include_directories(ms5837_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)

endif()


### Install
install(TARGETS pressure_ms5837_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
