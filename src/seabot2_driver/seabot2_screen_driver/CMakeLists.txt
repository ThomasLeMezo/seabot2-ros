cmake_minimum_required(VERSION 3.8)
project(seabot2_screen_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pressure_bme280_driver REQUIRED)
find_package(seabot2_mission REQUIRED)
find_package(seabot2_power_driver REQUIRED)
find_package(seabot2_safety REQUIRED)

### Executable
add_executable(screen_node src/screen_node.cpp src/screen.cpp)
ament_target_dependencies(screen_node
        rclcpp
        pressure_bme280_driver
        seabot2_mission
        seabot2_power_driver
        seabot2_safety
        )
target_link_libraries(screen_node "${cpp_typesupport_target}" i2c)

### Includes
target_include_directories(screen_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

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

  #  ament_add_gtest(test_screen_driver //.cpp src/.cpp)
  #  ament_target_dependencies(test_screen_driver rclcpp)
  #  target_link_libraries(test_screen_driver ${rclcpp_LIBRARIES} i2c)
  #  target_include_directories(test_screen_driver PUBLIC
  #          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #          $<INSTALL_INTERFACE:include>)
endif()

### Install
install(TARGETS screen_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
