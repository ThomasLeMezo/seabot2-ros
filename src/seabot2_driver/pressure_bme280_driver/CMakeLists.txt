cmake_minimum_required(VERSION 3.8)
project(pressure_bme280_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Bme280Data.msg"
        )

add_executable(bme280_node src/bme280_node.cpp src/bme280.c)
ament_target_dependencies(bme280_node rclcpp std_msgs)

# Includes
target_include_directories(bme280_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(bme280_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pressure_bme280_driver>)

# Custom interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bme280_node "${cpp_typesupport_target}" i2c)

install(TARGETS bme280_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
