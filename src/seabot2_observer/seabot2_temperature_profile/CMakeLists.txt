cmake_minimum_required(VERSION 3.8)
project(seabot2_temperature_profile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(seabot2_depth_filter REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/TemperatureProfile.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(temperature_profile_node
        src/temperature_profile_node.cpp
        src/temperature_profile.cpp
        )

ament_target_dependencies(temperature_profile_node
        rclcpp
        rosidl_default_generators
        seabot2_depth_filter
        temperature_tsys01_driver
        )
target_link_libraries(temperature_profile_node
        "${cpp_typesupport_target}"
        )

### Includes
target_include_directories(temperature_profile_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS temperature_profile_node
        DESTINATION lib/${PROJECT_NAME})


if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
### Export library
include_directories(include/seabot2_temperature_profile)
add_library(temperature_profile SHARED
        src/temperature_profile.cpp
)
target_include_directories(temperature_profile PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include/seabot2_temperature_profile)
set_target_properties(temperature_profile PROPERTIES
        PUBLIC_HEADER include/seabot2_temperature_profile/temperature_profile.h)

ament_target_dependencies(temperature_profile
        rclcpp
        )
target_link_libraries(temperature_profile "${cpp_typesupport_target}")

#ament_export_targets(temperature_profile HAS_LIBRARY_TARGET)

install(TARGETS temperature_profile
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/seabot2_temperature_profile/seabot2_temperature_profile/
)
endif()

ament_package()
