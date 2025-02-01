cmake_minimum_required(VERSION 3.8)
project(seabot2_audio_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gpsd_client REQUIRED)

# Message
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/SyncDspic.msg"
        DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Interfaces

### Executable
add_executable(audio_recorder
        src/audio_recorder_node.cpp
        src/tlv320adc.cpp
        src/dspic_acoustic.cpp
)

ament_target_dependencies(audio_recorder
        rclcpp
        rosidl_default_generators
        std_srvs
        std_msgs
        gpsd_client
)

target_link_libraries(audio_recorder
        "${cpp_typesupport_target}"
        i2c
)

### Includes
target_include_directories(audio_recorder PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS audio_recorder
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
