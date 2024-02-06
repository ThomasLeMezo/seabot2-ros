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

## Interfaces

### Executable
add_executable(audio_recorder
        src/audio_recorder_node.cpp
)

ament_target_dependencies(audio_recorder
        rclcpp
        rosidl_default_generators
        std_srvs
)

target_link_libraries(audio_recorder
        "${cpp_typesupport_target}"
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
