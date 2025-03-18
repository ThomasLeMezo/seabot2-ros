cmake_minimum_required(VERSION 3.8)
project(seabot2_temperature_profile)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Executable
add_executable(temperature_profile_node
        src/temperature_profile_node.cpp
        src/temperature_profile.cpp
)

ament_target_dependencies(temperature_profile_node
        rclcpp
        seabot2_msgs
)

### Includes
target_include_directories(temperature_profile_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS temperature_profile_node
        DESTINATION lib/${PROJECT_NAME})


if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    ### Export library
    include_directories(include/seabot2_temperature_profile)

    add_library(temperature_profile SHARED
            src/temperature_profile.cpp
    )

    target_include_directories(temperature_profile PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(temperature_profile
            rclcpp
    )

    ament_export_targets(temperature_profile HAS_LIBRARY_TARGET)
    ament_export_dependencies(rclcpp)

    install(
            DIRECTORY include/
            DESTINATION include
    )

    install(
            TARGETS temperature_profile
            EXPORT temperature_profile
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES DESTINATION include
    )
endif ()

ament_package()
