cmake_minimum_required(VERSION 3.8)
project(seabot2_mission)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(seabot2_srvs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mission_node src/mission_node.cpp
        src/mission.cpp
        src/waypoint.cpp
)

ament_target_dependencies(mission_node
        rclcpp
        seabot2_msgs
        seabot2_srvs
        std_srvs
)

target_include_directories(mission_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS mission_node
        DESTINATION lib/${PROJECT_NAME})

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")

    ### Export library
    include_directories(include/seabot2_mission)
    add_library(mission SHARED
            src/mission.cpp
            src/waypoint.cpp
    )

    target_include_directories(mission PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )


    ament_target_dependencies(mission
            rclcpp
            seabot2_msgs
    )

    ament_export_targets(mission HAS_LIBRARY_TARGET)
    ament_export_dependencies(rclcpp)

    install(
            DIRECTORY include/
            DESTINATION include
    )

    install(
            TARGETS mission
            EXPORT mission
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES DESTINATION include
    )

endif ()

ament_package()
