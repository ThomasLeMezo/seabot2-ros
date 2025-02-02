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

target_link_libraries(mission_node
        "${cpp_typesupport_target}"
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
            include/seabot2_mission
    )

    set(MISSION_HEADERS
            include/seabot2_mission/mission.hpp
            include/seabot2_mission/waypoint.hpp
    )

    set_target_properties(mission PROPERTIES
            PUBLIC_HEADER "${MISSION_HEADERS}"
    )

    ament_target_dependencies(mission
            rclcpp
            seabot2_msgs
    )

    target_link_libraries(mission
            "${cpp_typesupport_target}"
    )

    install(TARGETS mission
            LIBRARY DESTINATION lib
            PUBLIC_HEADER DESTINATION include/seabot2_mission/seabot2_mission/
    )
endif ()

ament_package()
