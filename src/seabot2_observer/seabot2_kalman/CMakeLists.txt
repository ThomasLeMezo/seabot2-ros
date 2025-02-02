cmake_minimum_required(VERSION 3.8)
project(seabot2_kalman)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

### Executable
add_executable(kalman_node
        src/kalman_node.cpp
        src/kalman.cpp
)

ament_target_dependencies(kalman_node
        rclcpp
        seabot2_msgs
        std_msgs
)

target_link_libraries(kalman_node
        "${cpp_typesupport_target}"
)

### Includes
target_include_directories(kalman_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS kalman_node
        DESTINATION lib/${PROJECT_NAME}
)


if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    ### Export library
    include_directories(include/seabot2_kalman)

    add_library(kalman SHARED
            src/kalman.cpp
    )

    target_include_directories(kalman PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            include/seabot2_kalman
    )

    set_target_properties(kalman PROPERTIES PUBLIC_HEADER
            include/seabot2_kalman/kalman.h
    )

    ament_target_dependencies(kalman
            rclcpp
    )

    target_link_libraries(kalman "${cpp_typesupport_target}")

    #ament_export_targets(kalman HAS_LIBRARY_TARGET)

    install(TARGETS kalman
            LIBRARY DESTINATION lib
            PUBLIC_HEADER DESTINATION include/seabot2_kalman/seabot2_kalman/
    )
endif ()

ament_package()
