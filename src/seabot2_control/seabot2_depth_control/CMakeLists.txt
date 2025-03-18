cmake_minimum_required(VERSION 3.8)
project(seabot2_depth_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(seabot2_msgs REQUIRED)
find_package(seabot2_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)

### Interfaces

### Executable
add_executable(depth_control_node
        src/depth_control_node.cpp
        src/alpha_solver.cpp
        src/depth_control.cpp
)

ament_target_dependencies(depth_control_node
        rclcpp
        seabot2_srvs
        seabot2_msgs
        std_srvs
        std_msgs
)

### Includes
target_include_directories(depth_control_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS depth_control_node
        DESTINATION lib/${PROJECT_NAME})

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
#    find_package(Ibex REQUIRED)

    ### Export library
    include_directories(include/seabot2_depth_control)
    add_library(depth_control SHARED
            src/depth_control.cpp
            src/alpha_solver.cpp
    )

    target_include_directories(depth_control PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(depth_control
            rclcpp
    )

    ament_export_targets(depth_control HAS_LIBRARY_TARGET)
    ament_export_dependencies(rclcpp)

    install(
            DIRECTORY include/
            DESTINATION include
    )

    install(
            TARGETS depth_control
            EXPORT depth_control
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES DESTINATION include
    )

endif()

ament_package()
