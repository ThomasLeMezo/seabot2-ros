cmake_minimum_required(VERSION 3.8)
project(seabot2_depth_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(seabot2_piston_driver REQUIRED)
find_package(seabot2_mission REQUIRED)
find_package(seabot2_depth_filter REQUIRED)
find_package(seabot2_kalman REQUIRED)
find_package(seabot2_safety REQUIRED)
find_package(seabot2_density REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ibex REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DepthControlDebug.msg"
        "msg/AlphaDebug.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Executable
add_executable(depth_control_node src/depth_control_node.cpp src/alpha_solver.cpp)
ament_target_dependencies(depth_control_node
        rclcpp
        rosidl_default_generators
        seabot2_piston_driver
        seabot2_mission
        seabot2_depth_filter
        seabot2_kalman
        seabot2_safety
        std_srvs
        std_msgs
        seabot2_density
        temperature_tsys01_driver
        )
target_link_libraries(depth_control_node
        "${cpp_typesupport_target}"
        Ibex::ibex
        )

### Includes
target_include_directories(depth_control_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        Ibex::ibex
        )

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS depth_control_node
        DESTINATION lib/${PROJECT_NAME})


#find_package(ibex REQUIRED)
#add_executable(test_ibex src/alpha_solver.cpp src/alpha_solver.cpp)
#ament_target_dependencies(test_ibex
#        rclcpp
#        rosidl_default_generators
#        std_srvs
#        ibex
#        )
#target_link_libraries(test_ibex "${cpp_typesupport_target}" Ibex::ibex OpenMP::OpenMP_CXX)
#target_include_directories(test_ibex PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#        Ibex::ibex
#        )
#
#install(TARGETS test_ibex
#        DESTINATION lib/${PROJECT_NAME})

ament_package()
