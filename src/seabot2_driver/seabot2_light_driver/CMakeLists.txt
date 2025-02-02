cmake_minimum_required(VERSION 3.8)
project(seabot2_light_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_srvs REQUIRED)
find_package(std_srvs REQUIRED)

### Executable
add_executable(light_node
        src/light_node.cpp
        src/light.cpp
)

ament_target_dependencies(light_node
        rclcpp
        seabot2_srvs
        std_srvs
)
target_link_libraries(light_node
        "${cpp_typesupport_target}"
        i2c
)

### Includes
target_include_directories(light_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(light_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

### Install
install(TARGETS light_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()