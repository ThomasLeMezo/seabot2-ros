cmake_minimum_required(VERSION 3.8)
project(icm20948_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

add_subdirectory(Fusion)

### Executable
add_executable(icm20948
        src/icm20948_node.cpp
        src/icm20948.cpp
)

ament_target_dependencies(icm20948
        rclcpp
        seabot2_msgs
)

target_link_libraries(icm20948
        Fusion
)

### Includes
target_include_directories(icm20948 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

### Install
install(TARGETS icm20948
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS Fusion
        DESTINATION lib/
)

ament_package()
