cmake_minimum_required(VERSION 3.8)
project(seabot2_density)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)

### Library
add_library(teos SHARED
        TeosCpp/src/SaarDataHandler.cpp
        TeosCpp/src/TeosBase.cpp
        TeosCpp/src/TeosIce.cpp
        TeosCpp/src/TeosSea.cpp
)
target_include_directories(teos PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/TeosCpp/include>
        $<INSTALL_INTERFACE:include>
        TeosCpp/include/TeosCpp
)

file(GLOB_RECURSE TEOS_PUBLIC_HEADERS
        "${PROJECT_SOURCE_DIR}/TeosCpp/include/*.h"
)

set_target_properties(teos PROPERTIES PUBLIC_HEADER
        "${TEOS_PUBLIC_HEADERS}"
)

### Executable
add_executable(density_node
        src/density_node.cpp
)

ament_target_dependencies(density_node
        rclcpp
        seabot2_msgs
)
target_link_libraries(density_node
        teos
)

### Includes
target_include_directories(density_node PUBLIC
        TeosCpp/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS teos
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/seabot2_density/teos
)
install(TARGETS density_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
