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
        lib/TeosCpp/src/SaarDataHandler.cpp
        lib/TeosCpp/src/TeosBase.cpp
        lib/TeosCpp/src/TeosIce.cpp
        lib/TeosCpp/src/TeosSea.cpp
)
target_include_directories(teos PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        lib/TeosCpp/include
)

file(GLOB_RECURSE TEOS_PUBLIC_HEADERS
        "${PROJECT_SOURCE_DIR}/lib/TeosCpp/include/*.h"
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
        "${cpp_typesupport_target}"
        teos
)

### Includes
target_include_directories(density_node PUBLIC
        lib/TeosCpp/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Install
install(TARGETS teos
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/seabot2_density/seabot2_density/teos
)
install(TARGETS density_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
