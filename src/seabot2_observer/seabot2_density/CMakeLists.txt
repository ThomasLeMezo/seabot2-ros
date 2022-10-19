cmake_minimum_required(VERSION 3.8)
project(seabot2_density)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(seabot2_depth_filter REQUIRED)
find_package(temperature_tsys01_driver REQUIRED)

### Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Density.msg"
        DEPENDENCIES std_msgs
        )
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
        lib/TeosCpp/include)

### Executable
add_executable(density_node src/density_node.cpp)

ament_target_dependencies(density_node
        rclcpp
        rosidl_default_generators
        seabot2_depth_filter
        temperature_tsys01_driver
        )
target_link_libraries(density_node
        "${cpp_typesupport_target}"
        teos)

### Includes
target_include_directories(density_node PUBLIC
        lib/TeosCpp/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install

install(TARGETS teos
        LIBRARY DESTINATION lib
)
install(TARGETS density_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
