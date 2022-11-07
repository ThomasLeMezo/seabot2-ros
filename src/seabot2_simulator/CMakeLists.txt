cmake_minimum_required(VERSION 3.8)

project(seabot2_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(seabot2_density REQUIRED)

find_library(LIBTEOS teos REQUIRED)

### Executable
add_executable(simulator_node src/simulator_node.cpp src/simulator.cpp)
ament_target_dependencies(simulator_node
        rclcpp
        rosidl_default_generators
        seabot2_density
)
target_link_libraries(simulator_node
        "${cpp_typesupport_target}"
        ${LIBTEOS}
        )

### Includes
target_include_directories(simulator_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()


### Install
install(TARGETS simulator_node
        DESTINATION lib/${PROJECT_NAME})

### Biding
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
#add_subdirectory(pybind11)

set(PYTHON_PACKAGE_DIR "${CMAKE_CURRENT_BINARY_DIR}/python_package")
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/python/setup.py
        ${PYTHON_PACKAGE_DIR}/setup.py
)

pybind11_add_module(seabot2_simulator SHARED
        python/simulator_binding.cpp
        src/simulator.cpp)
target_link_libraries(seabot2_simulator PRIVATE
        ${LIBTEOS}
        )
target_include_directories(seabot2_simulator
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${seabot2_density_DIR}/../../../include/seabot2_density)

# Copy .so files in the python module directory
add_custom_command(TARGET seabot2_simulator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:seabot2_simulator>" "${PYTHON_PACKAGE_DIR}/seabot2_simulator"
        )
#install( DIRECTORY "${PYTHON_PACKAGE_DIR}/seabot2_simulator" DESTINATION "${PYTHON_INSTALL_DIR}")
add_custom_target(pip_package)
add_custom_command(TARGET pip_package PRE_BUILD
        COMMAND ${PYTHON_EXECUTABLE} ARGS setup.py bdist_wheel -d ${CMAKE_BINARY_DIR}
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        )

add_custom_target(install_python)
add_custom_command(TARGET install_python PRE_BUILD
        COMMAND pip install --upgrade .
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        )

add_dependencies(install_python seabot2_simulator)

endif()

ament_package()

