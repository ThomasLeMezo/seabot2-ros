cmake_minimum_required(VERSION 3.8)

project(seabot2_simulator)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot2_msgs REQUIRED)
find_package(seabot2_density REQUIRED)
find_package(seabot2_kalman REQUIRED)
find_package(seabot2_depth_control REQUIRED)
find_package(seabot2_mission REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(seabot2_piston_driver REQUIRED)

find_library(LIBTEOS teos REQUIRED)
find_library(LIBKALMAN kalman REQUIRED)
find_library(LIBDEPTHCONTROL depth_control REQUIRED)
find_library(LIBMISSION mission REQUIRED)
find_library(LIBTEMPERATUREPROFILE temperature_profile REQUIRED)

### Executable
add_executable(simulator_node
        src/simulator_node.cpp
        src/simulator.cpp
)

ament_target_dependencies(simulator_node
        rclcpp
        seabot2_density
        seabot2_msgs
        seabot2_kalman
        seabot2_depth_control
        seabot2_mission
)
target_link_libraries(simulator_node
        ${cpp_typesupport_target}
        ${LIBTEOS}
        ${LIBKALMAN}
        ${LIBDEPTHCONTROL}
        ${LIBMISSION}
        ${LIBTEMPERATUREPROFILE}
        )

### Includes
target_include_directories(simulator_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

### Install
install(TARGETS simulator_node
        DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
        DESTINATION $ENV{HOME})

#
#########################################################################################################################
#### Biding
#
## Find python version & cmake variables
#find_package(Python COMPONENTS Interpreter Development REQUIRED)
#
## Find package in system path
#find_package(pybind11 CONFIG REQUIRED)
#
## define the path for the output files
#set(PYTHON_PACKAGE_DIR "${CMAKE_CURRENT_BINARY_DIR}/python_package")
#
## create a directory inside the ouput files path
#file(MAKE_DIRECTORY ${PYTHON_PACKAGE_DIR}/seabot2py)
#execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/python/seabot2py/" "${PYTHON_PACKAGE_DIR}/seabot2py")
#
## Add the configuration file (the input file contains macro variables)
#configure_file(
#        ${CMAKE_CURRENT_SOURCE_DIR}/python/setup.py
#        ${PYTHON_PACKAGE_DIR}/setup.py
#)
#
## Decleare the module for pybind11
#pybind11_add_module(seabot2py SHARED
#        python/simulator_binding.cpp
#        src/simulator.cpp)
#
## Link with libs
#target_link_libraries(seabot2py PRIVATE
#        "${cpp_typesupport_target}"
#        ${LIBKALMAN}
#        ${LIBTEOS}
#        )
#
## Include .h files and other header from ros package
#target_include_directories(seabot2py
#        PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#        ${seabot2_density_DIR}/../../../include/seabot2_density
#        ${seabot2_kalman_DIR}/../../../include/seabot2_kalman
#        )
#
## Copy .so files in the python module directory
#add_custom_command(TARGET seabot2py POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:seabot2py>" "${PYTHON_PACKAGE_DIR}/seabot2py")
#
## install( DIRECTORY "${PYTHON_PACKAGE_DIR}/seabot2py" DESTINATION "${PYTHON_INSTALL_DIR}")
#
## Create the wheel for the package
##add_custom_target(pip_package)
#add_custom_command(TARGET seabot2py POST_BUILD
#        COMMAND python3 ARGS setup.py bdist_wheel -d ${CMAKE_BINARY_DIR}
#        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
#        )
#
## Install the wheel
##add_custom_target(install_python)
#add_custom_command(TARGET seabot2py POST_BUILD
#        COMMAND pip install --upgrade .
#        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
#        )
#
##add_dependencies(seabot2py pip_package)
##add_dependencies(pip_package install_python)

ament_package()
endif()

