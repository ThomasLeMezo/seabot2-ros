cmake_minimum_required(VERSION 3.8)
project(seabot2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs)
find_package(geometry_msgs)
find_package(rcl_interfaces REQUIRED)

### Interfaces

set(msg_files
        "msg/AlphaDebug.msg"
        "msg/Bme280Data.msg"
        "msg/DebugFusion.msg"
        "msg/Density.msg"
        "msg/DepthControlDebug.msg"
        "msg/DepthControlSetPoint.msg"
        "msg/DepthPose.msg"
        "msg/Engine.msg"
        "msg/GnssPose.msg"
        "msg/GpsFix.msg"
        "msg/GpsPps.msg"
        "msg/IridiumSession.msg"
        "msg/IridiumStatus.msg"
        "msg/KalmanState.msg"
        "msg/LogParameter.msg"
        "msg/MissionState.msg"
        "msg/PistonSetPoint.msg"
        "msg/PistonState.msg"
        "msg/PowerState.msg"
        "msg/PressureSensorData.msg"
        "msg/Profile.msg"
        "msg/RawData.msg"
        "msg/RPY.msg"
        "msg/SafetyStatus2.msg"
        "msg/SafetyStatus.msg"
        "msg/SimulationDebug.msg"
        "msg/SimulationThermocline.msg"
        "msg/SyncDspic.msg"
        "msg/TemperatureKeepingDebug.msg"
        "msg/TemperatureProfile.msg"
        "msg/TemperatureSensorData.msg"
        "msg/Velocity.msg"
        "msg/Waypoint.msg"
        "msg/XbeeMessageHeader.msg"
        "msg/XbeeMessageLogState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs rcl_interfaces
        ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
