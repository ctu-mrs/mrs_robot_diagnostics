cmake_minimum_required(VERSION 3.15.0)
project(mrs_robot_diagnostics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-save-temps)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  message_generation
  nodelet
  mrs_msgs
  mrs_lib
  mrs_errorgraph
  )

set(LIBRARIES
  MRSRobotDiagnostics_StateMonitor
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# messages
add_message_files(DIRECTORY msg FILES
  BatteryState.msg
  SensorStatus.msg
  NodeCpuLoad.msg

  GeneralRobotInfo.msg
  StateEstimationInfo.msg
  ControlInfo.msg
  CollisionAvoidanceInfo.msg
  UavInfo.msg
  SystemHealthInfo.msg

  UavState.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  )

## | ------------ MRSRobotDiagnostics_StateMonitor ------------ |

## Declare a C++ library
add_library(MRSRobotDiagnostics_StateMonitor
  src/state_monitor.cpp
  )

  add_library(MRSRobotDiagnostics_CameraHandler
  src/sensor_handler_plugins/camera_handler.cpp
)

add_dependencies(MRSRobotDiagnostics_StateMonitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MRSRobotDiagnostics_StateMonitor
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/mrs_robot_diagnostics/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
