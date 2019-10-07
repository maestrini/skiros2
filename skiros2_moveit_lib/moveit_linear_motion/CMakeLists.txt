cmake_minimum_required(VERSION 2.8.3)
project(moveit_linear_motion)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
   LinearMotion.action
   )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   moveit_msgs
   sensor_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_linear_motion
  CATKIN_DEPENDS moveit_msgs roscpp sensor_msgs moveit_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(moveit_linear_motion
   src/LinearMotion.cpp
 )

## Declare a C++ executable
add_executable(linear_motion_server src/LinearMotionServer.cpp src/LinearMotion.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(linear_motion_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(moveit_linear_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(linear_motion_server ${catkin_LIBRARIES})
target_link_libraries(moveit_linear_motion  ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} moveit_linear_motion linear_motion_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
