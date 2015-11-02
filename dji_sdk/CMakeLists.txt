cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  interactive_markers
  dji_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   rel_pos.msg
   euler.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  set_home.srv
  set_target_point.srv
  cmd_sdk.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES dji_sdk
  CATKIN_DEPENDS roscpp std_msgs rospy message_runtime interactive_markers dji_ros
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


##SET_SOURCE_FILES_PROPERTIES(${DJI_SDK_LIB_SOURCES} PROPERTIES LANGUAGE CXX)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable

add_executable(dji_sdk_node src/dji_sdk_node.cpp )
add_dependencies(dji_sdk_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_sdk_node ${catkin_LIBRARIES})

add_executable(dji_gps_node src/dji_gps_node.cpp )
add_dependencies(dji_gps_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_gps_node ${catkin_LIBRARIES})

add_executable(dji_point_node src/dji_point_node.cpp )
add_dependencies(dji_point_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_point_node ${catkin_LIBRARIES})

add_executable(dji_debug_node src/dji_debug_node.cpp )
add_dependencies(dji_debug_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_debug_node ${catkin_LIBRARIES})

add_executable(dji_quat_node src/dji_quat_node.cpp )
add_dependencies(dji_quat_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_quat_node ${catkin_LIBRARIES})

add_executable(dji_follow_node src/dji_follow_node.cpp )
add_dependencies(dji_follow_node dji_sdk_generate_messages_cpp)
target_link_libraries(dji_follow_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
## add_dependencies(dji_sdk_node dji_sdk_generate_messages_cpp)

## Specify libraries to link a library or executable target against




