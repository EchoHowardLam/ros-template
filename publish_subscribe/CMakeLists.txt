cmake_minimum_required(VERSION 3.0.2)
project(publish_subscribe)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_subscribe
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(talker_node src/talker.cpp)
add_executable(talker_with_param_node src/talker_with_param.cpp)
add_executable(listener_node src/listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(talker_node
  ${catkin_LIBRARIES}
)
target_link_libraries(talker_with_param_node
  ${catkin_LIBRARIES}
)
target_link_libraries(listener_node
  ${catkin_LIBRARIES}
)
