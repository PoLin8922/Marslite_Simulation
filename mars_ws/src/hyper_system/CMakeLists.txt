cmake_minimum_required(VERSION 3.0.2)
project(hyper_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Navigability.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

add_executable(navigability_pub src/navigability_pub.cpp)
add_dependencies(navigability_pub hyper_system_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigability_pub ${catkin_LIBRARIES})
install(TARGETS navigability_pub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(LaserScan_to_MultiEchoLaserScan src/LaserScan_to_MultiEchoLaserScan.cpp)
target_link_libraries(LaserScan_to_MultiEchoLaserScan ${catkin_LIBRARIES})
install(TARGETS LaserScan_to_MultiEchoLaserScan RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

