cmake_minimum_required(VERSION 2.8.3)
project(robot_drive)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Release" )
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

find_package(catkin REQUIRED )
find_package(Eigen3  REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)


add_service_files(
  FILES
  RechargeStatus.srv
  StopStatus.srv
)

generate_messages(DEPENDENCIES std_msgs)
# 如果没有这句话 那么生成的可执行文件就会到build文件下  正确的是应该到devel下
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
  # ${Boost_INCLUDE_DIR}
)

include_directories(
${PROJECT_SOURCE_DIR}/../../uslam/ubt_key/include
)

# link directories
link_directories(
  #${PROJECT_SOURCE_DIR}/../../uslam/ubt_key/lib
  # ${Boost_LIBRARY_DIRS}
)

# set(ALL_TARGET_LIBRARIES "")


###########
## Build ##
###########

#add_library(

#)

#target_link_libraries(
# ${ALL_TARGET_LIBRARIES}
#)

add_executable(RobotDrive_node
  src/app/RobotDrive_node.cpp
)
add_dependencies(RobotDrive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(RobotDrive_node
  ${catkin_LIBRARIES}
  #ubt_key
  #glog
)

 install(TARGETS  RobotDrive_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


