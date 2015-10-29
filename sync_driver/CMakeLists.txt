cmake_minimum_required(VERSION 2.8.3)
project(sync_driver)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  camera_base
  bluefox2
  flea3
  imu_vn_100
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
  boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_library(sync_chameleon3
  src/Chameleon3ImuSynchronizer.cpp
)
target_link_libraries(sync_chameleon3
  ${catkin_LIBRARIES}
)
add_dependencies(sync_chameleon3
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#add_library(sync_driver
#  src/CamImuSynchronizer.cpp
#)
#target_link_libraries(sync_driver
#  ${catkin_LIBRARIES}
#)

add_executable(sync_driver_node
  src/sync_driver_node.cpp
)
target_link_libraries(sync_driver_node
  sync_chameleon3
  ${catkin_LIBRARIES}
)
