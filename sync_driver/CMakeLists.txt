cmake_minimum_required(VERSION 2.8.3)
project(sync_driver)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  camera_base
  bluefox2
  imu_vn_100
  roscpp
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
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

add_library(sync_driver
  src/CamImuSynchronizer.cpp
)
target_link_libraries(sync_driver
  ${catkin_LIBRARIES}
)
add_dependencies(sync_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(sync_node
  src/cam_imu_sync_node.cpp
)
target_link_libraries(sync_node
  sync_driver
  ${catkin_LIBRARIES}
)
