cmake_minimum_required(VERSION 2.8.3)
project(cam_imu_sync)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

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
  DEPENDS boost
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

add_library(cam_imu_sync
  src/CamImuSynchronizer.cpp
)
target_link_libraries(cam_imu_sync
  ${catkin_LIBRARIES}
)
add_dependencies(cam_imu_sync
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(cam_imu_sync_node
  src/cam_imu_sync_node.cpp
)
target_link_libraries(cam_imu_sync_node
  cam_imu_sync
  ${catkin_LIBRARIES}
)
