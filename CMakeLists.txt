cmake_minimum_required(VERSION 2.8.3)
project(cam_imu_sync)

add_definitions(-std=c++0x)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  camera_base
  bluefox2
  imu_vn_100
  roscpp
  rosbag
  cv_bridge
  eigen_conversions
  sensor_msgs
  feature_detector
  feature_tracker
  vision_utils
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 2.4.10 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
  boost eigen3 opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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

add_executable(sync_test_node
  src/sync_verification.cpp
)
target_link_libraries(sync_test_node
  ${catkin_LIBRARIES}
  opencv_core
  opencv_highgui
)
