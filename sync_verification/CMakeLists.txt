cmake_minimum_required(VERSION 2.8.3)
project(sync_verification)

add_definitions(-std=c++0x)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
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
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
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

add_library(sync_verifier
  src/SyncVerifier.cpp
)
target_link_libraries(sync_verifier
  ${catkin_LIBRARIES}
  opencv_core
  opencv_highgui
)
add_dependencies(sync_verifier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_executable(sync_verify_node
  src/sync_verify_node.cpp
)
target_link_libraries(sync_verify_node
  ${catkin_LIBRARIES}
  sync_verifier
)
