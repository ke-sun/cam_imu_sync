cmake_minimum_required(VERSION 2.8.3)
project(cam_imu_sync)

add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  camera_base
  flea3
  imu_vn_100
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_library(${PROJECT_NAME} src/CamImuSynchronizer.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(${PROJECT_NAME}_node src/cam_imu_sync_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
