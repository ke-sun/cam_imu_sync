cmake_minimum_required(VERSION 2.8.3)
project(camera_imu_sync)

find_package(catkin REQUIRED COMPONENTS
  camera_base
  bluefox2
  imu_3dm_gx4
  roscpp
)

find_package(Boost REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_definitions("-std=c++0x -Wall")


add_executable(stereo_imu_sync_node
  src/stereo_imu_sync_node.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(camera_imu_sync_node camera_imu_sync_generate_messages_cpp)

 target_link_libraries(stereo_imu_sync_node
   ${catkin_LIBRARIES}
 )
