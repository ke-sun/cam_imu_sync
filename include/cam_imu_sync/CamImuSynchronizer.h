/*
 * Copyright [2015] [Ke Sun  sunke.polyu@gmail.com]
 *                  [Chao Qu quchao@seas.upenn.edu]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAM_IMU_SYNC
#define CAM_IMU_SYNC

#include <vector>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <imu_vn_100/imu_ros_base.h>
#include <bluefox2/bluefox2_ros.h>
// TODO: include the header files for the camera driver

namespace cam_imu_sync {

/**
 * @brief CamImuSynchronizer
 *        The class is used for synchronize the time stamp for
 *        imu and cameras. Currently, it only supports imu VN100
 *        from VECTORNAV and cameras from BLUEFOX. The cameras
 *        are supposed to be externally triggered by the imu.
 * @author Ke Sun
 */
class CamImuSynchronizer {
 public:
  CamImuSynchronizer(const ros::NodeHandle& n);
  ~CamImuSynchronizer() {}

  /**
   * @brief initialize Initialize IMU and CAM objects
   * @return True If the driver is initialized successfully.
   */
  bool initialize();

  /**
   * @brief start Starts the driver(IMU and CAM(s))
   */
  void start();

 private:
  // Ros node
  ros::NodeHandle nh;

  // IMU object
  imu_vn_100::ImuRosBase imu;

  // TODO: Camera object(s)
  bluefox2::Bluefox2Ros lcam;
  bluefox2::Bluefox2Ros rcam;

  // A seperate thread waiting for images
  boost::shared_ptr<boost::thread> img_poll_thread_ptr;

  // Poll image(s) from camera(s)
  //    This function will be run on a seperate thread
  //    avoid blocking the main function.
  void pollImage();

  // Disable copy and assign contructor
  CamImuSynchronizer(const CamImuSynchronizer&);
  CamImuSynchronizer& operator=(const CamImuSynchronizer&);
};
}

#endif
