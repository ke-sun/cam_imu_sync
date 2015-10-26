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

#ifndef CAMIMUSYNC_CAMIMUSYNCHRONIZER_H
#define CAMIMUSYNC_CAMIMUSYNCHRONIZER_H

#include <vector>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

namespace cam_imu_sync {

/**
 * @brief CamImuSynchronizer
 *        The class is used for synchronize the time stamp for
 *        imu and cameras. Currently, it only supports imu VN100
 *        from VECTORNAV and cameras from BLUEFOX. The cameras
 *        are supposed to be externally triggered by the imu.
 * @author Ke Sun
 */
template <typename Imu, typename Cam, typename CamConfig>
class CamImuSynchronizer {
 public:
  CamImuSynchronizer(const ros::NodeHandle& pnh, int num_cameras = 2);
  ~CamImuSynchronizer() = default;
  CamImuSynchronizer(const CamImuSynchronizer&) = delete;
  CamImuSynchronizer& operator=(const CamImuSynchronizer&) = delete;

  using CamPtr = boost::shared_ptr<Cam>;

  /**
   * @brief initialize Initialize IMU and CAM objects
   * @return True If the driver is initialized successfully.
   */
  virtual void configure() = 0;

  void configureEachCamera(CamConfig& config);

  /**
   * @brief start Starts the driver(IMU and CAM(s))
   */
  void start();

 protected:
  // Ros node
  ros::NodeHandle pnh_;

  // IMU object
  Imu imu_;

  // Camera object(s)
  std::vector<CamPtr> cameras_;

  // A seperate thread waiting for images
  boost::shared_ptr<boost::thread> img_poll_thread_;

  // Poll image(s) from camera(s)
  //    This function will be run on a seperate thread
  //    avoid blocking the main function.
  void pollImage();
};

template <typename Imu, typename Cam, typename CamConfig>
CamImuSynchronizer<Imu, Cam, CamConfig>::CamImuSynchronizer(
    const ros::NodeHandle& pnh, int num_cameras)
    : pnh_(pnh), imu_(pnh) {
  // TODO: Imu should establish class invariants in constructor
  if (!imu_.initialize()) {
    throw std::runtime_error("Failed to initialize imu");
  }

  for (int i = 0; i < num_cameras; ++i) {
    const auto prefix = "cam" + std::to_string(i);
    cameras_.push_back(boost::make_shared<Cam>(pnh, prefix));
  }
}

template <typename Imu, typename Cam, typename CamConfig>
void CamImuSynchronizer<Imu, Cam, CamConfig>::start() {
  // Start the IMU streaming
  imu_.enableIMUStream(true);
  // Start polling images from the camera(s)
  img_poll_thread_.reset(new boost::thread(
      &CamImuSynchronizer<Imu, Cam, CamConfig>::pollImage, this));
}

template <typename Imu, typename Cam, typename CamConfig>
void CamImuSynchronizer<Imu, Cam, CamConfig>::pollImage() {
  float sync_rate = imu_.getSyncRate();
  float sync_duration = 1.0 / sync_rate;
  ros::Rate r(sync_rate);

  while (ros::ok()) {
    const auto time = imu_.getSyncTime() + ros::Duration(sync_duration);
    for (const auto& cam : cameras_) {
      cam->RequestSingle();
    }
    for (auto& cam : cameras_) {
      cam->PublishCamera(time);
    }
    r.sleep();
  }
}

template <typename Imu, typename Cam, typename CamConfig>
void CamImuSynchronizer<Imu, Cam, CamConfig>::configureEachCamera(
    CamConfig& config) {
  for (auto& cam : cameras_) {
    // TODO: this is bad interface, maybe fix it
    cam->camera().Configure(config);
  }
}

}  // namespace cam_imu_sync

#endif  // CAM_IMU_SYNC_CAMIMUSYNCHRONIZER_H
