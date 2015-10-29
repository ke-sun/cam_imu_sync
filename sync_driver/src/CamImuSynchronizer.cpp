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

#include <sync_driver/CamImuSynchronizer.h>
#include <bluefox2/Bluefox2DynConfig.h>

namespace cam_imu_sync {

CamImuSynchronizer::CamImuSynchronizer(const ros::NodeHandle& n)
    : nh(n), imu(n), lcam(n, "left"), rcam(n, "right")
{
  return;
}

bool CamImuSynchronizer::initialize() {
  // Initialize IMU
  if (!imu.initialize()) {
    ROS_ERROR("Fail to initialize IMU.");
    return false;
  }
  // Initialize cameras
  int cam_expose_us = 5000;
  nh.param<int>("expose_us", cam_expose_us, 5000);

  bluefox2::Bluefox2DynConfig cam_config;
  cam_config.aec = 0;
  cam_config.expose_us = cam_expose_us;
  cam_config.ctm = 3;
  lcam.camera().Configure(cam_config);
  lcam.set_fps(static_cast<int>(imu.getSyncRate()));
  rcam.camera().Configure(cam_config);
  rcam.set_fps(static_cast<int>(imu.getSyncRate()));

  return true;
}

void CamImuSynchronizer::start() {
  // Start the IMU streaming
  imu.enableIMUStream(true);
  // Start polling images from the camera(s)
  img_poll_thread_ptr = boost::shared_ptr<boost::thread>(
      new boost::thread(&CamImuSynchronizer::pollImage, this));
  return;
}

void CamImuSynchronizer::pollImage() {
  // TODO: continuously poll images and
  //    assign the latest time stamp from
  //    IMU to the image msgs
  float sync_rate = imu.getSyncRate();
  float sync_duration = 1.0/sync_rate;
  ros::Rate r(static_cast<int>(sync_rate));
  ros::Duration(1e-3).sleep();

  while (ros::ok()) {
    ros::Time new_time_stamp =
      imu.getSyncTime() + ros::Duration(sync_duration);
    lcam.RequestSingle();
    rcam.RequestSingle();
    lcam.PublishCamera(new_time_stamp);
    rcam.PublishCamera(new_time_stamp);
    r.sleep();
  }
  return;
}
}
