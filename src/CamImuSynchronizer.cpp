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



#include <cam_imu_sync/CamImuSynchronizer.h>

namespace cam_imu_sync {

CamImuSynchronizer::CamImuSynchronizer(
    const ros::Nodehandle& n):
  nh(n),
  imu(n)
  // TODO: pass the ros node handle to camera objects
  {
  return;
}

bool CamImuSynchronizer::initialize() {
  // Initialize IMU
  if(!imu.initialize()) {
    ROS_ERROR("Fail to initialize IMU.");
    return false;
  }
  // TODO: intialize camera(s)

  return true;
}

void CamImuSynchronizer::start() {
  // Start the IMU streaming
  imu.enableIMUStream();
  // Start polling images from the camera(s)
  img_poll_thread_ptr = new boost::thread(
      &CamImuSynchronizer::pollImage, this);
  return;
}

void CamImuSynchronizer::pollImage() {
  // TODO: continuously poll images and
  //    assign the latest time stamp from
  //    IMU to the image msgs
  while (true) {
    // bool new_flag = cam.getImage()
    // if (new_flag) {
    //  ros::Time new_time_stamp = imu.getSyncTime();
    //  cam.publishImage(new_time_stamp);
    // }
  }
  return;
}


}
