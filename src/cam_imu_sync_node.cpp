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

#include <ros/ros.h>
#include <cam_imu_sync/CamImuSynchronizer.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_imu_sync");
  ros::NodeHandle nh("~");

  // New instance of the IMU
  cam_imu_sync::CamImuSynchronizer sync_driver(nh);
  if (!sync_driver.initialize()) return -1;

  // Start the driver
  sync_driver.start();

  ros::spin();

  return 0;
}
