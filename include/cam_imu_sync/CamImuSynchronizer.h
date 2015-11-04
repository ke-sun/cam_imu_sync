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

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <flea3/Flea3DynConfig.h>

namespace flea3 {
class Flea3Ros;
}  // namespace flea3

namespace imu_vn_100 {
class ImuRosBase;
}  // namespace imu_vn_100

namespace cam_imu_sync {

class CamImuSynchronizer {
 public:
  using Imu = imu_vn_100::ImuRosBase;
  using ImuPtr = boost::shared_ptr<Imu>;
  using Cam = flea3::Flea3Ros;
  using CamPtr = boost::shared_ptr<Cam>;
  using Config = flea3::Flea3DynConfig;

  CamImuSynchronizer(const ros::NodeHandle& pnh);
  ~CamImuSynchronizer() = default;
  CamImuSynchronizer(const CamImuSynchronizer&) = delete;
  CamImuSynchronizer& operator=(const CamImuSynchronizer&) = delete;

  /**
   * @brief configure
   */
  void configure(Config& config, int level);

 private:
  bool is_polling_{false};
  ros::NodeHandle pnh_;
  ImuPtr imu_;
  std::vector<CamPtr> cameras_;
  boost::shared_ptr<boost::thread> img_poll_thread_;
  dynamic_reconfigure::Server<flea3::Flea3DynConfig> cam_cfg_server_;

  void pollImages();
  void startPoll();
  void stopPoll();
  void configureCameras(Config& config);
};

}  // namespace cam_imu_sync

#endif  // CAMIMUSYNC_CAMIMUSYNCHRONIZER_H
