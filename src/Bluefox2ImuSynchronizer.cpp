#include "cam_imu_sync/Bluefox2ImuSynchronizer.h"

namespace cam_imu_sync {

Bluefox2ImuSynchronizer::Bluefox2ImuSynchronizer(const ros::NodeHandle& pnh,
                                                 int num_cameras)
    : CamImuSynchronizer<ImuRosBase, Bluefox2Ros, Bluefox2DynConfig>(pnh, 2) {}

void Bluefox2ImuSynchronizer::configure() {
  Bluefox2DynConfig config;
  // Turn off auto expose
  config.aec = false;
  config.expose_us = 2000;
  config.ctm = bluefox2::Bluefox2Dyn_ctm_on_high_level;

  configureEachCamera(config);
}

}  // namespace cam_imu_sync
