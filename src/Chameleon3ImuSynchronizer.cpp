#include "cam_imu_sync/Chameleon3ImuSynchronizer.h"

namespace cam_imu_sync {

Chameleon3ImuSynchronizer::Chameleon3ImuSynchronizer(const ros::NodeHandle& pnh,
                                                     int num_cameras)
    : CamImuSynchronizer<ImuRosBase, Flea3Ros, Flea3DynConfig>(pnh, 2) {}

void Chameleon3ImuSynchronizer::configure() {
  Flea3DynConfig config;
  // Turn off auto shutter
  config.auto_shutter = false;
  config.shutter_ms = 10;
  config.trigger_source = flea3::Flea3Dyn_sc_gpio_2;
  config.trigger_polarity = flea3::Flea3Dyn_ts_high;

  configureEachCamera(config);
  for (auto& cam : cameras_) {
    cam->Start();
  }
}

}  // namespace cam_imu_sync
