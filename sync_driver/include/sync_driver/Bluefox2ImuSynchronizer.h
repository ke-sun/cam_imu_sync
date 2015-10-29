#ifndef CAMIMUSYNC_BLUEFOX2IMUSYNCHRONIZER_H
#define CAMIMUSYNC_BLUEFOX2IMUSYNCHRONIZER_H

#include "sync_driver/CamImuSynchronizer.h"

#include <imu_vn_100/imu_ros_base.h>
#include <bluefox2/bluefox2_ros.h>
#include <bluefox2/Bluefox2DynConfig.h>

namespace cam_imu_sync {

using imu_vn_100::ImuRosBase;
using bluefox2::Bluefox2Ros;
using bluefox2::Bluefox2DynConfig;

class Bluefox2ImuSynchronizer
    : public CamImuSynchronizer<ImuRosBase, Bluefox2Ros, Bluefox2DynConfig> {
 public:
  Bluefox2ImuSynchronizer(const ros::NodeHandle& pnh, int num_cameras);

  void configure() override;
};

}  // namespace cam_imu_sync

#endif  // CAMIMUSYNC_BLUEFOX2IMUSYNCHRONIZER_H
