#ifndef CAMIMUSYNC_CHAMELEON3IMUSYNCHRONIZER_H
#define CAMIMUSYNC_CHAMELEON3IMUSYNCHRONIZER_H

#include "cam_imu_sync/CamImuSynchronizer.h"
#include "imu_vn_100/imu_ros_base.h"
#include "flea3/flea3_ros.h"
#include "flea3/Flea3DynConfig.h"

namespace cam_imu_sync {

using imu_vn_100::ImuRosBase;
using flea3::Flea3Ros;
using flea3::Flea3DynConfig;

class Chameleon3ImuSynchronizer
    : public CamImuSynchronizer<ImuRosBase, Flea3Ros, Flea3DynConfig> {
 public:
  Chameleon3ImuSynchronizer(const ros::NodeHandle& pnh, int num_cameras);

  void configure() override;
};

}  // namespace cam_imu_sync

#endif  // CAMIMUSYNC_CHAMELEON3IMUSYNCHRONIZER_H
