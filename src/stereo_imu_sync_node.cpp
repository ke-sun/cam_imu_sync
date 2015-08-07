#include <ros/ros.h>
#include <ros/node_handle.h>
#include <boost/bind.hpp>
#include <imu_3dm_gx4/imu_ros_base.hpp>
#include <bluefox2/stereo_node.h>

using namespace imu_3dm_gx4;
using namespace bluefox2;

void imuTimerCallback(ImuRosBase* imu, const ros::TimerEvent& e) {
  imu->requestIMUOnce();
  return;
}

void filterTimerCallback(ImuRosBase* imu, const ros::TimerEvent& e) {
  imu->requestFilterOnce();
  return;
}

void stereoTimerCallback(StereoNode* stereo, const ros::TimerEvent& e) {
  stereo->AcquireOnce();
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_imu_sync");
  ros::NodeHandle nh("~");

  std::string device;
  bool verbose;
  int imu_rate, filter_rate;
  double stereo_rate;

  // Load parameters from launch file
  nh.param<std::string>("device", device, "/dev/ttyACM0");
  nh.param<bool>("verbose", verbose, false);
  nh.param<int>("imu_rate", imu_rate, 100);
  nh.param<int>("filter_rate", filter_rate, 100);
  nh.param<double>("stereo_rate", stereo_rate, 20.0);

  // New instance of the IMU
  ImuRosBase imu(nh, device, verbose);
  if (!imu.initialize()) return -1;

  // New instance of the stereo camera
  StereoNode stereo(nh);

  // Create timers to trigger IMU and Stereo cameras
  ros::Timer imu_timer = nh.createTimer(
      ros::Duration(1.0/static_cast<double>(imu_rate)),
      boost::bind(&imuTimerCallback, &imu, _1));
  ros::Timer filter_timer = nh.createTimer(
      ros::Duration(1.0/static_cast<double>(filter_rate)),
      boost::bind(&filterTimerCallback, &imu, _1));
  ros::Timer stereo_timer = nh.createTimer(
      ros::Duration(1.0/stereo_rate),
      boost::bind(&stereoTimerCallback, &stereo, _1));

  // Read data
  while (ros::ok()) {
    ros::spinOnce();
    imu.runOnce();
  }

  // Disconnect the device
  imu.disconnect();

  return 0;
}
