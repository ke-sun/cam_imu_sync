/*
 * Copyright [2015] [Ke Sun sunke.polyu@gmail.com]
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

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <vision_utils/VisionUtils.h>
#include <feature_detector/ShiFeatureDetector.h>
#include <feature_tracker/KltFeatureTracker.h>

namespace sync_verify {
/*
 * @brief SyncVerifier
 *    This class is developed in order to compute the
 *    time shift between the images and IMU msgs using
 *    angular velocity. It can be used as a tool to
 *    verify the synchronization between the camera and
 *    IMU.
 */
class SyncVerifier {
  public:
    SyncVerifier(const ros::NodeHandle& n):
      nh(n) {
      return;
    };
    ~SyncVerifier() {
      return;
    }

    /*
     * @brief initialize Initialize the verifier.
     *    Mostly, load parameters from ros parameter
     *    server.
     * @return True if successfully initialized.
     */
    bool initialize();

    /*
     * @brief run Start computing time shift of the data
     */
    void run();

  private:
    // A simple structure to store stamped angular velocity
    struct StampedAngularVelocity {
      double time;
      Eigen::Vector3d data;

      StampedAngularVelocity():
        time(0.0),
        data(Eigen::Vector3d::Zero()) {
        return;
      }
      StampedAngularVelocity(const double& t,
          const Eigen::Vector3d& ang_vel):
        time(t),
        data(ang_vel) {
        return;
      }
    };

    // Disable copy and assign contructor
    SyncVerifier(const SyncVerifier&);
    SyncVerifier operator=(const SyncVerifier&);

    // Load parameters
    bool loadParameters();

    // Compute angular velocity from images
    bool computeAngularVelocity(
        const sensor_msgs::Image::ConstPtr& mptr,
        Eigen::Vector3d& ang_vel);

    // Show the tracked features on images.
    //  (for debug only)
    void showImgs();

    // Reject the outliers in computed
    //  angular velocity from the images.
    void outlierRejection();

    // Compute the Jacobian and Residual during
    //  the optimization for time shift.
    void computeJacobian(Eigen::VectorXd& jacobian);
    void computeResidual(const double& time_shift,
        Eigen::VectorXd& residual);

    // Compute the time difference bewteen camera
    //  and IMU msgs.
    void computeTimeDiff();

    // Write data to a ros bagfile
    void writeOutput();

    // Ros node handle
    ros::NodeHandle nh;

    // Names of bagfiles for input and output
    std::string read_bagname;
    std::string write_bagname;

    // Topics of interest
    // input topic
    std::string imu_topic;
    std::string img_topic;
    // output topic
    std::string imu_ang_vel_topic;
    std::string raw_cam_ang_vel_topic;
    std::string shifted_cam_ang_vel_topic;

    // Calibration paramters
    cv::Mat cam_intrinsic;
    Eigen::Matrix3d offset_R;

    // Related to visual rotation estimation
    // Feature detector and tracker
    feature_detector::ShiFeatureDetector f_detector;
    feature_tracker::KltFeatureTracker f_tracker;
    // Ptr to previous image and current image
    cv_bridge::CvImagePtr prev_img_ptr;
    cv_bridge::CvImagePtr curr_img_ptr;
    // Set of previous features and current features
    std::vector<cv::Point2f> prev_features;
    std::vector<cv::Point2f> curr_features;
    // RanSAC settings
    double ransac_th;
    double ransac_confidence;
    // If show tracking results
    bool show_tracking_results;

    // Raw and processed messages
    std::vector<sensor_msgs::Imu> imu_imu;
    std::vector<sensor_msgs::Imu> cam_imu;
    std::vector<sensor_msgs::Imu> shifted_cam_imu;
    std::vector<StampedAngularVelocity> imu_ang_vel;
    std::vector<StampedAngularVelocity> cam_ang_vel;

};
}
