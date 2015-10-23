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

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include <vision_utils/VisionUtils.h>
#include <feature_detector/ShiFeatureDetector.h>
#include <feature_tracker/KltFeatureTracker.h>

namespace fd = feature_detector;
namespace ft = feature_tracker;
namespace vu = vision_utils;

// Names of bagfiles for input and output
std::string read_bagname;
std::string write_bagname;
// Topics of interest
std::string imu_topic;
std::string img_topic;
std::string imu_angular_vel_topic;
std::string cam_angular_vel_topic;
// Calibration paramters
cv::Mat cam_intrinsic;
Eigen::Matrix3d offset_R;
// Related to visual rotation estimation
cv_bridge::CvImagePtr prev_img_ptr;
cv_bridge::CvImagePtr curr_img_ptr;
std::vector<cv::Point2f> prev_features;
std::vector<cv::Point2f> curr_features;
std::vector<unsigned char> tracked_flags;
fd::ShiFeatureDetector f_detector;
ft::KltFeatureTracker f_tracker;
double ransac_th;
double ransac_confidence;
// Store the output data
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
std::vector<sensor_msgs::Imu> imu_imu(0);
std::vector<sensor_msgs::Imu> cam_imu(0);
std::vector<StampedAngularVelocity> imu_ang_vel(0);
std::vector<StampedAngularVelocity> cam_ang_vel(0);



bool loadParameters() {
  std::string ns("/sync_test/");

  if (!ros::param::get(ns+"input_bagfile_name", read_bagname))
    return false;
  if (!ros::param::get(ns+"output_bagfile_name", write_bagname))
    return false;
  if (!ros::param::get(ns+"input_imu_topic", imu_topic))
    return false;
  if (!ros::param::get(ns+"input_img_topic", img_topic))
    return false;
  if (!ros::param::get(ns+"output_imu_angular_vel_topic", imu_angular_vel_topic))
    return false;
  if (!ros::param::get(ns+"output_cam_angular_vel_topic", cam_angular_vel_topic))
    return false;

  if (!ros::param::get(ns+"ransac_threshold", ransac_th))
    return false;
  if (!ros::param::get(ns+"ransac_confidence", ransac_confidence))
    return false;

  double fx, fy, cx, cy, skew;
  if (!ros::param::get(ns+"fx", fx))
    return false;
  if (!ros::param::get(ns+"fy", fy))
    return false;
  if (!ros::param::get(ns+"cx", cx))
    return false;
  if (!ros::param::get(ns+"cy", cy))
    return false;
  if (!ros::param::get(ns+"skew", skew))
    return false;
  cam_intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  cam_intrinsic.at<double>(0, 0) = fx;
  cam_intrinsic.at<double>(1, 1) = fy;
  cam_intrinsic.at<double>(0, 2) = cx;
  cam_intrinsic.at<double>(1, 2) = cy;
  cam_intrinsic.at<double>(0, 1) = skew;
  cam_intrinsic.at<double>(2, 2) = 1.0f;

  std::cout << "fx: " << fx << std::endl;
  std::cout << "fy: " << fy << std::endl;
  std::cout << "cx: " << cx << std::endl;
  std::cout << "cy: " << cy << std::endl;
  std::cout << "sk: " << skew << std::endl;

  double roll, pitch, yaw;
  if (!ros::param::get(ns+"ic_offset_roll", roll))
    return false;
  if (!ros::param::get(ns+"ic_offset_pitch", pitch))
    return false;
  if (!ros::param::get(ns+"ic_offset_yaw", yaw))
    return false;
  offset_R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());

  if (!f_detector.populate(ns))
    return false;
  if (!f_tracker.populate(ns))
    return false;

  return true;
}

void medianFilter() {
  // The window of the median filter is hard-coded
  // to be 5.
  for (int i = 2; i < cam_imu.size()-2; ++i) {
    std::vector<double> med_x_candidate(3);
    std::vector<double> med_y_candidate(3);
    std::vector<double> med_z_candidate(3);

    med_x_candidate[0] = std::min(
        cam_ang_vel[i-2].data(0), cam_ang_vel[i-1].data(0));
    med_y_candidate[0] = std::min(
        cam_ang_vel[i-2].data(1), cam_ang_vel[i-1].data(1));
    med_z_candidate[0] = std::min(
        cam_ang_vel[i-2].data(2), cam_ang_vel[i-1].data(2));

    med_x_candidate[1] = std::min(
        cam_ang_vel[i].data(0), cam_ang_vel[i+1].data(0));
    med_y_candidate[1] = std::min(
        cam_ang_vel[i].data(1), cam_ang_vel[i+1].data(1));
    med_z_candidate[1] = std::min(
        cam_ang_vel[i].data(2), cam_ang_vel[i+1].data(2));

    med_x_candidate[2] = cam_ang_vel[i+2].data(0);
    med_y_candidate[2] = cam_ang_vel[i+2].data(1);
    med_z_candidate[2] = cam_ang_vel[i+2].data(2);

    cam_ang_vel[i].data(0) = med_x_candidate[0];
    cam_ang_vel[i].data(1) = med_y_candidate[0];
    cam_ang_vel[i].data(2) = med_z_candidate[0];

    for (int j = 1; j < 3; ++j) {
      if (cam_ang_vel[i].data(0) > med_x_candidate[j])
        cam_ang_vel[i].data(0) = med_x_candidate[j];
      if (cam_ang_vel[i].data(1) > med_y_candidate[j])
        cam_ang_vel[i].data(1) = med_y_candidate[j];
      if (cam_ang_vel[i].data(2) > med_z_candidate[j])
        cam_ang_vel[i].data(2) = med_z_candidate[j];
    }
  }
  // TODO: Take care of the first two and last two samples
  return;
}

void computeJacobian(Eigen::VectorXd& jacobian) {

  // Resize the dimension of Jacobian if necessary
  if (jacobian.rows()!= cam_ang_vel.size()*3)
    jacobian.resize(cam_imu.size()*3);

  for (int i = 1; i < cam_ang_vel.size()-1; ++i) {
    // Using linear interpolation
    double prev_dt = cam_ang_vel[i].time - cam_ang_vel[i-1].time;
    double next_dt = cam_ang_vel[i+1].time - cam_ang_vel[i].time;

    jacobian.segment<3>(3*i) =
      (cam_ang_vel[i+1].data-cam_ang_vel[i-1].data)/(prev_dt+next_dt);
  }

  // Handle the first sample point
  jacobian.segment<3>(0) =
    (cam_ang_vel[1].data-cam_ang_vel[0].data)/
    (cam_ang_vel[1].time-cam_ang_vel[0].time);

  // Handle the last sample point
  jacobian.segment<3>(cam_ang_vel.size()*3-3) =
    (cam_ang_vel[cam_ang_vel.size()-1].data-cam_ang_vel[cam_ang_vel.size()-2].data)/
    (cam_ang_vel[cam_ang_vel.size()-1].time-cam_ang_vel[cam_ang_vel.size()-2].time);
  return;
}

void computeResidual(const double& time_shift,
    Eigen::VectorXd& residual) {
  // Resize the dimension of residual if necessary
  if (residual.rows()!= cam_ang_vel.size()*3)
    residual.resize(cam_imu.size()*3);

  // Compute the residual at each time instance
  // where angular velocity from camera is avaiable
  for (int i = 0; i < cam_ang_vel.size(); ++i) {
    int match_index = -1;
    // Find the corresponding interval within the IMU data
    // sequence for the current cam angular velocity
    for (int j = 0; j < imu_imu.size(); ++i) {
      if (imu_ang_vel[j].time > cam_ang_vel[i].time+time_shift) {
        match_index = j;
        break;
      }
    }

    Eigen::Vector3d intp_imu_ang_vel;
    double interval, epsilon;
    // Compute the residual
    if (match_index != 0 && match_index != -1) {
      // Deal with the case when the data from camera
      // is within an interval of IMU data
      interval = imu_ang_vel[match_index].time-
        imu_ang_vel[match_index-1].time;
      epsilon = imu_ang_vel[match_index].time-
        cam_ang_vel[i].time-time_shift;
      intp_imu_ang_vel = imu_ang_vel[match_index].data -
        (imu_ang_vel[match_index].data-
         imu_ang_vel[match_index-1].data)*
        (epsilon/interval);
      residual.segment<3>(3*i) = intp_imu_ang_vel-cam_ang_vel[i].data;
    } else if (match_index == 0) {
      // Deal with the case when the data from camera
      // is before the first IMU data
      interval = imu_ang_vel[1].time-
        imu_ang_vel[0].time;
      epsilon = imu_ang_vel[0].time-
        cam_ang_vel[i].time-time_shift;
      intp_imu_ang_vel = imu_ang_vel[0].data -
        (imu_ang_vel[1].data-imu_ang_vel[0].data)*
        (epsilon/interval);
      residual.segment<3>(0) = intp_imu_ang_vel-cam_ang_vel[i].data;
    } else {
      // Deal with the case when the data from camera
      // is after the last IMU data
      interval = imu_ang_vel[imu_ang_vel.size()-1].time-
        imu_ang_vel[imu_ang_vel.size()-2].time;
      epsilon = cam_ang_vel[i].time+time_shift -
        imu_ang_vel[imu_ang_vel.size()-1].time;
      intp_imu_ang_vel = imu_ang_vel[imu_ang_vel.size()-1].data +
        (imu_ang_vel[imu_ang_vel.size()-1].data-
         imu_ang_vel[imu_ang_vel.size()-2].data)*
        (epsilon/interval);
      residual.segment<3>(3*cam_ang_vel.size()-3) =
        intp_imu_ang_vel-cam_ang_vel[i].data;
    }
  }
  return;
}

void computeTimeDelay() {
  double damping = 1e-5;

  Eigen::VectorXd jacobian;
  Eigen::VectorXd prev_residual;
  Eigen::VectorXd curr_residual;

  double prev_error = 0;
  double curr_error = 0;

  // Remove the outliers in the cam data
  medianFilter();

  computeResidual(0.0, curr_residual);
  curr_error = curr_residual.squaredNorm();
  prev_residual = curr_residual;
  prev_error = curr_error;

  for (int outer_cntr = 0; outer_cntr < 50; ++outer_cntr) {
    computeJacobian(jacobian);
    // Try compute the shift in time
    for (int inner_cntr = 0; inner_cntr < 100; ++inner_cntr) {
      Eigen::VectorXd time_shift_vec = (jacobian.transpose()*prev_residual) /
        (jacobian.squaredNorm()*(1.0+damping));
      double time_shift = time_shift_vec(0);
      computeResidual(time_shift, curr_residual);
      curr_error = curr_residual.squaredNorm();
      // Check the current residual
      if (curr_error >= prev_error) {
        damping *= 5.0;
        continue;
      } else {
        damping = damping/5.0 > 1e-5 ? damping/5.0 : 1e-5;
        prev_error = curr_error;
        prev_residual = curr_residual;
        for (int i = 0; i < cam_ang_vel.size(); ++i) {
          cam_ang_vel[i].time += time_shift;
        }
        break;
      }
    }
  }

  // Output the optimized time shift
  ROS_INFO("Time Shift = %f",
      cam_imu[0].header.stamp.toSec()-cam_ang_vel[0].time);

  return;
}

void showImgs() {
  cv::Mat drawing_pad;
  cv::cvtColor(curr_img_ptr->image, drawing_pad, CV_GRAY2BGR);

  // Draw current features on the current frame
  for (int i = 0; i < curr_features.size(); ++i) {
    cv::circle(drawing_pad,
        curr_features[i], 3, cv::Scalar(0, 255, 0), -1);
  }

  // Draw lines between previous features and
  // current features
  for (int i = 0; i < curr_features.size(); ++i) {
    cv::line(drawing_pad, prev_features[i],
        curr_features[i], cv::Scalar(255, 0, 0));
  }

  // Show the drawing results
  cv::imshow("Tracking Results", drawing_pad);
  cv::waitKey(10);
  return;
}

bool computeAngularVelocity(
    const sensor_msgs::Image::ConstPtr& mptr,
    Eigen::Vector3d& ang_vel) {
  // See if the previous image is set
  if (prev_img_ptr) {
    prev_features.clear();
    curr_features.clear();
    tracked_flags.clear();
    // Copy the current image
    curr_img_ptr = cv_bridge::toCvCopy(mptr,
        sensor_msgs::image_encodings::MONO8);
    // Detect features on the previous image
    f_detector.detect(
        prev_img_ptr->image, prev_features);
    // Tracked the detected features on the current image
    f_tracker.track(
        prev_img_ptr->image, curr_img_ptr->image,
        prev_features, curr_features, tracked_flags);
    // Remove the untracked features
    std::vector<cv::Point2f>::iterator prev_iter =
      prev_features.begin();
    for (int i = 0; i < tracked_flags.size(); ++i) {
      if (tracked_flags[i] == 0) {
        prev_features.erase(prev_iter);
        continue;
      }
      ++prev_iter;
    }
    showImgs();
    // Compute the relative rotation between the two
    // frames by epipolar geometry.
    cv::Mat opencv_R(3, 3, CV_64F);
    cv::Mat opencv_t(3, 1, CV_64F);
    vu::findRTfromE(
        prev_features, curr_features,
        ransac_th, ransac_confidence,
        cam_intrinsic, cam_intrinsic,
        opencv_R, opencv_t);

    // Computer time interval between two frames
    double dt = curr_img_ptr->header.stamp.toSec() -
      prev_img_ptr->header.stamp.toSec();

    // Conver the opencv type to eigen
    Eigen::Matrix3d dR;
    cv::cv2eigen(opencv_R, dR);
    dR.transposeInPlace();

    // Compute angular velocity
    Eigen::AngleAxisd angle_axis(dR);
    Eigen::Vector3d axis = angle_axis.axis();
    double angle = angle_axis.angle();
    double dot_angle = angle/dt;

    Eigen::Matrix3d interim_R =
      Eigen::AngleAxisd(angle/2.0, axis).toRotationMatrix();
    Eigen::Vector3d new_axis =
      offset_R*interim_R.transpose()*axis;
    ang_vel = dot_angle*new_axis;

    // Update previous image
    prev_img_ptr = curr_img_ptr;
    return true;
  } else {
    // Set the first image
    prev_img_ptr = cv_bridge::toCvCopy(mptr,
        sensor_msgs::image_encodings::MONO8);
    imshow("Tracking Results", prev_img_ptr->image);
    cv::waitKey(0);
    return false;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sync_text");
  ros::NodeHandle nh;

  ROS_INFO("Loading ROS parameters.");
  if (!loadParameters()) {
    ROS_ERROR("Cannot load parameters.");
    return -1;
  }

  cv::namedWindow("Tracking Results");
  cv::waitKey(10);

  // Topics we need
  std::vector<std::string> topics(0);
  topics.push_back(imu_topic);
  topics.push_back(img_topic);

  // Open the bagfile to be read
  ROS_INFO("Loading bag file.");
  rosbag::Bag read_bag(read_bagname, rosbag::bagmode::Read);
  rosbag::View view(read_bag, rosbag::TopicQuery(topics));

  // Process the recorded msgs
  ROS_INFO("Process data.");
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    if (!m.getTopic().compare(imu_topic)) {
      sensor_msgs::Imu::ConstPtr mptr =
        m.instantiate<sensor_msgs::Imu>();

      if (mptr != NULL) {
        imu_imu.push_back(*mptr);
        imu_ang_vel.push_back(StampedAngularVelocity(
              mptr->header.stamp.toSec(),
              Eigen::Vector3d(
                mptr->angular_velocity.x,
                mptr->angular_velocity.y,
                mptr->angular_velocity.z)));
      }

    } else if (!m.getTopic().compare(img_topic)) {
      sensor_msgs::Image::ConstPtr mptr =
        m.instantiate<sensor_msgs::Image>();

      // TODO: process the images
      if (mptr != NULL) {
        Eigen::Vector3d ang_vel;
        bool ready = computeAngularVelocity(mptr, ang_vel);
        if (ready) {
          double dt = (prev_img_ptr->header.stamp -
              curr_img_ptr->header.stamp).toSec();
          sensor_msgs::Imu new_imu;
          new_imu.header.stamp = prev_img_ptr->header.stamp +
            ros::Duration(dt/2.0);
          tf::vectorEigenToMsg(ang_vel, new_imu.angular_velocity);
          cam_imu.push_back(new_imu);
          cam_ang_vel.push_back(StampedAngularVelocity(
                new_imu.header.stamp.toSec(), ang_vel));
        }
      }

    }
  }

  // Close the bag for reading
  read_bag.close();

  // Compute the time delay
  computeTimeDelay();

  // Write the results to a new bagfile
  //ROS_INFO("Write results into a new bagfile.");
  //rosbag::Bag write_bag(write_bagname, rosbag::bagmode::Write);
  //for (int i = 0; i < imu_imu.size(); ++i) {
  //  write_bag.write(imu_angular_vel_topic,
  //      imu_imu[i].header.stamp, imu_imu[i]);
  //}
  //for (int i = 0; i < cam_imu.size(); ++i) {
  //  write_bag.write(cam_angular_vel_topic,
  //      cam_imu[i].header.stamp, cam_imu[i]);
  //}

  //write_bag.close();

  return 0;
}
