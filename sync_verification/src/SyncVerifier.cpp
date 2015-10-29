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

#include <cstdio>
#include <iostream>
#include <algorithm>

#include <boost/foreach.hpp>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen_conversions/eigen_msg.h>

#include <sync_verification/SyncVerifier.h>

using namespace std;
using namespace Eigen;

namespace fd = feature_detector;
namespace ft = feature_tracker;
namespace vu = vision_utils;

namespace sync_verify {

bool SyncVerifier::initialize() {
  ROS_INFO("Load ROS parameters.");
  if (!loadParameters())
    return false;
}

bool SyncVerifier::loadParameters() {
  //std::string ns("/sync_test/");
  std::string ns(ros::this_node::getName()+"/");

  if (!nh.getParam("input_bagfile_name", read_bagname))
    return false;
  if (!nh.getParam("input_imu_topic", imu_topic))
    return false;
  if (!nh.getParam("input_img_topic", img_topic))
    return false;

  double fx, fy, cx, cy, skew;
  if (!nh.getParam("fx", fx))
    return false;
  if (!nh.getParam("fy", fy))
    return false;
  if (!nh.getParam("cx", cx))
    return false;
  if (!nh.getParam("cy", cy))
    return false;
  if (!nh.getParam("skew", skew))
    return false;
  cam_intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  cam_intrinsic.at<double>(0, 0) = fx;
  cam_intrinsic.at<double>(1, 1) = fy;
  cam_intrinsic.at<double>(0, 2) = cx;
  cam_intrinsic.at<double>(1, 2) = cy;
  cam_intrinsic.at<double>(0, 1) = skew;
  cam_intrinsic.at<double>(2, 2) = 1.0f;

  double roll, pitch, yaw;
  if (!nh.getParam("ic_offset_roll", roll))
    return false;
  if (!nh.getParam("ic_offset_pitch", pitch))
    return false;
  if (!nh.getParam("ic_offset_yaw", yaw))
    return false;
  offset_R = AngleAxisd(yaw, Vector3d::UnitZ())*
    AngleAxisd(pitch, Vector3d::UnitY())*
    AngleAxisd(roll, Vector3d::UnitZ());

  if (!f_detector.populate(ns))
    return false;
  if (!f_tracker.populate(ns))
    return false;

  nh.param<string>("output_bagfile_name",
      write_bagname, "alignment_results.bag");
  nh.param<string>("output_imu_ang_vel_topic",
      imu_ang_vel_topic, "/sync/imu");
  nh.param<string>("output_cam_ang_vel_topic",
      raw_cam_ang_vel_topic, "/sync/raw_cam");
  nh.param<string>("output_aligned_cam_ang_vel_topic",
      shifted_cam_ang_vel_topic, "/sync/shifted_cam");

  nh.param<double>("ransac_threshold",
      ransac_th, 3);
  nh.param<double>("ransac_confidence",
      ransac_confidence, 0.99);

  nh.param<bool>("show_tracking_results",
      show_tracking_results, true);

  return true;
}

void SyncVerifier::outlierRejection() {
  for (int i = 0; i < cam_ang_vel.size()-1; ++i) {
    int match_index = cam_ang_vel.size()-1;
    // Find the corresponding interval within the IMU data
    // sequence for the current cam angular velocity
    for (int j = 0; j < imu_ang_vel.size(); ++j) {
      if (imu_ang_vel[j].time > cam_ang_vel[i].time) {
        match_index = j;
        break;
      }
    }
    // If outlier in angular velocity computed from images is found,
    //  just simply set the wrong angular velocity to the nearest
    //  IMU angular velocity.
    if ((cam_ang_vel[i].data-imu_ang_vel[match_index].data).squaredNorm() > 1.0)
      cam_ang_vel[i].data = imu_ang_vel[match_index].data;
  }
  return;
}

void SyncVerifier::computeJacobian(VectorXd& jacobian) {

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

void SyncVerifier::computeResidual(const double& time_shift,
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
    for (int j = 0; j < imu_ang_vel.size(); ++j) {
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
    }

    residual.segment<3>(3*i) =
      intp_imu_ang_vel-cam_ang_vel[i].data;
  }
  return;
}

void SyncVerifier::computeTimeDiff() {
  double damping = 1e-5;

  VectorXd jacobian;
  VectorXd prev_residual;
  VectorXd curr_residual;

  double prev_error = 0;
  double curr_error = 0;

  // Compute the initial cost
  computeResidual(0.0, curr_residual);
  curr_error = curr_residual.squaredNorm();
  prev_residual = curr_residual;
  prev_error = curr_error;
  //ROS_INFO("Initial cost: %f", prev_error);
  //getchar();

  for (int outer_cntr = 0; outer_cntr < 50; ++outer_cntr) {
    double time_shift = 0.0;
    computeJacobian(jacobian);


    for (int inner_cntr = 0; inner_cntr < 100; ++inner_cntr) {
      // Compute the shift in time
      VectorXd time_shift_vec = (jacobian.transpose()*prev_residual) /
        (jacobian.squaredNorm()*(1.0+damping));
      time_shift = -time_shift_vec(0);
      //ROS_INFO("Current step: %f", time_shift);

      // Compute the residual error with the updated time shift
      computeResidual(time_shift, curr_residual);
      curr_error = curr_residual.squaredNorm();
      //ROS_INFO("Damping factor: %f", damping);
      //ROS_INFO("Current cost: %f\n", curr_error);

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

    // Stop the optimization if the update is small
    if (abs(time_shift) < 1e-6)
      break;
  }

  // Output the optimized time shift
  ROS_INFO("Time Shift = %f",
      cam_imu[0].header.stamp.toSec()-cam_ang_vel[0].time);

  return;
}

void SyncVerifier::showImgs() {
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
  cv::waitKey(5);
  return;
}

bool SyncVerifier::computeAngularVelocity(
    const sensor_msgs::Image::ConstPtr& mptr,
    Eigen::Vector3d& ang_vel) {
  // See if the previous image is set
  if (prev_img_ptr) {
    prev_features.clear();
    curr_features.clear();
    vector<unsigned char> tracked_flags;
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

    // Show tracking results
    if (show_tracking_results)
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

void SyncVerifier::writeOutput() {
  // Create the shifted IMU msgs
  shifted_cam_imu.resize(cam_ang_vel.size());
  for (int i = 0; i < cam_imu.size(); ++i) {
    shifted_cam_imu[i].header.stamp = ros::Time(cam_ang_vel[i].time);
    tf::vectorEigenToMsg(cam_ang_vel[i].data,
        shifted_cam_imu[i].angular_velocity);
  }

  // Write the results to a new bagfile
  rosbag::Bag write_bag(write_bagname, rosbag::bagmode::Write);
  for (int i = 0; i < imu_imu.size(); ++i) {
    write_bag.write(imu_ang_vel_topic,
        imu_imu[i].header.stamp, imu_imu[i]);
  }
  for (int i = 0; i < cam_imu.size(); ++i) {
    write_bag.write(raw_cam_ang_vel_topic,
        cam_imu[i].header.stamp, cam_imu[i]);
  }
  for (int i = 0; i < shifted_cam_imu.size(); ++i) {
    write_bag.write(shifted_cam_ang_vel_topic,
        shifted_cam_imu[i].header.stamp, shifted_cam_imu[i]);
  }

  write_bag.close();
  return;
}

void SyncVerifier::run() {

  if (show_tracking_results) {
    cv::namedWindow("Tracking Results");
    cv::waitKey(10);
  }

  // Topics we need
  vector<string> topics(0);
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
              Vector3d(
                mptr->angular_velocity.x,
                mptr->angular_velocity.y,
                mptr->angular_velocity.z)));
      }

    } else if (!m.getTopic().compare(img_topic)) {
      sensor_msgs::Image::ConstPtr mptr =
        m.instantiate<sensor_msgs::Image>();

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

  // Filter the outliers
  ROS_INFO("Remove outliers in the computed angular velocity.");
  outlierRejection();

  // Compute the time delay
  ROS_INFO("Compute time shift between CAM and IMU msgs.");
  computeTimeDiff();

  // Write output to a bagfile
  ROS_INFO("Write output data to %s.", write_bagname.c_str());
  writeOutput();

  return;
}
} // End name space
