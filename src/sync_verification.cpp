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

#include <string>
#include <vector>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <vision_utils/VisionUtils.h>
#include <feature_detector/ShiFeatureDetector.h>
#include <feature_tracker/KltFeatureTracker.h>

using namespace std;

int main(int argc, char **argv) {
  // Bagfile names to read and write
  std::string read_bagname = "read_bagname.bag";
  std::string write_bagname = "write_bagname.bag";

  // Specify the topices
  string imu_topic = "/sync/imu";
  string left_img_topic = "/sync/left/image_raw";
  string right_img_topic = "/sync/right/image_raw";

  vector<string> topics(0);
  topics.push_back(imu_topic);
  topics.push_back(left_img_topic);
  topics.push_back(right_img_topic);

  string imu_angular_vel_topic = "/sync/imu_imu";
  string cam_angular_vel_topic = "/sync/cam_imu";

  vector<sensor_msgs::Imu> imu_imu(0);
  vector<sensor_msgs::Imu> cam_imu(0);

  // Open the bagfiles
  rosbag::Bag read_bag(read_bagname, rosbag::bagmode::Read);
  rosbag::View view(read_bag, rosbag::TopicQuery(topics));

  // Loop through the recorded msgs
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    if (!m.getTopic().compare(imu_topic)) {
      sensor_msgs::Imu::ConstPtr mptr =
        m.instantiate<sensor_msgs::Imu>();

      if (mptr != NULL) {
        imu_imu.push_back(*mptr);
      }

    } else if (!m.getTopic().compare(left_img_topic)) {
      sensor_msgs::Image::ConstPtr mptr =
        m.instantiate<sensor_msgs::Image>();

      // TODO: process the left images

    } else if (!m.getTopic().compare(right_img_topic)) {
      sensor_msgs::Image::ConstPtr mptr =
        m.instantiate<sensor_msgs::Image>();

      // TODO: process the right images

    }
  }

  // Close the bag for reading
  read_bag.close();

  // Write the results to a new bagfile

  return 0;
}
