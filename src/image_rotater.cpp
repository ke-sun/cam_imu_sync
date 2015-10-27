#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int rotflag = 0;
ros::Publisher pub_image;

void rot90(cv::Mat& matImage, int rotflag) {
  // 1=CW, 2=CCW, 3=180
  if (rotflag == 1) {
    cv::transpose(matImage, matImage);
    cv::flip(matImage, matImage, 1);  // transpose+flip(1)=CW
  } else if (rotflag == 3) {
    cv::transpose(matImage, matImage);
    cv::flip(matImage, matImage, 0);  // transpose+flip(0)=CCW
  } else if (rotflag == 2) {
    cv::flip(matImage, matImage, -1);  // flip(-1)=180
  } else if (rotflag != 0) {           // if not 0,1,2,3:
  }
}

void ImageCb(const sensor_msgs::ImageConstPtr& image_msg) {
  auto image = cv_bridge::toCvCopy(image_msg)->image;
  rot90(image, rotflag);
  cv_bridge::CvImage dst_msg(image_msg->header, image_msg->encoding, image);
  pub_image.publish(dst_msg.toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_rotater");
  ros::NodeHandle pnh("~");

  pnh.param<int>("rotflag", rotflag, 0);

  pub_image = pnh.advertise<sensor_msgs::Image>("image_rotated", 1);
  ros::Subscriber sub_image = pnh.subscribe("image_raw", 1, &ImageCb);

  ros::spin();
  return 0;
}
