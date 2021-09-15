#include <ros/ros.h>
#include "fusion_camera_detection.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "FusionCameraDetection");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  apollo::perception::onboard::FusionCameraDetection node(nh, private_nh);
  ros::spin();
  return 0;
}