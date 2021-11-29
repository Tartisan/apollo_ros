#include "fusion_camera_detection_component.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "FusionCameraDetectionComponent");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  using apollo::perception::onboard::FusionCameraDetectionComponent;
  FusionCameraDetectionComponent node(nh, private_nh);
  ros::spin();
  return 0;
}