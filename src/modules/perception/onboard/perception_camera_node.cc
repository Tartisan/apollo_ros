#include <ros/ros.h>
#include "modules/perception/onboard/component/fusion_camera_detection_component.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "perception_camera");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  using apollo::perception::onboard::FusionCameraDetectionComponent;
  FusionCameraDetectionComponent node(nh, private_nh);
  ros::spin();
  return 0;
}