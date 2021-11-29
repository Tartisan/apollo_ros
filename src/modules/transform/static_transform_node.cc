#include <ros/ros.h>
#include "static_transform_component.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "transform");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  apollo::transform::StaticTransformComponent node;
  node.Init();
  ros::spin();
  return 0;
}