#include <ros/ros.h>
#include "conti_radar_canbus_component.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ContiRadarCanbusComponent");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  apollo::drivers::conti_radar::ContiRadarCanbusComponent node;
  node.Init(nh, private_nh);
  ros::spin();
  return 0;
}