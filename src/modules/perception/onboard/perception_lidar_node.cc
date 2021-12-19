#include "modules/perception/onboard/component/detection_component.h"
#include "modules/perception/onboard/component/fusion_component.h"
#include "modules/perception/onboard/component/recognition_component.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

apollo::perception::onboard::DetectionComponent detection;
apollo::perception::onboard::FusionComponent recognition;
apollo::perception::onboard::RecognitionComponent fusion;

ros::Subscriber reader;
ros::Publisher writer;

void CallbackVelodyne64(
    const sensor_msgs::PointCloud2::ConstPtr &input_message) {
  bool status = false;
  auto lidar_frame_message = std::make_shared<LidarFrameMessage>();
  status = detection.Proc(input_message, lidar_frame_message);
  if (!status) {
    AERROR << "Lidar detection failed!";
    return;
  }
  auto sensor_frame_message = std::make_shared<SensorFrameMessage>();
  status = recognition.Proc(lidar_frame_message, sensor_frame_message);
  if (!status) {
    AERROR << "Lidar recognition failed!";
    return;
  }
  auto output_message = std::make_shared<PerceptionObstacles>();
  status = fusion.Proc(sensor_frame_message, output_message);
  if (!status) {
    AERROR << "Lidar fusion failed!";
    return;
  }
  writer.publish(*output_message);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "perception_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  std::string input_channel;
  std::string output_channel;
  nh.param("input_channel", input_channel,
           std::string("/apollo/sensor/velodyne64/compensator/PointCloud2"));
  nh.param("output_channel", output_channel,
           std::string("/apollo/perception/obstacles"));

  reader = nh.subscribe(input_channel, 1, &CallbackVelodyne64);
  writer = nh.advertise<PerceptionObstacles>(output_channel, 1);

  detection.Init(nh, private_nh);
  recognition.Init(nh, private_nh);
  fusion.Init(nh, private_nh);

  ros::spin();
  return 0;
}