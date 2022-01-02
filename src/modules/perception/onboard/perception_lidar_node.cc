#include "modules/perception/onboard/component/detection_component.h"
#include "modules/perception/onboard/component/fusion_component.h"
#include "modules/perception/onboard/component/recognition_component.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace apollo {
namespace perception {
namespace onboard {

using apollo::perception::PerceptionObstacles;

class FusionLidarDetection {
public:
  FusionLidarDetection(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    detection_.Init(nh, private_nh);
    recognition_.Init(nh, private_nh);
    fusion_.Init(nh, private_nh);

    std::string input_channel;
    std::string output_channel;
    nh.param("input_channel", input_channel,
            std::string("/apollo/sensor/velodyne64/compensator/PointCloud2"));
    nh.param("output_channel", output_channel,
            std::string("/apollo/perception/obstacles"));

    reader_ = nh.subscribe(input_channel, 1, 
        &FusionLidarDetection::CallbackVelodyne64, this);
    // writer_ = nh.advertise<PerceptionObstacles>(output_channel, 1);
  }
  ~FusionLidarDetection() {}

private:
  ros::Subscriber reader_;
  ros::Publisher writer_;
  DetectionComponent detection_;
  RecognitionComponent recognition_;
  FusionComponent fusion_;

private: 
  void CallbackVelodyne64(
      const sensor_msgs::PointCloud2::ConstPtr &input_message) {
    bool status = false;
    auto lidar_frame_message = std::make_shared<LidarFrameMessage>();
    status = detection_.Proc(input_message, lidar_frame_message);
    if (!status) {
      AERROR << "Lidar detection failed!";
      return;
    }
    auto sensor_frame_message = std::make_shared<SensorFrameMessage>();
    status = recognition_.Proc(lidar_frame_message, sensor_frame_message);
    if (!status) {
      AERROR << "Lidar recognition failed!";
      return;
    }
    auto output_message = std::make_shared<PerceptionObstacles>();
    status = fusion_.Proc(sensor_frame_message, output_message);
    if (!status) {
      AERROR << "Lidar fusion failed!";
      return;
    }
    // writer_.publish(*output_message);
  }

};

} // namespace onboard
} // namespace perception
} // namespace apollo


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "perception_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  apollo::perception::onboard::FusionLidarDetection node(nh, private_nh);
  ros::spin();
  return 0;
}