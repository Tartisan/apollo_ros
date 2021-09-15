#pragma once

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include "modules/perception/camera/tools/visualizer.h"
#include "modules/perception/camera/app/obstacle_camera_perception.h"

namespace apollo {
namespace perception {
namespace onboard {

class FusionCameraDetection {
public:
  FusionCameraDetection(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~FusionCameraDetection() {}

private:
  void OnReceiveImage(const sensor_msgs::ImageConstPtr &msg,
                      const std::string &camera_name);
  int InitAlgorithmPlugin();
  int InitCameraFrames();
  int InitProjectMatrix();
  int InitMotionService();
  void SetCameraHeightAndPitch();
  bool SetCameraHeight(const std::string &sensor_name,
                       const std::string &params_dir,
                       float default_camera_height, float *camera_height);
  bool LoadExtrinsics(const std::string &yaml_file,
                      Eigen::Matrix4d *camera_extrinsic);
  bool
  GetProjectMatrix(const std::vector<std::string> &camera_names,
                   const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
                   const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
                   Eigen::Matrix3d *project_matrix, double *pitch_diff);
  // static int GetGpuId(const camera::CameraPerceptionInitOptions &options);

private:
  std::vector<std::string> camera_names_{"front_6mm", "front_12mm"};
  std::string config_path_;
  int gpu_id_ = 0;

  // ros parameters
  bool use_camera_short_ = true;
  bool use_camera_long_ = true;
  std::string camera_short_topic_;
  std::string camera_long_topic_;

  // Subscriber
  ros::Subscriber sub_camera_short_;
  ros::Subscriber sub_camera_long_;
  
  // Publisher
  ros::Publisher pub_camera_obs_image_;
  ros::Publisher pub_camera_obs_marker_;

  // camera_height
  std::map<std::string, float> camera_height_map_;

  // camera_pitch_angle_diff
  // the pitch_diff = pitch_narrow - pitch_obstacle
  std::map<std::string, float> name_camera_pitch_angle_diff_map_;

  // pre-allocaated-mem data_provider;
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;

  // map for store params
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;
  std::map<std::string, Eigen::Matrix3f> intrinsic_map_;

  // camera obstacle pipeline
  camera::CameraPerceptionInitOptions camera_perception_init_options_;
  camera::CameraPerceptionOptions camera_perception_options_;
  std::unique_ptr<camera::ObstacleCameraPerception> camera_obstacle_pipeline_;

  // fixed size camera frames
  int frame_capacity_ = 20;
  int frame_id_ = 0;
  std::vector<camera::CameraFrame> camera_frames_;

  // image info.
  int image_width_ = 1920;
  int image_height_ = 1080;
  int image_channel_num_ = 3;
  int image_data_size_ = -1;

  // default camera pitch angle & height
  float default_camera_pitch_ = 0.f;
  float default_camera_height_ = 1.5f;

  // options for DataProvider
  bool enable_undistortion_ = false;

  Eigen::Matrix3d project_matrix_;
  double pitch_diff_ = 0.0;

  // variable for motion service
  base::MotionBufferPtr motion_buffer_;
  const int motion_buffer_size_ = 100;

  // visualization 
  bool enable_visualization_ = true;
  bool write_visual_img_ = false;
  std::string visual_camera_ = "front_6mm";
  camera::Visualizer visualize_;
};

} // namespace onboard
} // namespace perception
} // namespace apollo
