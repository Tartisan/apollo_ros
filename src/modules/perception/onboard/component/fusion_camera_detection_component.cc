#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/onboard/component/fusion_camera_detection_component.h"

namespace apollo {
namespace perception {
namespace onboard {

FusionCameraDetectionComponent::FusionCameraDetectionComponent(
    ros::NodeHandle node, ros::NodeHandle private_nh) {
  // ros parameters
  node.param("use_camera_short", use_camera_short_, true);
  node.param("use_camera_long", use_camera_long_, false);
  node.param("enable_visualization", enable_visualization_, true);
  node.param("write_visual_img", write_visual_img_, false);
  node.param("camera_short_topic", camera_short_topic_,
             std::string("apollo/sensor/camera/front_6mm/image"));
  node.param("camera_long_topic", camera_long_topic_,
             std::string("apollo/sensor/camera/front_12mm/image"));
  // get config file path
  config_path_ = lib::ConfigManager::Instance()->GetConfigPath();
  // subscribe
  sub_camera_short_ = node.subscribe<sensor_msgs::Image>(
      camera_short_topic_, 2,
      std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
                std::placeholders::_1, "front_6mm"));
  sub_camera_long_ = node.subscribe<sensor_msgs::Image>(
      camera_long_topic_, 2,
      std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
                std::placeholders::_1, "front_12mm"));
  // publish
  pub_camera_obs_image_ = node.advertise<sensor_msgs::Image>(
      "/perception/camera/obstacles/image", 1);

  camera_perception_init_options_.gpu_id = gpu_id_;
  camera_perception_init_options_.root_dir =
      config_path_ + "/perception/camera";
  camera_perception_init_options_.conf_file = "obstacle.pt";

  if (InitAlgorithmPlugin() != 0) {
    AERROR << "InitAlgorithmPlugin() failed.";
    return;
  }
  if (InitCameraFrames() != 0) {
    AERROR << "InitCameraFrames() failed.";
    return;
  }
  if (InitProjectMatrix() != 0) {
    AERROR << "InitProjectMatrix() failed.";
    return;
  }
  if (InitMotionService() != 0) {
    AERROR << "InitMotionService() failed.";
    return;
  }

  SetCameraHeightAndPitch();

  Eigen::Matrix4d ex_lidar2imu = Eigen::Matrix4d::Identity();
  // LoadExtrinsics(FLAGS_obs_sensor_intrinsic_path + "/" +
  //                    "velodyne128_novatel_extrinsics.yaml",
  //                &ex_lidar2imu);
  // AINFO << "velodyne128_novatel_extrinsics: " << ex_lidar2imu;
  double pitch_adj_degree = 0.0;
  double yaw_adj_degree = 0.0;
  double roll_adj_degree = 0.0;
  CHECK(visualize_.Init_all_info_single_camera(
      camera_names_, visual_camera_, intrinsic_map_, extrinsic_map_,
      ex_lidar2imu, pitch_adj_degree, yaw_adj_degree, roll_adj_degree,
      image_height_, image_width_));

  // homography_im2car_ = visualize_.homography_im2car(visual_camera_);
  // camera_obstacle_pipeline_->SetIm2CarHomography(homography_im2car_);

  visualize_.cv_imshow_img_ = enable_visualization_;
  if (write_visual_img_) {
    visualize_.write_out_img_ = true;
    visualize_.SetDirectory(config_path_ + "/../../../../debug_output");
  }
}

int FusionCameraDetectionComponent::InitAlgorithmPlugin() {
  camera_obstacle_pipeline_.reset(new camera::ObstacleCameraPerception);
  if (!camera_obstacle_pipeline_->Init(camera_perception_init_options_)) {
    AERROR << "camera_obstacle_pipeline_->Init() failed";
    return -1;
  }
  AINFO << "camera_obstacle_pipeline_->Init() succeed";
  return 0;
}

int FusionCameraDetectionComponent::InitCameraFrames() {
  if (camera_names_.size() != 2) {
    AERROR << "invalid camera_names_.size(): " << camera_names_.size();
    return -1;
  }
  // fixed size
  camera_frames_.resize(frame_capacity_);
  if (camera_frames_.empty()) {
    AERROR << "frame_capacity_ must > 0";
    return -1;
  }

  // init data_providers for each camera
  for (const auto &camera_name : camera_names_) {
    camera::DataProvider::InitOptions data_provider_init_options;
    data_provider_init_options.image_height = image_height_;
    data_provider_init_options.image_width = image_width_;
    data_provider_init_options.do_undistortion = enable_undistortion_;
    data_provider_init_options.sensor_name = camera_name;
    data_provider_init_options.device_id = gpu_id_;
    AINFO << "data_provider_init_options.device_id: "
          << data_provider_init_options.device_id
          << " camera_name: " << camera_name;

    std::shared_ptr<camera::DataProvider> data_provider(
        new camera::DataProvider);
    data_provider->Init(data_provider_init_options);
    data_providers_map_[camera_name] = data_provider;
  }

  //  init extrinsic/intrinsic
  for (const auto &camera_name : camera_names_) {
    base::BaseCameraModelPtr model;
    model =
        common::SensorManager::Instance()->GetUndistortCameraModel(camera_name);
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    Eigen::Matrix3f intrinsic = pinhole->get_intrinsic_params();
    intrinsic_map_[camera_name] = intrinsic;
    AINFO << "#intrinsics of " << camera_name << ": "
          << intrinsic_map_[camera_name];
    Eigen::Matrix4d extrinsic;
    LoadExtrinsics(config_path_ + "/perception/camera/params/" + camera_name +
                       "_extrinsics.yaml",
                   &extrinsic);
    extrinsic_map_[camera_name] = extrinsic;
    AINFO << "#extrinsics of " << camera_name << ": "
          << extrinsic_map_[camera_name];
  }

  // Init camera height
  for (const auto &camera_name : camera_names_) {
    float height = default_camera_height_;
    SetCameraHeight(camera_name, config_path_ + "/perception/camera/params/",
                    default_camera_height_, &height);
    camera_height_map_[camera_name] = height;
  }

  for (auto &frame : camera_frames_) {
    frame.track_feature_blob.reset(new base::Blob<float>());
    frame.lane_detected_blob.reset(new base::Blob<float>());
  }

  return 0;
}

int FusionCameraDetectionComponent::InitProjectMatrix() {
  if (!GetProjectMatrix(camera_names_, extrinsic_map_, intrinsic_map_,
                        &project_matrix_, &pitch_diff_)) {
    AERROR << "GetProjectMatrix failed";
    return -1;
  }
  AINFO << "project_matrix_: " << project_matrix_;
  AINFO << "pitch_diff_:" << pitch_diff_;
  name_camera_pitch_angle_diff_map_[camera_names_[0]] = 0.f;
  name_camera_pitch_angle_diff_map_[camera_names_[1]] =
      static_cast<float>(pitch_diff_);

  return 0;
}

int FusionCameraDetectionComponent::InitMotionService() {
  const std::string &channel_name_local = "/apollo/perception/motion_service";
  // std::function<void(const MotionServiceMsgType &)> motion_service_callback =
  //     std::bind(&FusionCameraDetectionComponent::OnMotionService, this,
  //               std::placeholders::_1);
  // auto motion_service_reader =
  //     node_->CreateReader(channel_name_local, motion_service_callback);
  // initialize motion buffer
  if (motion_buffer_ == nullptr) {
    motion_buffer_ = std::make_shared<base::MotionBuffer>(motion_buffer_size_);
  } else {
    motion_buffer_->set_capacity(motion_buffer_size_);
  }
  return 0;
}

void FusionCameraDetectionComponent::SetCameraHeightAndPitch() {
  camera_obstacle_pipeline_->SetCameraHeightAndPitch(
      camera_height_map_, name_camera_pitch_angle_diff_map_,
      default_camera_pitch_);
}

void FusionCameraDetectionComponent::OnReceiveImage(
    const sensor_msgs::ImageConstPtr &message, const std::string &camera_name) {
  AINFO << "====================";
  AINFO << "Received msg from camera " << camera_name;
  const int frame_size = static_cast<int>(camera_frames_.size());
  camera::CameraFrame &camera_frame = camera_frames_[frame_id_ % frame_size];
  camera_frame.data_provider = data_providers_map_[camera_name].get();
  camera_frame.data_provider->FillImageData(
      message->height, message->width,
      reinterpret_cast<const uint8_t *>(message->data.data()),
      message->encoding);
  camera_frame.frame_id = frame_id_;
  camera_frame.timestamp = ros::Time::now().toSec();
  camera_frame.project_matrix.setIdentity();
  camera_obstacle_pipeline_->GetCalibrationService(
      &camera_frame.calibration_service);
  camera_obstacle_pipeline_->Perception(camera_perception_options_,
                                        &camera_frame);

  frame_id_++;
  std::cout << "camera_frame.tracked_objects.size(): "
            << camera_frame.tracked_objects.size() << std::endl;

  if (enable_visualization_ || write_visual_img_) {
    camera::DataProvider::ImageOptions image_options;
    image_options.target_color = base::Color::BGR;
    // std::shared_ptr<base::Blob<uint8_t>> image_blob(new base::Blob<uint8_t>);
    // camera_frame.data_provider->GetImageBlob(image_options,
    // image_blob.get()); std::shared_ptr<CameraPerceptionVizMessage> viz_msg(
    //     new (std::nothrow) CameraPerceptionVizMessage(
    //         camera_name, msg_timestamp, camera2world_trans.matrix(),
    //         image_blob, camera_frame.tracked_objects,
    //         camera_frame.lane_objects, *error_code));
    // bool send_viz_ret = camera_viz_writer_->Write(viz_msg);
    // AINFO << "send out camera visualization msg, ts: " << msg_timestamp
    //       << " send_viz_ret: " << send_viz_ret;
    Eigen::Affine3d world2camera;
    world2camera.setIdentity();
    cv::Mat output_image(image_height_, image_width_, CV_8UC3,
                         cv::Scalar(0, 0, 0));
    base::Image8U out_image(image_height_, image_width_, base::Color::RGB);
    camera_frame.data_provider->GetImage(image_options, &out_image);
    memcpy(output_image.data, out_image.cpu_data(),
           out_image.total() * sizeof(uint8_t));
    // visualize_.ShowResult_all_info_single_camera(output_image, camera_frame,
    //                                              motion_buffer_,
    //                                              world2camera);
    visualize_.DrawBoxes(output_image, camera_frame);
    // to rosmsg
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = ros::Time::now();
    cv_ptr->header.frame_id = "base_link";
    cv_ptr->image = output_image;
    pub_camera_obs_image_.publish(cv_ptr->toImageMsg());
  }
}

// @description: load camera extrinsics from yaml file
bool FusionCameraDetectionComponent::LoadExtrinsics(
    const std::string &yaml_file, Eigen::Matrix4d *camera_extrinsic) {
  if (!cyber::common::PathExists(yaml_file)) {
    AINFO << yaml_file << " does not exist!";
    return false;
  }
  YAML::Node node = YAML::LoadFile(yaml_file);
  double qw = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  try {
    if (node.IsNull()) {
      AINFO << "Load " << yaml_file << " failed! please check!";
      return false;
    }
    qw = node["transform"]["rotation"]["w"].as<double>();
    qx = node["transform"]["rotation"]["x"].as<double>();
    qy = node["transform"]["rotation"]["y"].as<double>();
    qz = node["transform"]["rotation"]["z"].as<double>();
    tx = node["transform"]["translation"]["x"].as<double>();
    ty = node["transform"]["translation"]["y"].as<double>();
    tz = node["transform"]["translation"]["z"].as<double>();
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<double> &bc) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file " << yaml_file
           << " with error, YAML exception:" << e.what();
    return false;
  }
  camera_extrinsic->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = qx;
  q.y() = qy;
  q.z() = qz;
  q.w() = qw;
  (*camera_extrinsic).block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*camera_extrinsic)(0, 3) = tx;
  (*camera_extrinsic)(1, 3) = ty;
  (*camera_extrinsic)(2, 3) = tz;
  (*camera_extrinsic)(3, 3) = 1;
  return true;
}

bool FusionCameraDetectionComponent::SetCameraHeight(
    const std::string &sensor_name, const std::string &params_dir,
    float default_camera_height, float *camera_height) {
  float base_h = default_camera_height;
  float camera_offset = 0.0f;

  // apollo::perception::onboard::FusionCameraDetectionConfig
  //     fusion_camera_detection_param;
  // if (!GetProtoConfig(&fusion_camera_detection_param)) {
  //   AINFO << "load fusion camera detection component proto param failed";
  //   return false;
  // }
  // std::string lidar_type_name_str =
  //     fusion_camera_detection_param.lidar_type_name();
  try {
    // YAML::Node lidar_height =
    // YAML::LoadFile(params_dir + "/" + lidar_type_name_str + "_height.yaml");
    // base_h = lidar_height["vehicle"]["parameters"]["height"].as<float>();
    YAML::Node camera_ex =
        YAML::LoadFile(params_dir + sensor_name + "_extrinsics.yaml");
    camera_offset = camera_ex["transform"]["translation"]["z"].as<float>();
    AINFO << "camera_offset: " << camera_offset;
    *camera_height = camera_offset;
  } catch (YAML::InvalidNode &in) {
    AERROR << "load camera extrisic file error, YAML::InvalidNode exception";
    return false;
  } catch (YAML::TypedBadConversion<float> &bc) {
    AERROR << "load camera extrisic file error, "
           << "YAML::TypedBadConversion exception";
    return false;
  } catch (YAML::Exception &e) {
    AERROR << "load camera extrisic file "
           << " error, YAML exception:" << e.what();
    return false;
  }
  return true;
}

// @description: get project matrix
bool FusionCameraDetectionComponent::GetProjectMatrix(
    const std::vector<std::string> &camera_names,
    const std::map<std::string, Eigen::Matrix4d> &extrinsic_map,
    const std::map<std::string, Eigen::Matrix3f> &intrinsic_map,
    Eigen::Matrix3d *project_matrix, double *pitch_diff = nullptr) {
  // TODO(techoe): This condition should be removed.
  if (camera_names.size() != 2) {
    AINFO << "camera number must be 2!";
    return false;
  }
  *project_matrix =
      intrinsic_map.at(camera_names[0]).cast<double>() *
      extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
      extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0) *
      intrinsic_map.at(camera_names[1]).cast<double>().inverse();
  // extract the pitch_diff = pitch_narrow - pitch_obstacle
  if (pitch_diff != nullptr) {
    Eigen::Vector3d euler =
        (extrinsic_map.at(camera_names[0]).block<3, 3>(0, 0).inverse() *
         extrinsic_map.at(camera_names[1]).block<3, 3>(0, 0))
            .eulerAngles(0, 1, 2);
    *pitch_diff = euler(0);
    AINFO << "pitch diff: " << *pitch_diff;
  }
  return true;
}

} // namespace onboard
} // namespace perception
} // namespace apollo