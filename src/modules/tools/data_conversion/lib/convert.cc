#include "modules/tools/data_conversion/lib/convert.h"

void ConvertHeaderFromRosToPb(const std_msgs::Header *msg, 
                              apollo::common::Header *pb) {
  pb->set_timestamp_sec(msg->stamp.toSec());
  pb->set_frame_id(msg->frame_id);
  pb->set_sequence_num(msg->seq);
}

void ConvertHeaderFromPbToRos(apollo::common::Header *pb,
                              std_msgs::Header *msg) {
  ros::Time t(pb->timestamp_sec());
  msg->stamp = t;
  msg->seq = pb->sequence_num();
  msg->frame_id = pb->frame_id();
}

void ConvertPointCloudFromRosToPb(
    const sensor_msgs::PointCloud2::ConstPtr &msg,
    std::shared_ptr<apollo::drivers::PointCloud> &pb) {
  ConvertHeaderFromRosToPb(&msg->header, pb->mutable_header());
  pb->mutable_header()->set_lidar_timestamp(
      ::google::protobuf::uint64 (msg->header.stamp.toNSec()));
  pb->set_frame_id(msg->header.frame_id);
  pb->set_measurement_time(msg->header.stamp.toNSec());
  pb->set_width(msg->width);
  pb->set_height(msg->height);

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int stamp_offset = -1;
  int intensity_offset = -1;
  for (const auto &field : msg->fields) {
    if (field.name == "x") {
      x_offset = field.offset;
    } else if (field.name == "y") {
      y_offset = field.offset;
    } else if (field.name == "z") {
      z_offset = field.offset;
    } else if (field.name == "timestamp") {
      stamp_offset = field.offset;
    } else if (field.name == "intensity") {
      intensity_offset = field.offset;
    }
  }

  if (x_offset == -1 || y_offset == -1 || z_offset == -1 ||
      stamp_offset == -1 || intensity_offset == -1) {
    std::cerr << "Field not contains x, y, z, timestamp, instensity"
              << std::endl;
    return;
  }

  int total = msg->width * msg->height;
  auto data = msg->data;
  for (int i = 0; i < total; ++i) {
    auto cyber_point = pb->add_point();
    int offset = i * msg->point_step;
    cyber_point->set_x(*reinterpret_cast<float *>(&data[offset + x_offset]));
    cyber_point->set_y(*reinterpret_cast<float *>(&data[offset + y_offset]));
    cyber_point->set_z(*reinterpret_cast<float *>(&data[offset + z_offset]));
    cyber_point->set_intensity(
        *reinterpret_cast<uint8_t *>(&data[offset + intensity_offset]));
    cyber_point->set_timestamp(static_cast<std::uint64_t>(
        *reinterpret_cast<double *>(&data[offset + stamp_offset]) * 1e9));
  }
}

void ConvertPointCloudFromPbToRos(
    apollo::drivers::PointCloud *pb, sensor_msgs::PointCloud2 *msg) {
  sensor_msgs::PointCloud pointcloud;
  ConvertHeaderFromPbToRos(pb->mutable_header(), &pointcloud.header);
  geometry_msgs::Point32 point32;
  sensor_msgs::ChannelFloat32 channel_float32;
  channel_float32.name = "intensity";
  for (size_t i = 0; i < pb->point().size(); ++i) {
    point32.x = pb->point(i).x();
    point32.y = pb->point(i).y();
    point32.z = pb->point(i).z();
    channel_float32.values.push_back(pb->point(i).intensity());
    pointcloud.points.push_back(point32);
  }

  if (sensor_msgs::convertPointCloudToPointCloud2(pointcloud, *msg) != 1) {
    std::cerr << "Fail to convert sensor_msgs PointCloud to PointCloud2!"
              << std::endl;
    return;
  }
}

void ConvertImageFromRosToPb(const sensor_msgs::Image::ConstPtr &msg,
                             std::shared_ptr<apollo::drivers::Image> &pb) {
  ConvertHeaderFromRosToPb(&msg->header, pb->mutable_header());
  pb->set_frame_id(msg->header.frame_id);
  pb->set_measurement_time(msg->header.stamp.toSec());
  pb->set_width(msg->width);
  pb->set_height(msg->height);
  pb->set_encoding(msg->encoding);
  pb->set_step(msg->step);
  pb->mutable_data()->reserve(msg->step * msg->height);
  pb->set_data((const void *)&msg->data[0], msg->step * msg->height);
  if (msg->encoding != "yuyv" && msg->encoding != "rgb8") {
    std::cerr << "Wrong pixel fromat:" << msg->encoding
              << ",must be yuyv | rgb24" << std::endl;
    return;
  }
}

void ConvertImageFromPbToRos(apollo::drivers::Image *pb,
                             sensor_msgs::Image *msg) {
  ConvertHeaderFromPbToRos(pb->mutable_header(), &msg->header);
  msg->height = pb->height();
  msg->width = pb->width();
  msg->encoding = pb->encoding();
  msg->step = pb->step();
  memcpy(&msg->data[0], pb->mutable_data(), pb->step() * pb->height());
  if (msg->encoding != "yuyv" && msg->encoding != "rgb8") {
    std::cerr << "Wrong pixel fromat:" << msg->encoding
              << ",must be yuyv | rgb24" << std::endl;
    return;
  }
}

void ConvertCompressedImageFromRosToPb(
    const sensor_msgs::CompressedImage::ConstPtr &msg, 
    std::shared_ptr<apollo::drivers::CompressedImage> &pb) {
  ConvertHeaderFromRosToPb(&msg->header, pb->mutable_header());
  pb->set_frame_id(msg->header.frame_id);
  pb->set_measurement_time(msg->header.stamp.toSec());
  pb->set_format(msg->format);
  pb->set_data((const void *)&msg->data[0], msg->data.size());
}

void ConvertCompressedImageFromPbToRos(apollo::drivers::CompressedImage *pb,
                                       sensor_msgs::CompressedImage *msg) {
  ConvertHeaderFromPbToRos(pb->mutable_header(), &msg->header);
  msg->format = pb->format();
  msg->data.resize(pb->data().size());
  memcpy(&msg->data[0], pb->data().data(), pb->data().size());
}

void ConvertContiRadarFromPbToRos(apollo::drivers::ContiRadar *pb,
                                  drivers::ContiRadar *msg) {
  ConvertHeaderFromPbToRos(pb->mutable_header(), &msg->header);
  drivers::ContiRadarObs obs;
  for (int i = 0; i < pb->contiobs_size(); ++i) {
    auto contiobs = pb->mutable_contiobs(i);
    ConvertHeaderFromPbToRos(contiobs->mutable_header(), &obs.header);
    obs.clusterortrack = contiobs->clusterortrack();
    obs.obstacle_id = contiobs->obstacle_id();
    obs.longitude_dist = contiobs->longitude_dist();
    obs.lateral_dist = contiobs->lateral_dist();
    obs.longitude_vel = contiobs->longitude_vel();
    obs.lateral_vel = contiobs->lateral_vel();
    obs.rcs = contiobs->rcs();
    obs.dynprop = contiobs->dynprop();
    obs.longitude_dist_rms = contiobs->longitude_dist_rms();
    obs.lateral_dist_rms = contiobs->lateral_dist_rms();
    obs.longitude_vel_rms = contiobs->longitude_vel_rms();
    obs.lateral_vel_rms = contiobs->lateral_vel_rms();
    obs.probexist = contiobs->probexist();
    obs.meas_state = contiobs->meas_state();
    obs.longitude_accel = contiobs->longitude_accel();
    obs.lateral_accel = contiobs->lateral_accel();
    obs.oritation_angle = contiobs->oritation_angle();
    obs.longitude_accel_rms = contiobs->longitude_accel_rms();
    obs.lateral_accel_rms = contiobs->lateral_accel_rms();
    obs.oritation_angle_rms = contiobs->oritation_angle_rms();
    obs.length = contiobs->length();
    obs.width = contiobs->width();
    obs.obstacle_class = contiobs->obstacle_class();
    msg->contiobs.push_back(obs);
  }

  msg->cluster_list_status.near = pb->cluster_list_status().near();
  msg->cluster_list_status.far = pb->cluster_list_status().far();
  msg->cluster_list_status.meas_counter =
      pb->cluster_list_status().meas_counter();
  msg->cluster_list_status.interface_version =
      pb->cluster_list_status().interface_version();

  msg->object_list_status.nof_objects = pb->object_list_status().nof_objects();
  msg->object_list_status.meas_counter =
      pb->object_list_status().meas_counter();
  msg->object_list_status.interface_version =
      pb->object_list_status().interface_version();

  msg->radar_state.max_distance = pb->radar_state().max_distance();
  msg->radar_state.radar_power = pb->radar_state().radar_power();
  msg->radar_state.output_type = pb->radar_state().output_type();
  msg->radar_state.rcs_threshold = pb->radar_state().rcs_threshold();
  msg->radar_state.send_quality = pb->radar_state().send_quality();
  msg->radar_state.send_ext_info = pb->radar_state().send_ext_info();
}

void ConvertTransformFromPbToRos(apollo::transform::Transform *pb,
                                 geometry_msgs::Transform *msg) {
  msg->translation.x = pb->translation().x();
  msg->translation.y = pb->translation().y();
  msg->translation.z = pb->translation().z();
  msg->rotation.x = pb->rotation().qx();
  msg->rotation.y = pb->rotation().qy();
  msg->rotation.z = pb->rotation().qz();
  msg->rotation.w = pb->rotation().qw();
}

void ConvertTransformStampedsFromRosToPb(
    const tf2_msgs::TFMessageConstPtr &msg,
    std::shared_ptr<apollo::transform::TransformStampeds> &pb) {
  apollo::transform::TransformStamped *cyber_tf;
  for (size_t i = 0; i < msg->transforms.size(); ++i) {
    cyber_tf = pb->add_transforms();
    ConvertHeaderFromRosToPb(&msg->transforms[i].header, 
                             cyber_tf->mutable_header());
    cyber_tf->set_child_frame_id(msg->transforms[i].child_frame_id);
    cyber_tf->mutable_transform()->mutable_translation()->set_x(
        msg->transforms[i].transform.translation.x);
    cyber_tf->mutable_transform()->mutable_translation()->set_y(
        msg->transforms[i].transform.translation.y);
    cyber_tf->mutable_transform()->mutable_translation()->set_z(
        msg->transforms[i].transform.translation.z);
    cyber_tf->mutable_transform()->mutable_rotation()->set_qx(
        msg->transforms[i].transform.rotation.x);
    cyber_tf->mutable_transform()->mutable_rotation()->set_qy(
        msg->transforms[i].transform.rotation.y);
    cyber_tf->mutable_transform()->mutable_rotation()->set_qz(
        msg->transforms[i].transform.rotation.z);
    cyber_tf->mutable_transform()->mutable_rotation()->set_qw(
        msg->transforms[i].transform.rotation.w);
  }
}

void ConvertTransformStampedsFromPbToRos(
    apollo::transform::TransformStampeds *pb, tf2_msgs::TFMessage *msg) {
  geometry_msgs::TransformStamped ros_tf;
  for (int i = 0; i < pb->transforms_size(); ++i) {
    auto cyber_tf = pb->mutable_transforms(i);
    ConvertHeaderFromPbToRos(cyber_tf->mutable_header(), &ros_tf.header);
    ros_tf.child_frame_id = cyber_tf->child_frame_id();
    ConvertTransformFromPbToRos(cyber_tf->mutable_transform(),
                                &ros_tf.transform);
    msg->transforms.push_back(ros_tf);
  }
}

template <typename PointType>
void ConvertPointFromPbToRos(PointType *pb, geometry_msgs::Point *msg) {
  msg->x = pb->x();
  msg->y = pb->y();
  msg->z = pb->z();
}

void ConvertQuaternionFromPbToRos(apollo::common::Quaternion *pb,
                                  geometry_msgs::Quaternion *msg) {
  msg->x = pb->qx();
  msg->y = pb->qy();
  msg->z = pb->qz();
  msg->w = pb->qw();
}

void ConvertLocalizationEstimateFromPbToRos(
    apollo::localization::LocalizationEstimate *pb,
    localization::LocalizationEstimate *msg) {
  ConvertHeaderFromPbToRos(pb->mutable_header(), &msg->header);

  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_position(),
                          &msg->pose.position);
  ConvertQuaternionFromPbToRos(pb->mutable_pose()->mutable_orientation(),
                               &msg->pose.orientation);
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_linear_velocity(),
                          &msg->pose.linear_velocity);
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_linear_acceleration(),
                          &msg->pose.linear_acceleration);
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_angular_velocity(),
                          &msg->pose.angular_velocity);
  msg->pose.heading = pb->mutable_pose()->heading();
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_linear_acceleration_vrf(),
                          &msg->pose.linear_acceleration_vrf);
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_angular_velocity_vrf(),
                          &msg->pose.angular_velocity_vrf);
  ConvertPointFromPbToRos(pb->mutable_pose()->mutable_euler_angles(),
                          &msg->pose.euler_angles);

  ConvertPointFromPbToRos(pb->mutable_uncertainty()->mutable_position_std_dev(),
                          &msg->uncertainty.position_std_dev);
  ConvertPointFromPbToRos(
      pb->mutable_uncertainty()->mutable_orientation_std_dev(),
      &msg->uncertainty.orientation_std_dev);
  ConvertPointFromPbToRos(
      pb->mutable_uncertainty()->mutable_linear_velocity_std_dev(),
      &msg->uncertainty.linear_velocity_std_dev);
  ConvertPointFromPbToRos(
      pb->mutable_uncertainty()->mutable_linear_acceleration_std_dev(),
      &msg->uncertainty.linear_acceleration_std_dev);
  ConvertPointFromPbToRos(
      pb->mutable_uncertainty()->mutable_angular_velocity_std_dev(),
      &msg->uncertainty.angular_velocity_std_dev);
}
