#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>

#include "absl/strings/str_split.h"
#include "modules/perception/onboard/component/detection_component.h"

using apollo::perception::onboard::DetectionComponent;
using apollo::perception::onboard::LidarFrameMessage;
using apollo::perception::base::ObjectType;

bool ReadPointcloudFromFile(
    std::string file_name,
    std::shared_ptr<apollo::drivers::PointCloud> &pb_msg) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) { return false; }

  int count = 0;
  float item[4];
  while (file.read((char *)&item[count++], 4)) {
    if (count == 4) {
      auto point = pb_msg->add_point();
      point->set_x(item[0]);
      point->set_y(item[1]);
      point->set_z(item[2]);
      point->set_intensity(static_cast<uint32_t>(item[3] * 255));
      count = 0;
    }
  }
  return true;
}

enum ApolloScapeObjectType {
  SMALL_VEHICLE = 1, 
  BIG_VEHICLE = 2, 
  PEDESTRIAN = 3, 
  CYCLIST = 4, 
  TRAFFIC_CONE = 5, 
  OTHERS = 6, 
};

std::map<ObjectType, ApolloScapeObjectType> to_apolloscape_type {
  {ObjectType::UNKNOWN, ApolloScapeObjectType::OTHERS}, 
  {ObjectType::UNKNOWN_MOVABLE, ApolloScapeObjectType::OTHERS}, 
  {ObjectType::UNKNOWN_UNMOVABLE, ApolloScapeObjectType::OTHERS}, 
  {ObjectType::PEDESTRIAN, ApolloScapeObjectType::PEDESTRIAN}, 
  {ObjectType::BICYCLE, ApolloScapeObjectType::CYCLIST}, 
  {ObjectType::VEHICLE, ApolloScapeObjectType::SMALL_VEHICLE}, 
};

void OutputResultToFile(
    std::string result_file,
    std::shared_ptr<LidarFrameMessage> &lidar_frame_message) {
  std::ofstream ofs;
  ofs.open(result_file, 
           std::fstream::in | std::fstream::out | std::fstream::trunc);
  if (!ofs.is_open()) {
    AERROR << "Failed to write to " << result_file;
    return;
  }
  for (const auto &object : 
       lidar_frame_message->lidar_frame_->segmented_objects) {
    int apolloscape_type = static_cast<int>(to_apolloscape_type[object->type]);
    if (apolloscape_type > 4) { continue; }
    // if (object->confidence < 0.5) { continue; }
    ofs << apolloscape_type + (int)object->sub_type << " "
        << object->center[0] << " "
        << object->center[1] << " "
        << object->center[2] + object->size[2] / 2. << " "
        << object->size[0] << " "
        << object->size[1] << " "
        << object->size[2] << " "
        << object->theta << " "
        << object->confidence << "\n";
  }
  ofs.close();
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    AINFO << "\nUsage:\n\trosrun perception lidar_detection_evaluate [train/test]";
    return -1;
  }
  std::string data_type = argv[1];

  ros::init(argc, argv, "lidar_detection_evaluate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  DetectionComponent detection;
  detection.Init(nh, private_nh);

  std::string dataset_dir = "/media/datasets/apolloscape";
  std::string sample_list_file = dataset_dir + "/" + data_type + ".txt";
  std::ifstream ifs(sample_list_file, std::ios::in);
  if (!ifs.is_open()) {
    AERROR << "Open file failed: " << sample_list_file;
    return -1;
  }
  std::string sample;
  std::ifstream ifs_pc;
  while (ifs >> sample) {
    auto pb_msg = std::make_shared<apollo::drivers::PointCloud>();
    std::string sample_file = dataset_dir + "/" + sample;
    ReadPointcloudFromFile(sample_file, pb_msg);
    pb_msg->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    pb_msg->mutable_header()->set_frame_id("velodyne64");
    pb_msg->mutable_header()->set_lidar_timestamp(
        ::google::protobuf::uint64(ros::Time::now().toNSec()));
    pb_msg->set_frame_id("velodyne64");
    pb_msg->set_measurement_time(ros::Time::now().toNSec());
    pb_msg->set_width(pb_msg->point_size());
    pb_msg->set_height(1);
    AINFO << sample_file << " pointcloud size: " << pb_msg->point_size();

    auto lidar_frame_message = std::make_shared<LidarFrameMessage>();
    bool status = detection.Proc(pb_msg, lidar_frame_message);
    if (!status) {
      AERROR << "Lidar detection failed!";
      break;
    }

    std::vector<std::string> v = absl::StrSplit(sample, '/');
    std::string scene_name = v[2].substr(7, v[2].size() - 13);
    std::string sample_name = v[3].substr(0, v[3].size() - 4);
    std::string result_dir = dataset_dir + "/" + data_type + 
                             "ing/detection_result";
    boost::filesystem::path dir_1(result_dir);
    if (!boost::filesystem::exists(dir_1)) {
      boost::filesystem::create_directory(dir_1);
    }
    result_dir += "/" + scene_name;
    boost::filesystem::path dir_2(result_dir);
    if (!boost::filesystem::exists(dir_2)) {
      boost::filesystem::create_directory(dir_2);
    }
    std::string result_file = result_dir + "/" + sample_name + ".txt";
    AINFO << "result_file: " << result_file;
    OutputResultToFile(result_file, lidar_frame_message);
    // getchar();
  }

  return 0;
}