#include "drivers/ContiRadar.h"
#include "geometry_msgs/Quaternion.h"
#include "marker_color.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/MarkerArray.h"
#include <type_traits>
#include <unordered_map>

using namespace apollo::tools;

std::unordered_map<std::string, Color> radar_color{
    {"radar_front_left", Color::YELLOW},
    {"radar_front_right", Color::MAGENTA},
    {"radar_rear", Color::SKY_BLUE}};

inline void AddMarker(visualization_msgs::MarkerArray &output,
                      const drivers::ContiRadar &msg, std::string ns) {
  if (msg.contiobs.empty()) {
    return;
  }

  size_t orig_size = output.markers.size();
  output.markers.resize(orig_size + msg.contiobs.size());
  for (int i = orig_size; i < orig_size + msg.contiobs.size(); ++i) {
    output.markers[i].header.frame_id = ns;
    output.markers[i].header.stamp = ros::Time::now();
    output.markers[i].lifetime = ros::Duration(0.1);
    output.markers[i].ns = ns + "/object";
    output.markers[i].id = msg.contiobs[i - orig_size].obstacle_id;
    output.markers[i].action = visualization_msgs::Marker::ADD;
    output.markers[i].type = visualization_msgs::Marker::CUBE;
    // size
    // output.markers[i].scale.x = msg.contiobs[i - orig_size].length;
    // output.markers[i].scale.y = msg.contiobs[i - orig_size].width;
    output.markers[i].scale.x = 0.5;
    output.markers[i].scale.y = 0.5;
    output.markers[i].scale.z = 1.0;
    // color
    SetRGBA(output.markers[i], radar_color[ns], 0.7);
    // position
    output.markers[i].pose.position.x =
        msg.contiobs[i - orig_size].longitude_dist;
    output.markers[i].pose.position.y =
        msg.contiobs[i - orig_size].lateral_dist;
    output.markers[i].pose.position.z = -1.;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(
        -msg.contiobs[i - orig_size].oritation_angle);
    output.markers[i].pose.orientation = quat;
    // tf::Quaternion quat = tf::Quaternion(0, 0,
    // -msg.contiobs[i-orig_size].heading, 1); tf::quaternionTFToMsg(quat,
    // output.markers[i].pose.orientation);
  }
}

inline void AddText(visualization_msgs::MarkerArray &output,
                    const drivers::ContiRadar &msg, std::string ns) {
  if (msg.contiobs.empty()) {
    return;
  }

  size_t orig_size = output.markers.size();
  output.markers.resize(orig_size + msg.contiobs.size());
  for (int i = orig_size; i < orig_size + msg.contiobs.size(); ++i) {
    output.markers[i].header.frame_id = ns;
    output.markers[i].header.stamp = ros::Time::now();
    output.markers[i].lifetime = ros::Duration(0.1);
    output.markers[i].ns = ns + "/text";
    output.markers[i].id = msg.contiobs[i - orig_size].obstacle_id;
    output.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    output.markers[i].action = visualization_msgs::Marker::ADD;
    // size
    output.markers[i].scale.x = 0.8;
    output.markers[i].scale.y = 0.8;
    output.markers[i].scale.z = 0.8;
    // color
    SetRGBA(output.markers[i], radar_color[ns], 0.7);
    // position
    output.markers[i].pose.position.x =
        msg.contiobs[i - orig_size].longitude_dist;
    output.markers[i].pose.position.y =
        msg.contiobs[i - orig_size].lateral_dist;
    output.markers[i].pose.position.z = 0;

    std::stringstream text;
    text << "id: " << msg.contiobs[i - orig_size].obstacle_id << std::endl;
    // text << "x|y: " << std::setprecision(4)
    //      << msg.contiobs[i - orig_size].longitude_dist << "|"
    //      << msg.contiobs[i - orig_size].lateral_dist << std::endl;
    text << "vx|vy: " << std::setprecision(4)
         << msg.contiobs[i - orig_size].longitude_vel << "|"
         << msg.contiobs[i - orig_size].lateral_vel << std::endl;
    // text << "w|l: " << std::setprecision(4) << msg.contiobs[i -
    // orig_size].width
    //      << "|" << msg.contiobs[i - orig_size].length << std::endl;
    // text << "heading: " << std::fixed << std::setprecision(0) <<
    // msg.contiobs[i-orig_size].heading * 180. / M_PI << std::endl;
    output.markers[i].text = text.str();
  }
}

inline void AddOrientation(visualization_msgs::MarkerArray &output,
                           const drivers::ContiRadar &msg, std::string ns) {
  if (msg.contiobs.empty())
    return;

  size_t orig_size = output.markers.size();
  output.markers.resize(orig_size + msg.contiobs.size());
  for (int i = orig_size; i < orig_size + msg.contiobs.size(); ++i) {
    output.markers[i].header.frame_id = ns;
    output.markers[i].header.stamp = ros::Time::now();
    output.markers[i].lifetime = ros::Duration(0.1);
    output.markers[i].ns = ns + "/orientation";
    output.markers[i].id = msg.contiobs[i - orig_size].obstacle_id;
    output.markers[i].action = visualization_msgs::Marker::ADD;
    output.markers[i].type = visualization_msgs::Marker::ARROW;
    // size
    output.markers[i].scale.x = 2.0;
    output.markers[i].scale.y = 0.5;
    output.markers[i].scale.z = 1.0;
    // color
    SetRGBA(output.markers[i], radar_color[ns], 0.7);
    // position
    output.markers[i].pose.position.x =
        msg.contiobs[i - orig_size].longitude_dist;
    output.markers[i].pose.position.y =
        msg.contiobs[i - orig_size].lateral_dist;
    output.markers[i].pose.position.z = 2.0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(
        -msg.contiobs[i - orig_size].oritation_angle);
    output.markers[i].pose.orientation = quat;
  }
}

visualization_msgs::MarkerArray radar_front_left_markers_;
void CallbackRadarFrontLeft(const drivers::ContiRadar::ConstPtr &msg) {
  std::string sensor_name = "radar_front_left";
  std::cout << sensor_name << " contiobs size: " << msg->contiobs.size()
            << std::endl;
  radar_front_left_markers_.markers.clear();
  AddMarker(radar_front_left_markers_, *msg, sensor_name);
  AddText(radar_front_left_markers_, *msg, sensor_name);
}

visualization_msgs::MarkerArray radar_front_right_markers_;
void CallbackRadarFrontRight(const drivers::ContiRadar::ConstPtr &msg) {
  std::string sensor_name = "radar_front_right";
  std::cout << sensor_name << " contiobs size: " << msg->contiobs.size()
            << std::endl;
  radar_front_right_markers_.markers.clear();
  AddMarker(radar_front_right_markers_, *msg, sensor_name);
  AddText(radar_front_right_markers_, *msg, sensor_name);
}

visualization_msgs::MarkerArray radar_rear_markers_;
void CallbackRadarRear(const drivers::ContiRadar::ConstPtr &msg) {
  std::string sensor_name = "radar_rear";
  std::cout << sensor_name << " contiobs size: " << msg->contiobs.size()
            << std::endl;
  radar_rear_markers_.markers.clear();
  AddMarker(radar_rear_markers_, *msg, sensor_name);
  AddText(radar_rear_markers_, *msg, sensor_name);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_visualizer");
  ros::NodeHandle nh;
  // subscriber
  ros::Subscriber sub_radar_front_left = nh.subscribe(
      "/apollo/sensor/radar/front_left", 1, CallbackRadarFrontLeft);
  ros::Subscriber sub_radar_front_right = nh.subscribe(
      "/apollo/sensor/radar/front_right", 1, CallbackRadarFrontRight);
  ros::Subscriber sub_radar_rear =
      nh.subscribe("/apollo/sensor/radar/rear", 1, CallbackRadarRear);

  // publisher
  ros::Publisher pub_radar_fl = nh.advertise<visualization_msgs::MarkerArray>(
      "/apollo/sensor/radar/front_left/markers", 1, true);
  ros::Publisher pub_radar_fr = nh.advertise<visualization_msgs::MarkerArray>(
      "/apollo/sensor/radar/front_right/markers", 1, true);
  ros::Publisher pub_radar_r = nh.advertise<visualization_msgs::MarkerArray>(
      "/apollo/sensor/radar/rear/markers", 1, true);

  ros::Rate rate(13);
  while (ros::ok()) {
    pub_radar_fl.publish(radar_front_left_markers_);
    pub_radar_fr.publish(radar_front_right_markers_);
    pub_radar_r.publish(radar_rear_markers_);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
