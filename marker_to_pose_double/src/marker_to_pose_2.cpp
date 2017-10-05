#include "ros/ros.h"
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h" //

class Marker2Pose
{
public: 
  Marker2Pose();

private:
  ros::NodeHandle n;
  ros::Subscriber marker_sub;
  ros::Publisher pose_pub;
  ros::Subscriber info_sub; //
  ros::Publisher info_pub; //

  sensor_msgs::CameraInfo camera_info;
  geometry_msgs::PoseStamped current_pose;
  void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg);
  void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};

Marker2Pose::Marker2Pose()
{
  marker_sub = n.subscribe<aruco_msgs::MarkerArray>("/aruco_marker_publisher_2/markers", 10, &Marker2Pose::markerCallback, this);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/aruco_marker_publisher_2/PoseStamped", 10);

  info_sub = n.subscribe<sensor_msgs::CameraInfo>("/camera2/color/camera_info",10, &Marker2Pose::infoCallback, this); //
  info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera2/color/calibrated_camera_info",10); //
}  

void Marker2Pose::markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg)
{
  current_pose.header.seq = msg->markers[0].header.seq;
  current_pose.header.stamp = msg->markers[0].header.stamp;
  current_pose.header.frame_id = msg->markers[0].header.frame_id;

  current_pose.pose.position.x = msg->markers[0].pose.pose.position.x;
  current_pose.pose.position.y = msg->markers[0].pose.pose.position.y;
  current_pose.pose.position.z = msg->markers[0].pose.pose.position.z;

  current_pose.pose.orientation.x = msg->markers[0].pose.pose.orientation.x;
  current_pose.pose.orientation.y = msg->markers[0].pose.pose.orientation.y;
  current_pose.pose.orientation.z = msg->markers[0].pose.pose.orientation.z;
  current_pose.pose.orientation.w = msg->markers[0].pose.pose.orientation.w;

  pose_pub.publish(current_pose);
}

void Marker2Pose::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_info.header = msg->header;
  camera_info.height = msg->height;
  camera_info.width = msg->width;
  camera_info.distortion_model = msg->distortion_model;
  camera_info.D = msg->D;
  camera_info.K = msg->K;
  camera_info.R = msg->R;
  camera_info.P = msg->P;
  camera_info.binning_x = msg->binning_x;
  camera_info.binning_y = msg->binning_y;
  camera_info.roi = msg->roi;
  camera_info.K[0] = 625.4995;
  camera_info.K[2] = 340.1406;
  camera_info.K[4] = 627.5500;
  camera_info.K[5] = 233.7812;
  camera_info.P[0] = 625.4995;
  camera_info.P[2] = 340.1406;
  camera_info.P[5] = 627.5500;
  camera_info.P[6] = 233.7812;
  info_pub.publish(camera_info);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "readmarker");
  Marker2Pose marker2pose;
  ros::spin();
  
  return 0;
}
