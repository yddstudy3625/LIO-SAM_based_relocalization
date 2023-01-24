#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "utility.h"

using namespace std;

ros::Publisher gps_pub;
ros::Publisher gps_odom_pub;
ros::Publisher pubGpsOdometry;
nav_msgs::Odometry::ConstPtr gps_first_odom_ = nullptr;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  sensor_msgs::NavSatFix gps_data;
  gps_data.header.stamp = msg->header.stamp;
  gps_data.header.frame_id = "navsat_link";
  gps_data.status = msg->status;
  gps_data.latitude = msg->latitude;
  gps_data.longitude = msg->longitude;
  gps_data.altitude = msg->altitude;
  gps_data.position_covariance = msg->position_covariance;
  gps_data.position_covariance_type = msg->position_covariance_type;
  gps_pub.publish(gps_data);
}

void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  nav_msgs::Odometry odom_data;
  // gps_data.header.stamp = msg->header.stamp;
  double time = ros::Time::now().toSec();  // 把时间戳转化成浮点型格式
  odom_data.header.stamp = ros::Time().fromSec(time);  // 把浮点型变成时间戳
  odom_data.header.frame_id = "navsat_link";
  odom_data.pose = msg->pose;
  odom_data.twist = msg->twist;
  gps_odom_pub.publish(odom_data);
}

void GPSOdometryPubHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
  if (gps_first_odom_ == nullptr) {
    gps_first_odom_ = odomMsg;
  }

  nav_msgs::Odometry odometry;
  odometry.header.stamp = odomMsg->header.stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "navsat_link";

  odometry.pose.pose.position.x =
      odomMsg->pose.pose.position.x - 325583.81303664425;
  odometry.pose.pose.position.y =
      odomMsg->pose.pose.position.y - 3447742.608652003;
  odometry.pose.pose.position.z =
      odomMsg->pose.pose.position.z - 0.0;
  odometry.pose.pose.orientation.x = odomMsg->pose.pose.orientation.x;
  odometry.pose.pose.orientation.y = odomMsg->pose.pose.orientation.y;
  odometry.pose.pose.orientation.z = odomMsg->pose.pose.orientation.z;
  odometry.pose.pose.orientation.w = odomMsg->pose.pose.orientation.w;
  odometry.twist = odomMsg->twist;

  pubGpsOdometry.publish(odometry);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "republish_frameid_gps");
  cout << "republish_frameid_gps start" << endl;

  ros::NodeHandle nh_;
  gps_pub = nh_.advertise<sensor_msgs::NavSatFix>("/fix_modify", 10);
  ros::Subscriber gps_sub = nh_.subscribe("/fix", 10, gps_callback);
  gps_odom_pub = nh_.advertise<nav_msgs::Odometry>("/gps/odom_retime", 10);
  ros::Subscriber imu_odom_sub =
      nh_.subscribe("/oxts_ins/odom", 10, gps_odom_callback);
  ros::Subscriber subGPSOdometry = nh_.subscribe<nav_msgs::Odometry>(
      "gps/odom_retime", 5, GPSOdometryPubHandler,
      ros::TransportHints().tcpNoDelay());
  pubGpsOdometry =
      nh_.advertise<nav_msgs::Odometry>("gps_odom_incremental", 2000);
  ros::spin();
  return 0;
}
