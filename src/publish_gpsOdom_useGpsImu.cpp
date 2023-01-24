#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "utility.h"
#include "Geocentric/LocalCartesian.hpp"
#include "GeographicLib/LocalCartesian.hpp"


static GeographicLib::LocalCartesian geo_converter;

std::deque<sensor_msgs::NavSatFix> gpsQueue;
std::deque<sensor_msgs::Imu> imuQueue;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  gpsQueue.push_back(*msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  imuQueue.push_back(*msg);
}

int main(int argc, char *argv[]) {
  // google::InitGoogleLogging(argv[0]);
  // FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  // FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "publish_gpsOdom_useGpsImu");
  ros::NodeHandle nh;

  ros::Subscriber gps_sub = nh.subscribe("/fix", 10, gps_callback);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 10, imu_callback);
  ros::Publisher  pubGpsOdometry = nh.advertise<nav_msgs::Odometry>("gps_odom_incremental", 20);

  /**  移动智地高精车出发原点 gps , -21.2, 7.4 can init success **/
  double longitude_orin = 121.1702651870648282;
  double latitude_orin = 31.1506836219693000;
  double altitude_orin = 4.3672881126403809;
  geo_converter.Reset(latitude_orin, longitude_orin, altitude_orin);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    while (imuQueue.size() > 0 && gpsQueue.size() > 0) {
  
        sensor_msgs::Imu imu_data = imuQueue.front();
        sensor_msgs::NavSatFix gps_data = gpsQueue.front();

        double d_time = gps_data.header.stamp.toSec() - imu_data.header.stamp.toSec();
        if (d_time < -0.05) {
            // std::cout << " gps data is too front " << std::endl;
            gpsQueue.pop_front();
        } else if (d_time > 0.05) {
            // std::cout << " imu data is too front " << std::endl;
            imuQueue.pop_front();
        } else {
            gpsQueue.pop_front();
            imuQueue.pop_front();

            nav_msgs::Odometry odometry;
            odometry.header.stamp = gps_data.header.stamp;
            odometry.header.frame_id = "odom";
            odometry.child_frame_id = "navsat_link";

            double local_E = 0.0;
            double local_N = 0.0;
            double local_U = 0.0;
            geo_converter.Forward(gps_data.latitude, gps_data.longitude, gps_data.altitude, 
                  local_E, local_N,local_U);

            odometry.pose.pose.position.x = local_E;
            odometry.pose.pose.position.y = local_N;
            odometry.pose.pose.position.z = local_U;
            odometry.pose.pose.orientation.x = imu_data.orientation.x;
            odometry.pose.pose.orientation.y = imu_data.orientation.y;
            odometry.pose.pose.orientation.z = imu_data.orientation.z;
            odometry.pose.pose.orientation.w = imu_data.orientation.w;
            // odometry.twist = odomMsg->twist;

            pubGpsOdometry.publish(odometry);
        }
      }
    

    rate.sleep();
  }

  return 0;
}