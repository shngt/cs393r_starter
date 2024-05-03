//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam-main.cc
\brief   Main entry point for slam
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"

#include "slam.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

using amrl_msgs::VisualizationMsg;
using geometry::line2f;
using geometry::Line;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");

DECLARE_int32(v);

bool run_ = true;
slam::SLAM slam_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
VisualizationMsg vis_msg_;
VisualizationMsg local_vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
  local_vis_msg_ = visualization::NewVisualizationMessage("base_link", "slam");
}

void PublishMap() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.5) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  const vector<Vector2f> map = slam_.GetMap();
  printf("Map: %lu points\n", map.size());
  for (const Vector2f& p : map) {
    visualization::DrawPoint(p, 0xC0C0C0, vis_msg_);
  }
  visualization_publisher_.publish(vis_msg_);
}

void PublishPose() {
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  slam_.GetPose(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  // static int color = 0x000000;
  // if (slam_.apply_new_scan_) {
  //   printf("Laser t=%f\n", msg.header.stamp.toSec());
  //   vector<Vector2f> point_cloud;
  //   const float angle_increment = (msg.angle_max - msg.angle_min) / msg.ranges.size();
  //   // save the pointcloud in the robot's frame
  //   // std::fill(point_cloud.begin(), point_cloud.end(), Vector2f(0, 0));
  //   for (size_t i = 0; i < msg.ranges.size(); i += 10) {
  //       if (msg.ranges[i] >= msg.range_max) {
  //         continue;
  //       }
  //       const float angle = msg.angle_min + i * angle_increment;
  //       Vector2f point(msg.ranges[i] * cos(angle) + 0.2, msg.ranges[i] * sin(angle));
  //       point_cloud.push_back(point);
  //   }
  //   printf("color: %d\n", color);
  //   for (const Vector2f& p : point_cloud) {
  //     visualization::DrawPoint(p, color, local_vis_msg_);
  //   }
  //   color = color + 0x0000FF;
  // }
  // visualization_publisher_.publish(local_vis_msg_);
  last_laser_msg_ = msg;
  slam_.ObserveLaser(msg);
  // printf("color in hex: %x\n", color);
  // if (color == 0x0001FE) {
  //   for (const Vector2f& p : slam_.transformed_point_cloud_) {
  //     visualization::DrawPoint(p, 0xFF0000, local_vis_msg_);
  //   }
  //   visualization_publisher_.publish(local_vis_msg_);
  //   // color = 0x000000;
  // }
  PublishMap();
  PublishPose();
  // if (slam_.pose_index_ == 3) exit(0);
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  slam_.ObserveOdometry(odom_loc, odom_angle);
}


int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Initialize ROS.
  ros::init(argc, argv, "slam");
  ros::NodeHandle n;
  InitializeMsgs();

  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);

  ros::Subscriber laser_sub = n.subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  ros::Subscriber odom_sub = n.subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  ros::spin();

  return 0;
}
