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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "path_options.h"
#include "latency_compensation.h"
#include "global_planner.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    nav_path_(vector<Vector2f>({Vector2f(0, 0), Vector2f(0, 0)})),
    latency_compensation_(new LatencyCompensation(0, 0, 0))
  {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  // visualization::ClearVisualizationMsg(local_viz_msg_);
  printf("Current position is (%f, %f) angle %f\n", robot_loc_.x(), robot_loc_.y(), robot_angle_);
  printf("Setting navigation goal to (%f, %f) angle %f\n", loc.x(), loc.y(), angle);
  // Clear previous path
  nav_path_.clear();
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = false;
  PlanPath(robot_loc_, nav_goal_loc_, map_, nav_path_);
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  latency_compensation_->recordObservation(loc[0], loc[1], angle, ros::Time::now().toSec());
  Observation predictedState = latency_compensation_->getPredictedState();
  odom_loc_ = {predictedState.x, predictedState.y};
  odom_angle_ = predictedState.theta;
  robot_vel_ = {predictedState.vx, predictedState.vy};
  robot_omega_ = predictedState.omega;

  point_cloud_ = latency_compensation_->forward_predict_point_cloud(point_cloud_, predictedState.x, predictedState.y, predictedState.theta);
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                            
}

void Navigation::SetLatencyCompensation(LatencyCompensation* latency_compensation) {
  latency_compensation_ = latency_compensation;
}

// Convert (velocity, curvature) to (x_dot, y_dot, theta_dot)
Control Navigation::GetCartesianControl(float velocity, float curvature, double time) {
  float x_dot = velocity * cos(curvature);
  float y_dot = velocity * sin(curvature);
  float theta_dot = velocity * curvature;

  return {x_dot, y_dot, theta_dot, time};
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;
  // cout << current_speed << endl;
  // distance_traveled_ += current_speed * robot_config_.dt;
  // float dist_to_go = (10 - distance_traveled_); // hard code to make it go 10 forward
  // float cmd_vel = run1DTimeOptimalControl(dist_to_go, current_speed, robot_config_);

  // Check whether new navigation plan is needed based on goal
  // if (nav_complete_) {
  //   return;
  // }

  // Check if the robot is within a certain distance of the goal
  if ((nav_goal_loc_ - robot_loc_).norm() < 0.5) {
    nav_complete_ = true;
  }

  // If navigation is complete, stop the robot
  if (nav_complete_) {
    drive_msg_.velocity = 0;
    drive_msg_.curvature = 0;
    drive_pub_.publish(drive_msg_);
    return;
  }

  // Go through edges in reverse and check if the robot is within a certain distance of the edge
  bool on_path = false;
  // printf("Nav path size: %lu\n", nav_path_.size());
  for (size_t i = nav_path_.size() - 1; i > 0; i--) {
    Vector2f edge_start = nav_path_[i - 1];
    Vector2f edge_end = nav_path_[i];
    Vector2f projected_point = geometry::ProjectPointOntoLineSegment(robot_loc_, edge_start, edge_end);
    float dist = (projected_point - robot_loc_).norm();
    if (dist < 0.1) {
      on_path = true;
      break;
    }
  }
  // printf("On path: %d\n", on_path);
  // If the robot is not on the path, replan
  if (!on_path) {
    nav_path_.clear();
    PlanPath(robot_loc_, nav_goal_loc_, map_, nav_path_);
  }

  // Set circle radius for carrot following
  float carrot_radius = 0.5;
  visualization::DrawArc(robot_loc_, carrot_radius, 0, M_2PI, 0xFF0000, global_viz_msg_);
  Vector2f carrot_point = Vector2f(INFINITY, INFINITY);
  Vector2f carrot_point_robot = Vector2f(INFINITY, INFINITY);
  // Find point on path which intersects with circle
  for (size_t i = 0; i < nav_path_.size() - 1; i++) {
    Vector2f edge_start = nav_path_[i];
    Vector2f edge_end = nav_path_[i + 1];
    float carrot_distance = INFINITY;
    if (geometry::FurthestFreePointCircle(edge_start, edge_end, robot_loc_, carrot_radius, &carrot_distance, &carrot_point)) {
      // World to local frame transformation
      visualization::DrawCross(carrot_point, 0.1, 0xFF0000, global_viz_msg_);
      Eigen::Matrix3f world_T_local = Eigen::Matrix3f::Identity();
      world_T_local.block<2, 1>(0, 2) = robot_loc_;
      world_T_local.block<2, 2>(0, 0) << cos(robot_angle_), sin(robot_angle_), -sin(robot_angle_), cos(robot_angle_);
      carrot_point_robot = world_T_local.block<2, 2>(0, 0) * (carrot_point - robot_loc_);
      visualization::DrawCross(carrot_point_robot, 0.1, 0x00FF00, local_viz_msg_);
      break;
      // Transform carrot point to robot frame
      // float carrot_point_robot_frame_x = carrot_point.x() * cos(robot_angle_) + carrot_point.y() * sin(robot_angle_) - robot_loc_.x();
      // float carrot_point_robot_frame_y = -1 * carrot_point.x() * sin(robot_angle_) + carrot_point.y() * cos(robot_angle_) - robot_loc_.y();
    }
    // Vector2f closest_point = geometry::ProjectPointOntoLineSegment(robot_loc_, edge_start, edge_end);
    // float dist = (closest_point - robot_loc_).norm();
    // if (dist > carrot_distance) {
    //   Vector2f direction = (closest_point - robot_loc_).normalized();
    //   Vector2f carrot_point = robot_loc_ + carrot_distance * direction;
    //   nav_path_.insert(nav_path_.begin() + i + 1, carrot_point);
    //   break;
    // }
  }

  float current_speed = robot_vel_.norm();

  vector<PathOption> path_options = samplePathOptions(61, point_cloud_, robot_config_);
  int best_path = selectPath(path_options, carrot_point_robot);

  drive_msg_.curvature = path_options[best_path].curvature;
  drive_msg_.velocity = run1DTimeOptimalControl(path_options[best_path].free_path_length, current_speed, robot_config_);
	// printf("Curvature: %f, Velocity: %f\n", drive_msg_.curvature, drive_msg_.velocity);
  // cout << drive_msg_.curvature << " " << drive_msg_.velocity << endl;

  // visualization here
  visualization::DrawRectangle(Vector2f(robot_config_.length/2 - robot_config_.base_link_offset, 0),
      robot_config_.length, robot_config_.width, 0, 0x00FF00, local_viz_msg_);
  // Draw all path options in blue
  for (unsigned int i = 0; i < path_options.size(); i++) {
      visualization::DrawPathOption(path_options[i].curvature, path_options[i].free_path_length, 0, 0x0000FF, false, local_viz_msg_);
  }
  // Draw the best path in red
  visualization::DrawPathOption(path_options[best_path].curvature, path_options[best_path].free_path_length, path_options[best_path].clearance, 0xFF0000, true, local_viz_msg_);
// Find the closest point in the point cloud

  // Plot the closest point in purple
  visualization::DrawLine(path_options[best_path].closest_point, Vector2f(0, 1/path_options[best_path].curvature), 0xFF00FF, local_viz_msg_);
  // for debugging
  
    
  visualization::DrawPoint(Vector2f(0, 1/path_options[best_path].curvature), 0x0000FF, local_viz_msg_);

  for (size_t i = 0; i < nav_path_.size() - 1; i++) {
    visualization::DrawPoint(nav_path_[i], 0xFF00FF, global_viz_msg_);
    visualization::DrawPoint(nav_path_[i + 1], 0xFF00FF, global_viz_msg_);
    visualization::DrawLine(nav_path_[i], nav_path_[i + 1], 0xFFA500, global_viz_msg_);
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  // Record control for latency compensation
  Control control = GetCartesianControl(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toSec());
  latency_compensation_->recordControl(control);

  // Hack because ssh -X is slow
  // if (latency_compensation_->getControlQueue().size() == 100) {
  //  exit(0);
}

} // namespace navigation
