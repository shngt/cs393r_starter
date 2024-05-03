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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/NoiseModel.h>
// #include <opencv2/opencv.hpp>

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;
using gtsam::Pose2;
using gtsam::BetweenFactor;
using gtsam::LevenbergMarquardtOptimizer;

namespace slam {

SLAM::SLAM() :
    K1_(0.3),
    K2_(0.3),
    K3_(0.5),
    K4_(0.5),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    // estimated_loc_(0, 0),
    // estimated_angle_(0),
    loc_threshold_(0.3),
    angle_threshold_(M_PI / 12),
    apply_new_scan_(false),
    x_freq_(11.0),
    y_freq_(11.0),
    theta_freq_(31.0),
    // candidate_poses(vector<Pose>(x_freq_ * y_freq_ * theta_freq_)),
    // log_prob_grid_(500, vector<double>(500, -std::numeric_limits<double>::infinity())), 
    log_prob_grid_resolution_(0.03),
    log_prob_grid_origin_(Vector2f(-10, -10)),
    log_prob_grid_initialized_(false),
    graph_(gtsam::NonlinearFactorGraph()), // Vector3f(0.3, 0.3, 0.1).cast<double>()
    log_prob_grid_history_(std::vector<std::vector<std::vector<double>>>()),
    pose_history_(std::vector<Pose2>()),
    initial_estimate_(gtsam::Values()),
    pose_index_(1)
    {
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
      graph_.add(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));
      initial_estimate_.insert(1, gtsam::Pose2(0, 0, 0));
      pose_history_.push_back(gtsam::Pose2(0, 0, 0));
    }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  if (pose_index_ <= 1) {
    *loc = {0, 0};
    *angle = 0;
    return;
  }
  Pose2 pose = result_.at<Pose2>(pose_index_);
  *loc = {pose.x(), pose.y()};
  *angle = pose.theta();
  return;
}

void SLAM::RunCSM(
  const vector<Vector2f>& point_cloud, 
  const Vector2f& old_loc,
  const double old_angle,
  const vector<vector<double>>& old_log_prob_grid, 
  std::vector<Pose>& candidate_poses,
  Eigen::Matrix3f& covariance,
  Pose2& new_pose
) {
  // static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  // CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  Pose best_pose = {{0, 0}, 0, -std::numeric_limits<double>::infinity()};
  // printf("Old Pose: (%f, %f, %f)\n", old_loc.x(), old_loc.y(), old_angle);
  for (Pose& pose : candidate_poses) {
    // Run CSM algorithm to align the point cloud to the pose.
    // Update best_pose if the new pose is better.
    double pose_observation_log_likelihood = 1;
    for (const Vector2f& point: point_cloud) {
      // Transform the point to the pose's frame.
      // Vector2f transformed_point =  Rotation2Df(-pose.angle) * (pose.loc - old_loc) + Rotation2Df(AngleDiff(pose.angle, old_angle)) * point;
      Vector2f transformed_point = Rotation2Df(pose.angle) * point + pose.loc;
      transformed_point = Rotation2Df(-old_angle) * (transformed_point - old_loc);
      // Vector2f loc_diff = pose.loc - estimated_loc_;
      // double angle_diff = AngleDiff(pose.angle, estimated_angle_);
      // loc_diff = Rotation2Df(-pose.angle) * loc_diff; // Rotate the point by the pose's angle. // Rotation2Df(-estimated_angle) * loc_diff;
      // Vector2f transformed_point = Rotation2Df(angle_diff) * point + loc_diff;
      // Look up probability of the point in the log probability grid.
      Vector2f grid_point = (transformed_point - log_prob_grid_origin_) / log_prob_grid_resolution_;
      int x = grid_point.x(), y = grid_point.y();
      if (x >= 0 && x < (int) old_log_prob_grid.size() && y >= 0 && y < (int) old_log_prob_grid[0].size()) {
        double log_prob = old_log_prob_grid[x][y];
        // if (log_prob > -std::numeric_limits<double>::infinity()) {
        pose_observation_log_likelihood *= log_prob;
        // }
      }
    }
    // Add the motion model likelihood to the pose likelihood.
    double pose_log_likelihood = pose_observation_log_likelihood * pose.log_likelihood; //  0.00008 * pose_observation_log_likelihood + 
    // printf("Pose: (%f, %f, %f), Log Likelihood: %f\n", pose.loc.x(), pose.loc.y(), pose.angle, pose_log_likelihood);
    pose.log_likelihood = pose_log_likelihood;
    // pose.log_likelihood += pose_observation_log_likelihood;
    // Update the best pose if the new pose is better.
    if (pose.log_likelihood > best_pose.log_likelihood) {
      best_pose = pose;
    }
  }

  // transformed_point_cloud_.clear();
  // for (const Vector2f& point : point_cloud) {
  //   // Convert to global frame
  //   Vector2f tp = Rotation2Df(-best_pose.angle) * (best_pose.loc - old_loc) + Rotation2Df(AngleDiff(best_pose.angle, old_angle)) * point;
  //   // Vector2f tp = Rotation2Df(best_pose.angle) * point + best_pose.loc;
  //   // tp = Rotation2Df(-old_angle) * (tp - old_loc);
  //   transformed_point_cloud_.push_back(tp);
  // }
  // printf("Best Pose: (%f, %f, %f)\n", best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);

  // Extract best pose location and angle and add to pose history
  new_pose = Pose2(best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);

  // // Save the best pose as the new estimated pose.
  // estimated_loc_ = best_pose.loc;
  // estimated_angle_ = best_pose.angle;
  // // printf("Best Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);
  // Calculate covariances
  Matrix3f sigma_xi = Matrix3f::Zero();
  Matrix3f K = Matrix3f::Zero();
  Vector3f u = Vector3f::Zero();
  double s = 0.0; 
  for (auto pose : candidate_poses) {
    Vector3f xi(pose.loc.x(), pose.loc.y(), pose.angle);
    // printf("xi: (%f, %f, %f)\n", xi(0), xi(1), xi(2));
    // cout << "xi * xi.transpose(): " << xi * xi.transpose() << endl;
    // printf("exp(pose.log_likelihood): %f\n", exp(pose.log_likelihood));
    // printf("pose.log_likelihood: %f\n", pose.log_likelihood);
    K += xi * xi.transpose() * pose.log_likelihood;
    u += xi * pose.log_likelihood;
    s += pose.log_likelihood;
    sigma_xi += (1/s)*K - (1/(pow(s,2)))*u*u.transpose();
  }
  covariance = sigma_xi;
  // exit(0);
}

void SLAM::ConstructLogProbGrid(const vector<Vector2f>& point_cloud) {
  // static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  // CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Construct the log probability grid from the point cloud.
  // Iterate over all points in the point cloud and update the log probability
  // grid.
  vector<vector<double>> log_prob_grid(500, vector<double>(500, 0.0));
    // Iterate over all points in the point cloud and update the log probability grid
  // double max_val = -std::numeric_limits<double>::infinity();
  // double min_val = std::numeric_limits<double>::infinity();
  // double mean_val = 0;
  for (const Vector2f& point : point_cloud) {
    // Get index of point in log probability grid
    Vector2f grid_point = (point - log_prob_grid_origin_) / log_prob_grid_resolution_;
    int x = grid_point.x(), y = grid_point.y();
    // Iterate over grid
    for (int i = 0; i < (int) log_prob_grid.size(); i++) {
      for (int j = 0; j < (int) log_prob_grid[0].size(); j++) {
        double x_dist = (i - x) * log_prob_grid_resolution_;
        double y_dist = (j - y) * log_prob_grid_resolution_;
        double log_prob = exp(-1 * (x_dist * x_dist + y_dist * y_dist) / (2 * 0.01));
        log_prob_grid[i][j] = std::max(log_prob_grid[i][j], log_prob);
      }
    }
    // Iterate over small neighborhood around the point
    // int max_offset = 10 * 0.02 / log_prob_grid_resolution_;
    // for (int i = -max_offset; i <= max_offset; i++) {
    //   for (int j = -max_offset; j <= max_offset; j++) {
    //     int x_new = x + i, y_new = y + j;
    //     if (x_new >= 0 && x_new < (int) log_prob_grid.size() && y_new >= 0 && y_new < (int) log_prob_grid[0].size()) {
    //       printf("x_new: %d, y_new: %d\n", x_new, y_new);
    //       double x_dist = i * log_prob_grid_resolution_;
    //       double y_dist = j * log_prob_grid_resolution_;
    //       double dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    //       double log_prob = exp(-dist * dist / (2 * 0.01));
    //       log_prob_grid[x_new][y_new] = std::max(log_prob_grid[x_new][y_new], log_prob);
    //     }
    //   }
    // }
  }
  // for (int i = 0; i < (int) log_prob_grid.size(); i++) {
  //   for (int j = 0; j < (int) log_prob_grid[0].size(); j++) {
  //     max_val = std::max(max_val, log_prob_grid[i][j]);
  //     min_val = std::min(min_val, log_prob_grid[i][j]);
  //     mean_val += log_prob_grid[i][j];
  //   }
  // }
  // printf("Max Value: %f, Min Value: %f\n", max_val, min_val);
  // mean_val /= (500 * 500);
  // printf("Mean Value: %f\n", mean_val);

  // vector<vector<double>> point_cloud_grid(500, vector<double>(500, 0));
  // for (const Vector2f& point : point_cloud) {
  //   // Get index of point in log probability grid
  //   Vector2f grid_point = (point - log_prob_grid_origin_) / log_prob_grid_resolution_;
  //   int x = grid_point.x(), y = grid_point.y();
  //   // Set a neighborhood around the point to 1
  //   for (int i = -2; i <= 2; i++) {
  //     for (int j = -2; j <= 2; j++) {
  //       int x_new = x + i, y_new = y + j;
  //       if (x_new >= 0 && x_new < (int) point_cloud_grid.size() && y_new >= 0 && y_new < (int) point_cloud_grid[0].size()) {
  //         point_cloud_grid[x_new][y_new] = 1;
  //       }
  //     }
  //   }
  // }


  // Visualize log_prob_grid with opencv
  // cv::Mat log_prob_grid_img(500, 500, CV_8UC3);
  // cv::Mat point_cloud_grid_img(500, 500, CV_8UC3);
  // for (int i = 0; i < (int) log_prob_grid.size(); i++) {
  //   for (int j = 0; j < (int) log_prob_grid[0].size(); j++) {
  //     log_prob_grid_img.at<cv::Vec3b>(j, i) = cv::Vec3b(255 * (log_prob_grid[i][j] - min_val) / (max_val - min_val), 0, 0);
  //     point_cloud_grid_img.at<cv::Vec3b>(j, i) = cv::Vec3b(255 * point_cloud_grid[i][j], 0, 0);
  //   }
  // }

  // cv::normalize(log_prob_grid_img, log_prob_grid_img, 0, 1, cv::NORM_MINMAX);

  // cv::imwrite("log_prob_grid.png", log_prob_grid_img);
  // cv::imwrite("point_cloud_grid.png", point_cloud_grid_img);
  // exit(0);

  log_prob_grid_history_.push_back(log_prob_grid);
}

void SLAM::ConstructPointCloud(const sensor_msgs::LaserScan& msg, vector<Vector2f>& point_cloud) {
  // Construct a point cloud from the laser scan.
  const double angle_increment = (msg.angle_max - msg.angle_min) / msg.ranges.size();
  for (size_t i = 0; i < msg.ranges.size(); i++) {
    if (msg.ranges[i] >= msg.range_max || msg.ranges[i] < msg.range_min) {
      continue;
    }
    const double angle = msg.angle_min + i * angle_increment;
    Vector2f point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));
    point_cloud.push_back(point);
  }
  // Decimate the point cloud by taking every 10th point.
  vector<Vector2f> decimated_point_cloud;
  DecimatePointCloud(point_cloud, decimated_point_cloud);
  point_cloud = decimated_point_cloud;
}

void SLAM::DecimatePointCloud(const vector<Vector2f>& point_cloud, vector<Vector2f>& decimated_point_cloud) {
  // Decimate the point cloud by taking every 10th point.
  for (size_t i = 0; i < point_cloud.size(); i += 10) {
    decimated_point_cloud.push_back(point_cloud[i]);
  }
}

void SLAM::ObserveLaser(const sensor_msgs::LaserScan& msg) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  // static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  // CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Always add first scan
  if (!odom_initialized_) {
    return;
  }
  if (odom_initialized_ && !log_prob_grid_initialized_) {
    odometry_pose_history_.push_back({prev_odom_loc_, prev_odom_angle_, 0.0});
    // printf("Initializing log prob grid for the first time\n");
    vector<Vector2f> point_cloud;
    ConstructPointCloud(msg, point_cloud);
    ConstructLogProbGrid(point_cloud);
    log_prob_grid_initialized_ = true;
    Vector2f estimated_loc = Vector2f(pose_history_.back().x(), pose_history_.back().y());
    double estimated_angle = pose_history_.back().theta();
    for (size_t i = 0; i < point_cloud.size(); i++) {
      map_.push_back(Rotation2Df(estimated_angle) * point_cloud[i] + estimated_loc);
    }
    return;
  }
  if (!apply_new_scan_) { //  && (!odom_initialized_ || log_prob_grid_initialized_
    return;
  }
  // printf("%d, %d, %d\n", apply_new_scan_, odom_initialized_, log_prob_grid_initialized_);
  apply_new_scan_ = false;
  pose_index_++;

  vector<Vector2f> point_cloud;
  ConstructPointCloud(msg, point_cloud);
  // printf("Pose Index: %d\n", pose_index_);

  // For loop to iterate over the last 3 poses
  assert(odometry_pose_history_.size() <= 3);
  for (int i = 0; i < (int) odometry_pose_history_.size(); i++) {
    // Get old pose from pose_history_
    int pose_history_index = pose_index_ - 1 - odometry_pose_history_.size() + i;
    // printf("Pose History Index: %d\n", pose_history_index);
    Pose2 old_pose = pose_history_[pose_history_index];
    Vector2f old_pose_loc = Vector2f(old_pose.x(), old_pose.y());
    double old_pose_angle = old_pose.theta();
    
    // Get corresponding odometry pose from odometry_pose_history_
    Vector2f old_odom_loc = odometry_pose_history_[i].loc;
    double old_odom_angle = odometry_pose_history_[i].angle;

    // printf("Odometry Pose: (%f, %f, %f)\n", old_odom_loc.x(), old_odom_loc.y(), old_odom_angle);

    // Get list of candidate poses with likelihood
    std::vector<Pose> candidate_poses(x_freq_ * y_freq_ * theta_freq_);
    PredictMotionModel(old_odom_loc, old_odom_angle, old_pose_loc, old_pose_angle, candidate_poses);

    // Get old log prob grid from history
    vector<vector<double>> old_log_prob_grid = log_prob_grid_history_[i];

    // Run CSM to align the point cloud to the last saved pose
    Eigen::Matrix3f covariance;
    Pose2 new_pose;
    RunCSM(point_cloud, old_pose_loc, old_pose_angle, old_log_prob_grid, candidate_poses, covariance, new_pose);

    // // Hardcode covariance matrix for now
    // covariance = Matrix3f::Identity();

    // Print covariance matrix
    // printf("Covariance Matrix: \n");
    // cout << covariance << endl;

    // Add factor to pose graph
    // Solve for the joint solution over pose graph
    // Add odometry factors
    gtsam::noiseModel::Gaussian::shared_ptr noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance.cast<double>());
    //   // (gtsam::Matrix3(3, 3) << sigma_xi(0,0), sigma_xi(0,1), sigma_xi(0,2),
    //   //                          sigma_xi(1,0), sigma_xi(1,1), sigma_xi(1,2),
    //   //                          sigma_xi(2,0), sigma_xi(2,1), sigma_xi(2,2))
    // );

    // printf("Adding factor between %d and %d\n", pose_history_index + 1, pose_index_);
    graph_.add(BetweenFactor<Pose2>(pose_history_index + 1, pose_index_, old_pose.between(new_pose), noise_model)); // new_pose.between(old_pose)

    // If from closest previous pose, add to initial estimate
    if (i == (int) odometry_pose_history_.size() - 1) {
      initial_estimate_.insert(pose_index_, new_pose);
    }
  }
  // Save previous odom poses
  odometry_pose_history_.push_back({prev_odom_loc_, prev_odom_angle_, 0.0});
  // Save log prob grid
  ConstructLogProbGrid(point_cloud);
  // Keep only last 3 entries of odometry_pose_history_
  if (odometry_pose_history_.size() > 3) {
    odometry_pose_history_.erase(odometry_pose_history_.begin());
  }
  // Keep only last 3 entries of log_prob_grid_history_
  if (log_prob_grid_history_.size() > 3) {
    log_prob_grid_history_.erase(log_prob_grid_history_.begin());
  }

  // for(int i = 1; i<= point_cloud_history_.size(); i++){
  //   // Get previous point cloud from point_cloud_history_
  //   vector<Vector2f> prev_point_cloud = point_cloud_history_[point_cloud_history_.size() - i];
    
  //   // Get corresponding odometry pose from odometry_pose_history_
  //   Pose odom_pose = odometry_pose_history_[odometry_pose_history_.size() - i];

  // // Get list of candidate poses with likelihood
  // std::vector<Pose> motion_model_likelihood = PredictMotionModel(loc_diff, angle_diff, projected_loc, projected_angle);

  // // Run CSM to align the point cloud to the last saved pose
  // covariance = RunCSM(point_cloud, previous_point_cloud, succesive_motion_model_likelihood);

  // // Add factor to pose graph
  // // Solve for the joint solution over pose graph
  // // Add odometry factors
  // gtsam::noiseModel::Gaussian::shared_ptr noise_model = gtsam::noiseModel::Gaussian::Covariance(sigma_xi.cast<double>());
  // //   // (gtsam::Matrix3(3, 3) << sigma_xi(0,0), sigma_xi(0,1), sigma_xi(0,2),
  // //   //                          sigma_xi(1,0), sigma_xi(1,1), sigma_xi(1,2),
  // //   //                          sigma_xi(2,0), sigma_xi(2,1), sigma_xi(2,2))
  // // );
  // // gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
  // Pose2 pose1 = Pose2(best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);
  // graph_.add(BetweenFactor<Pose2>(pose_index_ - i, pose_index_, pose1, noise_model));

  // // optimize using Levenberg-Marquardt optimization
  // initial_estimate_.insert(pose_index_, pose1);
  // }

  // Optimization results
  if (pose_index_ > 1) {}
  result_ = LevenbergMarquardtOptimizer(graph_, initial_estimate_).optimize();
  // Iterate over result and update pose history
  pose_history_.clear();
  for (int i = 1; i <= pose_index_; i++) {
    Pose2 pose = result_.at<Pose2>(i);
    pose_history_.push_back(pose);
  }
  // result_.print("Final Result:\n");
  // if (pose_index_ == 5) exit(0);
  // Estimate pairwise non-succesive poses
  
  // Print estimated location and angle
  // printf("Estimated Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);

  // Save point cloud history
  // point_cloud_history_.push_back(point_cloud);

  // Transform the point cloud to the estimated pose and add to map
  Vector2f estimated_loc = Vector2f(result_.at<Pose2>(pose_index_).x(), result_.at<Pose2>(pose_index_).y());
  double estimated_angle = result_.at<Pose2>(pose_index_).theta();
  // Pose2 relative_pose = result_.at<Pose2>(pose_index_ - 1).between(result_.at<Pose2>(pose_index_));
  // transformed_point_cloud_.clear();
  // printf("Estimated Pose before adding: (%f, %f, %f)\n", estimated_loc.x(), estimated_loc.y(), estimated_angle);
  for (size_t i = 0; i < point_cloud.size(); i++) {
    map_.push_back(Rotation2Df(estimated_angle) * point_cloud[i] + estimated_loc);
    // transformed_point_cloud_.push_back(Rotation2Df(relative_pose.theta()) * point_cloud[i] + Vector2f({relative_pose.x(), relative_pose.y()}));
  }

  // Apply the new point cloud to the log probability grid
  // Iterate over all points in the point cloud and update the log probability
  // for (const Vector2f& point : point_cloud) {
  //   // Get index of point in log probability grid
  //   Vector2f grid_point = (point - log_prob_grid_origin_) / log_prob_grid_resolution_;
  //   int max_offset = 30 * 0.02 / log_prob_grid_resolution_;
  //   // Iterate over all points in the log probability grid within max_offset of grid_point
  //   for (int x_offset = -max_offset; x_offset <= max_offset; x_offset++) {
  //     for (int y_offset = -max_offset; y_offset <= max_offset; y_offset++) {
  //       int x = grid_point.x() + x_offset, y = grid_point.y() + y_offset;
  //       if (x >= 0 && x < (int) log_prob_grid_.size() && y >= 0 && y < (int) log_prob_grid_[0].size()) {
  //         double x_dist = x_offset * log_prob_grid_resolution_;
  //         double y_dist = y_offset * log_prob_grid_resolution_;
  //         double dist = sqrt(x_dist * x_dist + y_dist * y_dist);
  //         double log_prob = -dist * dist / (2 * 0.01 * 0.01);
  //         log_prob_grid_[x][y] = std::max(log_prob_grid_[x][y], log_prob);
  //       }
  //     }
  //   }
    // log_prob_grid_initialized_ = true;
    // Logprob grid history
    // log_prob_grid_history_.push_back(log_prob_grid_);
}

  // apply_new_scan_ = false;

  // Run GTSAM, place covariance in prior noise

// Function returns the candidate poses based on the motion model
void SLAM::PredictMotionModel(
  const Vector2f& old_odom_loc, 
  const double old_odom_angle, 
  const Vector2f& old_pose_loc, 
  const double old_pose_angle, 
  vector<Pose>& candidate_poses
) {
  // static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  // CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  Vector2f odom_loc_diff = prev_odom_loc_ - old_odom_loc;
  double loc_dist = odom_loc_diff.norm();

  double odom_angle_diff = AngleDiff(prev_odom_angle_, old_odom_angle);
  double angle_dist = AngleDist(prev_odom_angle_, old_odom_angle);
  // printf("Odom Diff: (%f, %f, %f)\n", odom_loc_diff.x(), odom_loc_diff.y(), odom_angle_diff);
  // printf("loc_dist: %f, angle_dist: %f\n", loc_dist, angle_dist);
  Vector2f projected_loc = old_pose_loc + Rotation2Df(old_pose_angle - old_odom_angle) * odom_loc_diff;
  // Vector2f projected_loc = old_pose_loc + Rotation2Df(-old_odom_angle) * odom_loc_diff;
  double projected_angle = AngleMod(old_pose_angle + odom_angle_diff);

  double angle_stddev = K1_ * angle_dist + K2_ * loc_dist;
  double loc_stddev = K3_ * angle_dist + K4_ * loc_dist;
  assert(angle_stddev > 0 && loc_stddev > 0);

  // Cache sin and cos for efficieny
  double cos_pose_angle = cos(projected_angle);
  double sin_pose_angle = sin(projected_angle);

  // Iterate over all possible combinations of pose (x, y, theta)
  for (int x_r = 0; x_r < x_freq_; x_r++) {
    double x_r_norm = 0.25 * (2 * x_r / (x_freq_ - 1) - 1);
		double del_x = loc_stddev * x_r_norm;
    // printf("del_x: %f\n", del_x);
    for (int y_r = 0; y_r < y_freq_; y_r++) {
      double y_r_norm = 0.25 * (2 * y_r / (y_freq_ - 1) - 1);
			double del_y = loc_stddev * y_r_norm;
      // printf("del_y: %f\n", del_y);
      for (int theta_r = 0; theta_r < theta_freq_; theta_r++) {
        double theta_r_norm = (M_PI / 8) * (2 * theta_r / (theta_freq_ - 1) - 1);
				double del_theta = angle_stddev * theta_r_norm;
        // printf("del_theta: %f\n", del_theta);

				double x_pose = projected_loc.x() + del_x * cos_pose_angle - del_y * sin_pose_angle;
				double y_pose = projected_loc.y() + del_x * sin_pose_angle + del_y * cos_pose_angle;
				double theta_pose = projected_angle + del_theta;

				// Calculate the motion model likelihood
				// double log_likelihood = -(x_noise*x_noise)/(loc_stddev*loc_stddev) 
				// 				   -(y_noise*y_noise)/(loc_stddev*loc_stddev) 
				// 				   -(t_noise*t_noise)/(t_stddev*t_stddev);

        double log_likelihood = exp(-1.0 * (x_r_norm * x_r_norm + y_r_norm * y_r_norm) / (2.0)
                                  -1.0 * (theta_r_norm * theta_r_norm) / 2.0);

        int idx = x_r * y_freq_ * theta_freq_ + y_r * theta_freq_ + theta_r;
        // printf("idx: %d\n", idx);
				candidate_poses[idx] = {{x_pose, y_pose}, theta_pose, log_likelihood};
        // printf("Candidate Pose: (%f, %f, %f), with log likelihood %f\n", 
        //   candidate_poses[idx].loc.x(), candidate_poses[idx].loc.y(), candidate_poses[idx].angle, candidate_poses[idx].log_likelihood);
        
      }
		}
	}
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const double odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    // apply_new_scan_ = true;
    // print("Odom initialized with: %f, %f, %f\n", prev_odom_loc_.x(), prev_odom_loc_.y(), prev_odom_angle_);
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  // printf("Previous Odom from ObserveOdometry: (%f, %f, %f)\n", prev_odom_loc_.x(), prev_odom_loc_.y(), prev_odom_angle_);
  Vector2f odom_loc_diff = odom_loc - prev_odom_loc_;
  // Calculate the difference between the odometry and the predicted motion model
  // Vector2f loc_delta = Rotation2Df(-prev_odom_angle_) * (odom_loc - prev_odom_loc_);
  // double loc_diff = loc_delta.norm();
  double loc_diff = odom_loc_diff.norm();
  // double angle_delta = odom_angle - prev_odom_angle_;
  // double angle_diff = fabs(angle_delta - M_2PI * floor(angle_delta / M_2PI + 0.5));
  double angle_diff = AngleDist(odom_angle, prev_odom_angle_);

  // printf("Odom Diff: (%f, %f, %f)\n", odom_loc_diff.x(), odom_loc_diff.y(), AngleDiff(odom_angle, prev_odom_angle_));
  // double dist = odom_diff.norm();

  if (loc_diff > loc_threshold_ || angle_diff > angle_threshold_) {
    // Update the candidate poses based on the odometry
    // Set flag to add new scan
    // printf("Flagged to register new scan");
    apply_new_scan_ = true;
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
}

vector<Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  // Subsample 5000 points from the map and return
  int step = ceil((double) map_.size() / 5000);
  vector<Vector2f> subsampled_map;
  for (size_t i = 0; i < map_.size(); i += step) {
    subsampled_map.push_back(map_[i]);
  }
  return subsampled_map;
}

}  // namespace slam
