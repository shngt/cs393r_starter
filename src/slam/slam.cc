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
    apply_new_scan_(false),
    K1_(0.3),
    K2_(0.3),
    K3_(0.5),
    K4_(0.5),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    estimated_loc_(0, 0),
    estimated_angle_(0),
    loc_threshold_(0.5),
    angle_threshold_(M_PI / 6),
    x_freq_(11.0),
    y_freq_(11.0),
    theta_freq_(31.0),
    candidate_poses_(vector<Pose>(x_freq_ * y_freq_ * theta_freq_)),
    log_prob_grid_(500, vector<float>(500, -std::numeric_limits<float>::infinity())), 
    log_prob_grid_resolution_(0.04),
    log_prob_grid_origin_(Vector2f(-10, -10)),
    log_prob_grid_initialized_(false),
    graph_(gtsam::NonlinearFactorGraph()), // Vector3f(0.3, 0.3, 0.1).cast<double>()
    initial_estimate_(gtsam::Values()),
    pose_index_(1)
    {
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
      graph_.add(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0, 0, 0), priorNoise));
      initial_estimate_.insert(1, gtsam::Pose2(0, 0, 0));
    }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = estimated_loc_;
  *angle = estimated_angle_;
}

void SLAM::RunCSM(const vector<Vector2f>& point_cloud) {
  Pose best_pose = {{0, 0}, 0, -std::numeric_limits<float>::infinity()};
  // printf("Estimated Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);
  for (Pose& pose : candidate_poses_) {
    // printf("Candidate Pose: (%f, %f, %f), with log likelihood %f\n", 
    //   pose.loc.x(), pose.loc.y(), pose.angle, pose.log_likelihood);
    // Run CSM algorithm to align the point cloud to the pose.
    // Update best_pose if the new pose is better.
    float pose_observation_log_likelihood = 0;
    for (const Vector2f& point: point_cloud) {
      // Convert to map frame
      // Vector2f transformed_point = Rotation2Df(-pose.angle) * (pose.loc - estimated_loc_) + Rotation2Df(AngleDiff(pose.angle, estimated_angle_)) * point;
      Vector2f transformed_point = Rotation2Df(pose.angle) * point + pose.loc;
      // Convert to estimated pose frame
      transformed_point = Rotation2Df(-estimated_angle_) * (transformed_point - estimated_loc_);
      // printf("Transformed Point: (%f, %f)\n", transformed_point.x(), transformed_point.y());
      // Vector2f loc_diff = estimated_loc_ - pose.loc; // pose.loc - estimated_loc_;
      // float angle_diff = AngleDiff(estimated_angle_, pose.angle); // AngleDiff(pose.angle, estimated_angle_);
      // loc_diff = Rotation2Df(-pose.angle) * loc_diff; // Rotate the point by the pose's angle. // Rotation2Df(-estimated_angle) * loc_diff;
      // Vector2f transformed_point = Rotation2Df(angle_diff) * point + loc_diff;
      // Look up probability of the point in the log probability grid.
      Vector2f grid_point = (transformed_point - log_prob_grid_origin_) / log_prob_grid_resolution_;
      int x = grid_point.x(), y = grid_point.y();
      if (x >= 0 && x < (int) log_prob_grid_.size() && y >= 0 && y < (int) log_prob_grid_[0].size()) {
        float log_prob = log_prob_grid_[x][y];
        if (log_prob > -std::numeric_limits<float>::infinity()) {
          pose_observation_log_likelihood += log_prob;
        }
      }
    }
    // printf("Candidate Pose: (%f, %f, %f), with pose observation log likelihood %f\n", 
    // pose.loc.x(), pose.loc.y(), pose.angle, pose_observation_log_likelihood);
    // Add the motion model likelihood to the pose likelihood.
    float pose_log_likelihood = pose.log_likelihood;
    if (pose_observation_log_likelihood < 0.0) {
      pose_log_likelihood += pose_observation_log_likelihood;
    }
    pose.log_likelihood = pose_log_likelihood;
    // Update the best pose if the new pose is better.
    if (pose.log_likelihood > best_pose.log_likelihood) {
      best_pose = pose;
      // printf("Best Pose: (%f, %f, %f), Log Likelihood: %f\n", best_pose.loc.x(), best_pose.loc.y(), best_pose.angle, best_pose.log_likelihood);
      // printf("Observation Log Likelihood: %f\n", pose_observation_log_likelihood);
    }
  }
  // Construct transformed point cloud
  // printf("Rotating by angle: %f\n and translating by: (%f, %f)", best_pose.angle, best_pose.loc.x(), best_pose.loc.y());
  transformed_point_cloud_.clear();
  for (const Vector2f& point : point_cloud) {
    // Convert to global frame
    // Vector2f transformed_point_ = Rotation2Df(-best_pose.angle) * (best_pose.loc - estimated_loc_) + Rotation2Df(AngleDiff(best_pose.angle, estimated_angle_)) * point;
    Vector2f tp = Rotation2Df(best_pose.angle) * point + best_pose.loc;
    tp = Rotation2Df(-estimated_angle_) * (tp - estimated_loc_);
    transformed_point_cloud_.push_back(tp);
  }

  // printf("Best Pose: (%f, %f, %f)\n", best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);
  // Save the best pose as the new estimated pose.
  estimated_loc_ = best_pose.loc;
  estimated_angle_ = best_pose.angle;
  // Calculate covariances
  Matrix3f sigma_xi = Matrix3f::Zero();
  Matrix3f K = Matrix3f::Zero();
  Vector3f u = Vector3f::Zero();
  float s = 0.0; 
    for (auto pose : candidate_poses_) {
      Vector3f xi(pose.loc.x(), pose.loc.y(), pose.angle);
      K += xi * xi.transpose() * exp(pose.log_likelihood);
      u += xi * exp(pose.log_likelihood);
      s += exp(pose.log_likelihood);
      sigma_xi += (1/s)*K - (1/(pow(s,2)))*u*u.transpose();
    }
  covariances_.push_back(sigma_xi);

  // Solve for the joint solution over pose graph
  // Add odometry factors
  gtsam::noiseModel::Gaussian::shared_ptr noise_model = gtsam::noiseModel::Gaussian::Covariance(sigma_xi.cast<double>());
  //   // (gtsam::Matrix3(3, 3) << sigma_xi(0,0), sigma_xi(0,1), sigma_xi(0,2),
  //   //                          sigma_xi(1,0), sigma_xi(1,1), sigma_xi(1,2),
  //   //                          sigma_xi(2,0), sigma_xi(2,1), sigma_xi(2,2))
  // );
  // gtsam::noiseModel::Diagonal::shared_ptr model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
  Pose2 pose1 = Pose2(best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);
  printf("Pose: (%f, %f, %f)\n", best_pose.loc.x(), best_pose.loc.y(), best_pose.angle);
  pose_index_++;
  pose_history_.push_back(pose1);
  // printf("Pose Index: %d\n", pose_index_);
  // printf("Last pose: (%f, %f, %f)\n", pose_history_.back().x(), pose_history_.back().y(), pose_history_.back().theta());
  // printf("Relative Pose: (%f, %f, %f)\n", pose1.x(), pose1.y(), pose1.theta());
  graph_.add(BetweenFactor<Pose2>(pose_index_ - 1, pose_index_, pose1, noise_model));

  // optimize using Levenberg-Marquardt optimization
  initial_estimate_.insert(pose_index_, pose1);
  result_ = LevenbergMarquardtOptimizer(graph_, initial_estimate_).optimize();
  result_.print("Final Result:\n");
  // // Estimate pairwise non-succesive poses
  // for (int i = 2; i <= candidate_poses_.size(); i++) {
  //   for(int j = 1; j<=i-1; j++){

  //   }
  // }
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  // if (!log_prob_grid_initialized_) {

  //   log_prob_grid_initialized_ = true;
  //   return;
  // }

  if (!apply_new_scan_) {
    return;
  }

  apply_new_scan_ = false;

  vector<Vector2f> point_cloud;
  const float angle_increment = (angle_max - angle_min) / ranges.size();
  // save the pointcloud in the robot's frame
  // std::fill(point_cloud.begin(), point_cloud.end(), Vector2f(0, 0));
  for (size_t i = 0; i < ranges.size(); i++) {
      if (ranges[i] >= range_max) {
        continue;
      }
      const float angle = angle_min + i * angle_increment;
      Vector2f point(ranges[i] * cos(angle) + 0.2, ranges[i] * sin(angle));
      point_cloud.push_back(point);
  }
  // Run CSM to align the point cloud to the last saved pose
  RunCSM(point_cloud);
  
  // Print estimated location and angle
  // printf("Estimated Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);

  // Transform the point cloud to the estimated pose and add to map
  for (size_t i = 0; i < point_cloud.size(); i++) {
    // printf("point added to map\n");
    map_.push_back(Rotation2Df(estimated_angle_) * point_cloud[i] + estimated_loc_);
  }

  // Apply the new point cloud to the log probability grid
  // Reset the grid
  log_prob_grid_ = vector<vector<float>>(500, vector<float>(500, -std::numeric_limits<float>::infinity()));
  // Iterate over all points in the point cloud and update the log probability
  for (const Vector2f& point : point_cloud) {
    // Get index of point in log probability grid
    // printf("Point: (%f, %f)\n", point.x(), point.y());
    Vector2f grid_point = (point - log_prob_grid_origin_) / log_prob_grid_resolution_;
    // printf("Grid Point: (%f, %f)\n", grid_point.x(), grid_point.y());
    // int max_offset = 30 * 0.02 / log_prob_grid_resolution_;
    // // Iterate over all points in the log probability grid within max_offset of grid_point
    // for (int x_offset = -max_offset; x_offset <= max_offset; x_offset++) {
    //   for (int y_offset = -max_offset; y_offset <= max_offset; y_offset++) {
    //     int x = grid_point.x() + x_offset, y = grid_point.y() + y_offset;
    //     if (x >= 0 && x < (int) log_prob_grid_.size() && y >= 0 && y < (int) log_prob_grid_[0].size()) {
    //       float x_dist = x_offset * log_prob_grid_resolution_;
    //       float y_dist = y_offset * log_prob_grid_resolution_;
    //       // printf("X Dist: %f, Y Dist: %f\n", x_dist, y_dist);
    //       // float dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    //       float log_prob = -(x_dist * x_dist + y_dist * y_dist) / (2 * 0.01 * 0.01);
    //       // printf("Log Prob: %f\n", log_prob);
    //       log_prob_grid_[x][y] = std::max(log_prob_grid_[x][y], log_prob);
    //     }
    //   }
    // }
    int x = grid_point.x(), y = grid_point.y();
    // Iterate over grid
    for (int i = 0; i < (int) log_prob_grid_.size(); i++) {
      for (int j = 0; j < (int) log_prob_grid_[0].size(); j++) {
        float x_dist = (i - x) * log_prob_grid_resolution_;
        float y_dist = (j - y) * log_prob_grid_resolution_;
        float log_prob = -(x_dist * x_dist + y_dist * y_dist) / (2 * 0.01 * 0.01);
        log_prob_grid_[i][j] = std::max(log_prob_grid_[i][j], log_prob);
      }
    }
    log_prob_grid_initialized_ = true;
  }

  apply_new_scan_ = false;
}

void SLAM::PredictMotionModel(float loc_diff, float angle_diff, Vector2f current_pose_loc, float current_pose_angle) {
  float angle_stddev = K1_ * angle_diff + K2_ * loc_diff;
  float loc_stddev = K3_ * angle_diff + K4_ * loc_diff;

  // Cache sin and cos for efficieny
  float cos_pose_angle = cos(current_pose_angle);
  float sin_pose_angle = sin(current_pose_angle);

  // Iterate over all possible combinations of pose (x, y, theta)
  for (int x_r = 0; x_r < x_freq_; x_r++) {
    float x_r_norm = (2 * x_r / (x_freq_ - 1) - 1);
		float del_x = loc_stddev * x_r_norm;
    for (int y_r = 0; y_r < y_freq_; y_r++) {
      float y_r_norm = (2 * y_r / (y_freq_ - 1) - 1);
			float del_y = loc_stddev * y_r_norm;
      for (int theta_r = 0; theta_r < theta_freq_; theta_r++) {
        float theta_r_norm = (2 * theta_r / (theta_freq_ - 1) - 1);
				float del_theta = angle_stddev * theta_r_norm;

				float x_pose = current_pose_loc.x() + del_x * cos_pose_angle - del_y * sin_pose_angle;
				float y_pose = current_pose_loc.y() + del_x * sin_pose_angle + del_y * cos_pose_angle;
				float theta_pose = current_pose_angle + del_theta;

				// Calculate the motion model likelihood
				// float log_likelihood = -(x_noise*x_noise)/(loc_stddev*loc_stddev) 
				// 				   -(y_noise*y_noise)/(loc_stddev*loc_stddev) 
				// 				   -(t_noise*t_noise)/(t_stddev*t_stddev);

        float log_likelihood = -1.0 * (x_r_norm * x_r_norm + y_r_norm * y_r_norm + theta_r_norm * theta_r_norm) / 2.0 ;

        int idx = x_r * y_freq_ * theta_freq_ + y_r * theta_freq_ + theta_r;
				candidate_poses_[idx] = {{x_pose, y_pose}, theta_pose, log_likelihood};
        // printf("Candidate Pose: (%f, %f, %f), with log likelihood %f\n", 
        //   candidate_poses_[idx].loc.x(), candidate_poses_[idx].loc.y(), candidate_poses_[idx].angle, candidate_poses_[idx].log_likelihood);
			}
		}
	}
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  // printf("Odom Loc: (%f, %f), Odom Angle: %f\n", odom_loc.x(), odom_loc.y(), odom_angle);
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  Vector2f odom_loc_diff = odom_loc - prev_odom_loc_;
  float odom_angle_diff = AngleDiff(odom_angle, prev_odom_angle_);
  printf("Odom Diff: (%f, %f, %f)\n", odom_loc_diff.x(), odom_loc_diff.y(), odom_angle_diff);
  float dist = odom_loc_diff.norm();
  // printf("Estimated Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);
  // Project the estimated pose based on the odometry difference
  Vector2f projected_loc =  Rotation2Df(estimated_angle_) * odom_loc_diff; // estimated_loc_ + odom_loc_diff; //

  // odom diff observed in odom frame - have to convert to estimated pose frame
  // Vector2f projected_loc = estimated_loc_ + Rotation2Df(estimated_angle_ - prev_odom_angle_) * odom_loc_diff;
  float projected_angle = AngleMod(estimated_angle_ + odom_angle_diff); // fmod(estimated_angle_ + odom_angle_diff + M_PI, 2*M_PI) - M_PI; // 

  printf("Projected Pose: (%f, %f, %f)\n", projected_loc.x(), projected_loc.y(), projected_angle);

  if (dist > loc_threshold_ || fabs(odom_angle_diff) > angle_threshold_) {
    // Update the candidate poses based on the odometry
    PredictMotionModel(dist, fabs(odom_angle_diff), projected_loc, projected_angle);
    // Set flag to add new scan
    apply_new_scan_ = true;
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
}

vector<Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  // Subsample 5000 points from the map and return
  int step = ceil((float) map_.size() / 5000);
  vector<Vector2f> subsampled_map;
  for (size_t i = 0; i < map_.size(); i += step) {
    subsampled_map.push_back(map_[i]);
  }
  return subsampled_map;
}

}  // namespace slam
