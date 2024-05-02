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
    apply_new_scan_(false),
    x_freq_(11.0),
    y_freq_(11.0),
    theta_freq_(31.0),
    // candidate_poses(vector<Pose>(x_freq_ * y_freq_ * theta_freq_)),
    // log_prob_grid_(500, vector<float>(500, -std::numeric_limits<float>::infinity())), 
    log_prob_grid_resolution_(0.04),
    log_prob_grid_origin_(Vector2f(-10, -10)),
    log_prob_grid_initialized_(false),
    graph_(gtsam::NonlinearFactorGraph()), // Vector3f(0.3, 0.3, 0.1).cast<double>()
    log_prob_grid_history_(std::vector<std::vector<std::vector<float>>>()),
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
  const float old_angle,
  const vector<vector<float>>& old_log_prob_grid, 
  std::vector<Pose>& candidate_poses,
  Eigen::Matrix3f& covariance,
  Pose2& new_pose
) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  Pose best_pose = {{0, 0}, 0, -std::numeric_limits<float>::infinity()};
  for (Pose& pose : candidate_poses) {
    // Run CSM algorithm to align the point cloud to the pose.
    // Update best_pose if the new pose is better.
    float pose_observation_log_likelihood = 0;
    for (const Vector2f& point: point_cloud) {
      // Transform the point to the pose's frame.
      Vector2f transformed_point = Rotation2Df(pose.angle) * point + pose.loc;
      transformed_point = Rotation2Df(-old_angle) * (transformed_point - old_loc);
      // Vector2f loc_diff = pose.loc - estimated_loc_;
      // float angle_diff = AngleDiff(pose.angle, estimated_angle_);
      // loc_diff = Rotation2Df(-pose.angle) * loc_diff; // Rotate the point by the pose's angle. // Rotation2Df(-estimated_angle) * loc_diff;
      // Vector2f transformed_point = Rotation2Df(angle_diff) * point + loc_diff;
      // Look up probability of the point in the log probability grid.
      Vector2f grid_point = (transformed_point - log_prob_grid_origin_) / log_prob_grid_resolution_;
      int x = grid_point.x(), y = grid_point.y();
      if (x >= 0 && x < (int) old_log_prob_grid.size() && y >= 0 && y < (int) old_log_prob_grid[0].size()) {
        float log_prob = old_log_prob_grid[x][y];
        if (log_prob > -std::numeric_limits<float>::infinity()) {
          pose_observation_log_likelihood += log_prob;
        }
      }
    }
    // Add the motion model likelihood to the pose likelihood.
    float pose_log_likelihood = pose_observation_log_likelihood + pose.log_likelihood;
    // printf("Pose: (%f, %f, %f), Log Likelihood: %f\n", pose.loc.x(), pose.loc.y(), pose.angle, pose_log_likelihood);
    pose.log_likelihood = pose_log_likelihood;
    // pose.log_likelihood += pose_observation_log_likelihood;
    // Update the best pose if the new pose is better.
    if (pose.log_likelihood > best_pose.log_likelihood) {
      best_pose = pose;
    }
  }
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
  float s = 0.0; 
  for (auto pose : candidate_poses) {
    Vector3f xi(pose.loc.x(), pose.loc.y(), pose.angle);
    K += xi * xi.transpose() * exp(pose.log_likelihood);
    u += xi * exp(pose.log_likelihood);
    s += exp(pose.log_likelihood);
    sigma_xi += (1/s)*K - (1/(pow(s,2)))*u*u.transpose();
  }
  covariance = sigma_xi;
}

void SLAM::ConstructLogProbGrid(const vector<Vector2f>& point_cloud) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Construct the log probability grid from the point cloud.
  // Iterate over all points in the point cloud and update the log probability
  // grid.
  vector<vector<float>> log_prob_grid(500, vector<float>(500, -std::numeric_limits<float>::infinity()));
    // Iterate over all points in the point cloud and update the log probability grid
  for (const Vector2f& point : point_cloud) {
    // Get index of point in log probability grid
    Vector2f grid_point = (point - log_prob_grid_origin_) / log_prob_grid_resolution_;
    int x = grid_point.x(), y = grid_point.y();
    // Iterate over grid
    for (int i = 0; i < (int) log_prob_grid.size(); i++) {
      for (int j = 0; j < (int) log_prob_grid[0].size(); j++) {
        float x_dist = (i - x) * log_prob_grid_resolution_;
        float y_dist = (j - y) * log_prob_grid_resolution_;
        float log_prob = -1 * (x_dist * x_dist + y_dist * y_dist) / (2 * 0.02 * 0.02);
        log_prob_grid[i][j] = std::max(log_prob_grid[i][j], log_prob);
      }
    }
  }
  log_prob_grid_history_.push_back(log_prob_grid);
}

void SLAM::ConstructPointCloud(const sensor_msgs::LaserScan& msg, vector<Vector2f>& point_cloud) {
  // Construct a point cloud from the laser scan.
  const float angle_increment = (msg.angle_max - msg.angle_min) / msg.ranges.size();
  for (size_t i = 0; i < msg.ranges.size(); i++) {
    if (msg.ranges[i] >= msg.range_max || msg.ranges[i] < msg.range_min) {
      continue;
    }
    const float angle = msg.angle_min + i * angle_increment;
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
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Always add first scan
  if (!odom_initialized_) {
    return;
  }
  if (odom_initialized_ && !log_prob_grid_initialized_) {
    odometry_pose_history_.push_back({prev_odom_loc_, prev_odom_angle_, 0.0});
    printf("Initializing log prob grid for the first time\n");
    vector<Vector2f> point_cloud;
    ConstructPointCloud(msg, point_cloud);
    ConstructLogProbGrid(point_cloud);
    log_prob_grid_initialized_ = true;
    Vector2f estimated_loc = Vector2f(pose_history_.back().x(), pose_history_.back().y());
    float estimated_angle = pose_history_.back().theta();
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

  // Get last pose_history and add some random displacement (x,y,theta) to get the groundtruth
  Pose2 old_pose = pose_history_.back() + Pose2(0.1, 0.1, 0.1);

  // Make up a random 3x3 covariance matrix filled with random values (diagonal matrix with positive entries)
  Matrix3f ran_covariance = Matrix3f::Random(3,3);

  // Generate noise from gaussian with mean zero and random covariance matrix above
  random_device rd{};
  mt19937 gen{rd()};

  Vector3f mean = Vector3f::Zero();
  MultivariateNormal<float> mvn(mean, ran_covariance);
  Vector3f noise = mvn.Sample(gen);

  // Add noise to the ground truth to get noisy initial estimate
  Pose2 noisy_pose = old_pose + Pose2(noise(0), noise(1), noise(2));
  

  vector<Vector2f> point_cloud;
  ConstructPointCloud(msg, point_cloud);
  // printf("Pose Index: %d\n", pose_index_);

  // For loop to iterate over the last 5 poses
  assert(odometry_pose_history_.size() <= 5);
  for (int i = 0; i < (int) odometry_pose_history_.size(); i++) {
    // Get old pose from pose_history_
    int pose_history_index = pose_index_ - 1 - odometry_pose_history_.size() + i;
    printf("Pose History Index: %d\n", pose_history_index);
    Pose2 old_pose = pose_history_[pose_history_index];
    Vector2f old_pose_loc = Vector2f(old_pose.x(), old_pose.y());
    float old_pose_angle = old_pose.theta();
    
    // Get corresponding odometry pose from odometry_pose_history_
    Vector2f old_odom_loc = odometry_pose_history_[i].loc;
    float old_odom_angle = odometry_pose_history_[i].angle;

    // Print statement to check if the odometry pose history are being correctly read
    // printf("Odometry Pose: (%f, %f, %f)\n", old_odom_loc.x(), old_odom_loc.y(), old_odom_angle);

    // Get list of candidate poses with likelihood
    std::vector<Pose> candidate_poses(x_freq_ * y_freq_ * theta_freq_);
    PredictMotionModel(old_odom_loc, old_odom_angle, old_pose_loc, old_pose_angle, candidate_poses);

    // Get old log prob grid from history
    vector<vector<float>> old_log_prob_grid = log_prob_grid_history_[i];

    // Run CSM to align the point cloud to the last saved pose
    Eigen::Matrix3f covariance;
    Pose2 new_pose;
    RunCSM(point_cloud, old_pose_loc, old_pose_angle, old_log_prob_grid, candidate_poses, covariance, new_pose);

    // Add factor to pose graph
    // Solve for the joint solution over pose graph
    // Add odometry factors
    gtsam::noiseModel::Gaussian::shared_ptr noise_model = gtsam::noiseModel::Gaussian::Covariance(covariance.cast<double>());
    //   // (gtsam::Matrix3(3, 3) << sigma_xi(0,0), sigma_xi(0,1), sigma_xi(0,2),
    //   //                          sigma_xi(1,0), sigma_xi(1,1), sigma_xi(1,2),
    //   //                          sigma_xi(2,0), sigma_xi(2,1), sigma_xi(2,2))
    // );

    printf("Adding factor between %d and %d\n", pose_history_index + 1, pose_index_);

    // Then replace 'new_pose' with 'noisy_pose' in graph_.add(BetweenFactor<Pose2>(pose_history_index + 1, pose_index_, new_pose.between(old_pose), noise_model)); // new_pose.between(old_pose)
    graph_.add(BetweenFactor<Pose2>(pose_history_index + 1, pose_index_, noisy_pose.between(old_pose), noise_model)); // new_pose.between(old_pose)

    // If from closest previous pose, add to initial estimate
    if (i == (int) odometry_pose_history_.size() - 1) {
      initial_estimate_.insert(pose_index_, new_pose);
    }
  }
  // Save previous odom poses
  odometry_pose_history_.push_back({prev_odom_loc_, prev_odom_angle_, 0.0});
  // Save log prob grid
  ConstructLogProbGrid(point_cloud);
  // Keep only last 5 entries of odometry_pose_history_
  if (odometry_pose_history_.size() > 5) {
    odometry_pose_history_.erase(odometry_pose_history_.begin());
  }
  // Keep only last 5 entries of log_prob_grid_history_
  if (log_prob_grid_history_.size() > 5) {
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
  result_.print("Final Result:\n");

  // Check if optimization 'result' is equal to the ground truth pose
  // Print old_pose
  printf("Old Pose: (%f, %f, %f)\n", old_pose.x(), old_pose.y(), old_pose.theta());
  


  // if (pose_index_ == 5) exit(0);
  // Estimate pairwise non-succesive poses
  
  // Print estimated location and angle
  // printf("Estimated Pose: (%f, %f, %f)\n", estimated_loc_.x(), estimated_loc_.y(), estimated_angle_);

  // Save point cloud history
  // point_cloud_history_.push_back(point_cloud);

  // Transform the point cloud to the estimated pose and add to map
  Vector2f estimated_loc = Vector2f(result_.at<Pose2>(pose_index_).x(), result_.at<Pose2>(pose_index_).y());
  float estimated_angle = result_.at<Pose2>(pose_index_).theta();
  for (size_t i = 0; i < point_cloud.size(); i++) {
    map_.push_back(Rotation2Df(estimated_angle) * point_cloud[i] + estimated_loc);
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
  //         float x_dist = x_offset * log_prob_grid_resolution_;
  //         float y_dist = y_offset * log_prob_grid_resolution_;
  //         float dist = sqrt(x_dist * x_dist + y_dist * y_dist);
  //         float log_prob = -dist * dist / (2 * 0.01 * 0.01);
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
  const float old_odom_angle, 
  const Vector2f& old_pose_loc, 
  const float old_pose_angle, 
  vector<Pose>& candidate_poses
) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  Vector2f odom_loc_diff = prev_odom_loc_ - old_odom_loc;
  float loc_dist = odom_loc_diff.norm();

  float odom_angle_diff = AngleDiff(prev_odom_angle_, old_odom_angle);
  float angle_dist = AngleDist(prev_odom_angle_, old_odom_angle);

  Vector2f projected_loc = old_pose_loc + Rotation2Df(old_pose_angle - old_odom_angle) * odom_loc_diff;
  float projected_angle = AngleMod(old_pose_angle + odom_angle_diff);

  float angle_stddev = K1_ * angle_dist + K2_ * loc_dist;
  float loc_stddev = K3_ * angle_dist + K4_ * loc_dist;

  // Cache sin and cos for efficieny
  float cos_pose_angle = cos(projected_angle);
  float sin_pose_angle = sin(projected_angle);

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

				float x_pose = projected_loc.x() + del_x * cos_pose_angle - del_y * sin_pose_angle;
				float y_pose = projected_loc.y() + del_x * sin_pose_angle + del_y * cos_pose_angle;
				float theta_pose = projected_angle + del_theta;

				// Calculate the motion model likelihood
				// float log_likelihood = -(x_noise*x_noise)/(loc_stddev*loc_stddev) 
				// 				   -(y_noise*y_noise)/(loc_stddev*loc_stddev) 
				// 				   -(t_noise*t_noise)/(t_stddev*t_stddev);

        float log_likelihood = -1.0 * (x_r_norm * x_r_norm + y_r_norm * y_r_norm + theta_r_norm * theta_r_norm) / 2.0 ;

        int idx = x_r * y_freq_ * theta_freq_ + y_r * theta_freq_ + theta_r;
        // printf("idx: %d\n", idx);
				candidate_poses[idx] = {{x_pose, y_pose}, theta_pose, log_likelihood};
        // printf("Candidate Pose: (%f, %f, %f), with log likelihood %f\n", 
        //   candidate_poses[idx].loc.x(), candidate_poses[idx].loc.y(), candidate_poses[idx].angle, candidate_poses[idx].log_likelihood);
        
      }
		}
	}
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
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
  // float loc_diff = loc_delta.norm();
  float loc_diff = odom_loc_diff.norm();
  // float angle_delta = odom_angle - prev_odom_angle_;
  // float angle_diff = fabs(angle_delta - M_2PI * floor(angle_delta / M_2PI + 0.5));
  float angle_diff = AngleDist(odom_angle, prev_odom_angle_);

  // printf("Odom Diff: (%f, %f, %f)\n", odom_loc_diff.x(), odom_loc_diff.y(), angle_diff);
  // float dist = odom_diff.norm();

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
  int step = ceil((float) map_.size() / 5000);
  vector<Vector2f> subsampled_map;
  for (size_t i = 0; i < map_.size(); i += step) {
    subsampled_map.push_back(map_[i]);
  }
  return subsampled_map;
}

}  // namespace slam
