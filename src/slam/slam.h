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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose {
  Eigen::Vector2f loc;
  float angle;
  float log_likelihood;
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Run CSM on the given point cloud
  void RunCSM(const std::vector<Eigen::Vector2f>& point_cloud);

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  void PredictMotionModel(const Eigen::Vector2f& odom_loc, const float odom_angle, Eigen::Vector2f current_pose_loc, float current_pose_angle);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:
  float K1_;
  float K2_;
  float K3_;
  float K4_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Estimated robot pose
  Eigen::Vector2f estimated_loc_;
  float estimated_angle_;

  // Thresholds for adding new pose
  float loc_threshold_;
  float angle_threshold_;

  // Flag to indicate if a new scan should be applied
  bool apply_new_scan_;

  // Frequency of x, y, and theta sampling in the motion model calculations
  float x_freq_;
  float y_freq_;
  float theta_freq_;

  // All possible candidate poses from motion model
  std::vector<Pose> candidate_poses_;

  // Log probability grid for CSM
  std::vector<std::vector<float>> log_prob_grid_;
  float log_prob_grid_resolution_;
  Eigen::Vector2f log_prob_grid_origin_;
  bool log_prob_grid_initialized_;

  // Map
  std::vector<Eigen::Vector2f> map_;

  // For convariance calcular
  std::vector<Eigen::Matrix3f> covariances_;

  // Nonlinear factor graph
  gtsam::NonlinearFactorGraph graph_;

  // History of odometry-reported poses
  std::vector<Pose> odometry_pose_history_;

  // History of calculated poses
  std::vector<gtsam::Pose2> pose_history_;

  // History of point clouds
  std::vector<std::vector<Eigen::Vector2f>> point_cloud_history_;

  // Create initial estimate to the solution
  gtsam::Values initial_estimate_;

  // Optimization results
  gtsam::Values result_;

  // Pose index for optimization
  int pose_index_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
