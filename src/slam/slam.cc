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

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    x_freq_(11.0),
    y_freq_(11.0),
    theta_freq_(31.0),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}

void SLAM::PredictMotionModel(const Vector2f& odom_loc, const float odom_angle, float current_pose_angle){
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);   
  // The very first callback goes here
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
  }

  Vector2f loc_delta =
    Rotation2Df(-prev_odom_angle_) * (odom_loc - prev_odom_loc_);
  float angle_delta = odom_angle - prev_odom_angle_;

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;

  float angle_diff = fabs(angle_delta - M_2PI * floor(angle_delta / M_2PI + 0.5));
  float loc_diff = loc_delta.norm();
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

				float x_pose = odom_loc.x() + del_x * cos_pose_angle - del_y * sin_pose_angle;
				float y_pose = odom_loc.y() + del_x * sin_pose_angle + del_y * cos_pose_angle;
				float theta_pose = current_pose_angle + del_theta;

				// Calculate the motion model likelihood
				// float log_likelihood = -(x_noise*x_noise)/(loc_stddev*loc_stddev) 
				// 				   -(y_noise*y_noise)/(loc_stddev*loc_stddev) 
				// 				   -(t_noise*t_noise)/(t_stddev*t_stddev);

        float log_likelihood = - (x_r * x_r) - (y_r * y_r) - (theta_r * theta_r);

        int idx = x_r * y_freq_ * theta_freq_ + y_r * theta_freq_ + theta_r;
				candidate_poses_[idx] = {{x_pose, y_pose}, theta_pose, log_likelihood};
			}
		}
	}
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  Vector2f odom_diff = odom_loc - prev_odom_loc_;
  float angle_diff = AngleDiff(odom_angle, prev_odom_angle_);
  float dist = odom_diff.norm();

  Vector2f projected_loc = estimated_loc_ + Rotation2Df(estimated_angle_) * odom_diff;
  float projected_angle = AngleMod(estimated_angle_ + angle_diff);

  if (dist > loc_threshold_ || fabs(angle_diff) > angle_threshold_) {
    // Update the candidate poses based on the odometry
    PredictMotionModel(odom_loc, odom_angle, projected_angle);
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
