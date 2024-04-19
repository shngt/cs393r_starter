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
    x_res(10.0),
    y_res(10.0),
    t_res(30.0),
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

  // Calculate sin(theta) and cos(theta) for trig computations
  const float cos_angle = cos(current_pose_angle);
  const float sin_angle = sin(current_pose_angle);

  // 3 for loops for each dimension of voxel cube (x, y, theta)
	for (int x_i=0; x_i<x_res_; x_i++)
	{
		float x_noise = x_stddev*(2*x_i/(x_res_-1)-1);
		// float x_noise = 0; //odom only
		for (int y_i=0; y_i<y_res_; y_i++)
		{
			float y_noise = y_stddev*(2*y_i/(y_res_-1)-1);
			// float y_noise = 0; //odom only
			for (int t_i=0; t_i<t_res_; t_i++)
			{
				float t_noise = t_stddev*(2*t_i/(t_res_-1)-1);
				// float t_noise = 0; //odom only
				float t_pose = current_pose_angle + t_noise;
				float x_pose = odom_loc.x() + x_noise*cos_angle - y_noise*sin_angle;
				float y_pose = odom_loc.y() + x_noise*sin_angle + y_noise*cos_angle;
				// Calculate the motion model likelihood
				float log_likelihood = -(x_noise*x_noise)/(x_stddev*x_stddev) 
								   -(y_noise*y_noise)/(y_stddev*y_stddev) 
								   -(t_noise*t_noise)/(t_stddev*t_stddev);
				
				Pose this_pose = {{x_pose, y_pose}, t_pose};
				possible_poses_.push_back({this_pose, log_likelihood});
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
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
