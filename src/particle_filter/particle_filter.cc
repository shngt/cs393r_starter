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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

int counter = 0;

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges/10);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); i++) {
    scan[i] = Vector2f(INFINITY, INFINITY);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  // for (size_t i = 0; i < map_.lines.size(); ++i) {
  //   const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    // line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // // Access the end points using `.p0` and `.p1` members:
    // printf("P0: %f, %f P1: %f,%f\n", 
    //        my_line.p0.x(),
    //        my_line.p0.y(),
    //        my_line.p1.x(),
    //        my_line.p1.y());

    // // Check for intersections:
    // bool intersects = map_line.Intersects(my_line);
    // // You can also simultaneously check for intersection, and return the point
    // // of intersection:
    // Vector2f intersection_point; // Return variable
    // intersects = map_line.Intersection(my_line, &intersection_point);
  Eigen::Matrix3f world_T_local = Eigen::Matrix3f::Identity();
  world_T_local.block<2, 1>(0, 2) = loc;
  world_T_local.block<2, 2>(0, 0) << cos(angle), -sin(angle), sin(angle), cos(angle);

  for (size_t i = 0; i < scan.size(); i++) {
    float ray_angle = angle_min + i*10*(angle_max - angle_min)/num_ranges;
    Eigen::Vector2f base_link_end_point = Eigen::Vector2f(range_max*cos(ray_angle) + laser_offset, range_max*sin(ray_angle));
    Eigen::Vector2f end_point = world_T_local.block<2, 2>(0, 0)*base_link_end_point + world_T_local.block<2, 1>(0, 2);
    line2f ray(loc, end_point);
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f& map_line = map_.lines[j];
      Eigen::Vector2f intersection_point;
      if (ray.Intersection(map_line, &intersection_point)) {
        if (scan[i] == Vector2f(INFINITY, INFINITY) || (intersection_point - loc).norm() < (scan[i] - loc).norm()) {
            scan[i] = intersection_point;
        }
      }
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  vector<Vector2f> predicted_scan;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, angle_min, angle_max, &predicted_scan);
  float weight = 0;
  for (size_t i = 0; i < ranges.size(); i+=10) {
    if (ranges[i] <= range_min || ranges[i] >= range_max || predicted_scan[i/10] == Vector2f(INFINITY, INFINITY)){
      continue;
    }
    else if (ranges[i] < (p_ptr->loc - predicted_scan[i/10]).norm() - params_.d_short){
      weight += pow(params_.d_short, 2);
    }
    else if (ranges[i] > (p_ptr->loc - predicted_scan[i/10]).norm() + params_.d_long){
      weight += pow(params_.d_long, 2);
    }
    else {
      weight += pow(ranges[i] - (p_ptr->loc - predicted_scan[i/10]).norm(), 2);
    }
  }
  float x = -0.5*weight / (params_.observation_model_stddev*params_.observation_model_stddev);
  p_ptr->weight = params_.observation_model_gamma * x;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // we find the maximum of the log likelihoods,
  // then apply a coefficient to give the maximum likelihood a weight of 1
  // to get rid of the disappearing weights problem
  double max_weight_ = -INFINITY;
  for (const Particle& p : particles_)
    max_weight_ = std::max(max_weight_, p.weight);

  vector<double> buckets(particles_.size());
  double total_weight = 0;
  for (size_t i = 0; i < particles_.size(); ++i) {
    float weight = exp(particles_[i].weight - max_weight_);
    buckets[i] = total_weight + weight;
    total_weight += weight;
  }
  vector<Particle> new_particles(particles_.size());
  for (size_t j = 0; j < particles_.size(); ++j) {
    double r = rng_.UniformRandom(0, total_weight);
    // Find which bucket r falls into
    int bucket_idx = std::upper_bound(buckets.begin(), buckets.end(), r) - buckets.begin();
    particles_[bucket_idx].weight = 1.0;
    new_particles[j] = particles_[bucket_idx];

  }
  particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if (!odom_initialized_) {
    return;
  }
  for (Particle& p : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &p);
  }
  counter +=1;
  if(counter == 10){
    Resample();
    counter = 0;
  }
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }
  
  Vector2f delta(odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleDiff(odom_angle, prev_odom_angle_);
  float sigma = params_.k1*delta.norm() + params_.k2*delta_angle;

  for (Particle& p : particles_) {
    Vector2f epsilon(
        rng_.Gaussian(0, sigma),
        rng_.Gaussian(0, sigma));
    // transform the delta to the world frame
    Vector2f delta_local = Vector2f(delta.x()*cos(p.angle - prev_odom_angle_) - delta.y()*sin(p.angle - prev_odom_angle_), delta.x()*sin(p.angle - prev_odom_angle_) + delta.y()*cos(p.angle - prev_odom_angle_));
    p.loc += delta_local + epsilon;
    p.angle += delta_angle + rng_.Gaussian(0, params_.k3*delta.norm() + params_.k4*delta_angle);
    // ensure angle is between 0 and 2pi
    p.angle = math_util::AngleMod(p.angle);
  }
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  particles_.resize(params_.num_particles);
  odom_initialized_ = false;

  for (Particle& p : particles_) {
    p.loc = loc;
    p.angle = angle;
    p.weight = 1.0/params_.num_particles;
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  if (!odom_initialized_) {
    return;
  }
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
  float cos_angle = 0;
  float sin_angle = 0;
  
  for (const Particle& p : particles_) {
    loc += p.loc;
    cos_angle += cos(p.angle);
    sin_angle += sin(p.angle);
    
  }
  angle = atan2(sin_angle, cos_angle);
  loc /= particles_.size();

}


} 
