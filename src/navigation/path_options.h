#include "navigation.h"
// #inlcude <vector>
using std::vector;
using Eigen::Vector2f;


#ifndef PATH_OPTIONS_H
#define PATH_OPTIONS_H

float run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& nav_params);

void setPathOption(navigation::PathOption& path_option,
    float curvature,
    const std::vector<Eigen::Vector2f>& point_cloud,
    const navigation::NavigationParams& nav_params);

vector<navigation::PathOption> samplePathOptions(int num_options,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config);

int selectPath(const vector<navigation::PathOption>& path_options);

#endif  // PATH_OPTIONS_H