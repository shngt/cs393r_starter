#include "navigation.h"
#include <iostream>
#include <unordered_map>
#include <queue>
#include <vector>
#include "shared/math/line2d.h"

using std::vector;
using std::unordered_map;
using std::priority_queue;
using std::pair;
using Eigen::Vector2f;
using vector_map::VectorMap;
using geometry::line2f;

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

void PlanPath(const Vector2f& start, const Vector2f& goal, VectorMap& map_, vector<Vector2f>& path);

#endif  // GLOBAL_PLANNER_H