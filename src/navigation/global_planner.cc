#include "global_planner.h"

// Hash function for Vector2f
namespace std {
    template<> struct hash<Eigen::Vector2f> {
        std::size_t operator()(const Eigen::Vector2f& vec) const {
            // Hash combining technique based on boost's hash_combine
            std::size_t seed = 0;
            auto hasher = std::hash<float>();
            seed ^= hasher(vec[0]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
            seed ^= hasher(vec[1]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
            return seed;
        }
    };
}

// Custom comparator for pair<float, Vector2f>
// Custom comparator for std::pair<float, Eigen::Vector2f>
struct PairFloatVector2fComparator {
    bool operator()(const std::pair<float, Eigen::Vector2f>& lhs, const std::pair<float, Eigen::Vector2f>& rhs) const {
        // First compare the float values
        if (lhs.first > rhs.first) return true;
        if (lhs.first < rhs.first) return false;

        // If the float values are equal, compare the Eigen::Vector2f values
        // Compare the x-component first
        if (lhs.second[0] > rhs.second[0]) return true;
        if (lhs.second[0] < rhs.second[0]) return false;

        // If the x-components are equal, compare the y-component
        return lhs.second[1] > rhs.second[1];
    }
};

void PlanPath(const Vector2f& start, const Vector2f& goal, VectorMap& map_, vector<Vector2f>& path) {
    std::cout << "Planning path from " << start[0] << ", " << start[1] << " to " << goal[0] << ", " << goal[1] << std::endl;
    // Parent map
    unordered_map<Vector2f, Vector2f> parent;
    // Cost map
    unordered_map<Vector2f, float> cost;

    // // Add a dummy value to parent
    // parent[goal] = start;
    // // Check if goal is in map
    // if (parent.find(goal) == parent.end()) {
    //     std::cout << "Goal not in map" << std::endl;
    //     return;
    // }

    // Frontier vertices
    priority_queue<pair<float, Vector2f>, vector<pair<float, Vector2f>>, PairFloatVector2fComparator> frontier;
    
    // Round start and goal to nearest 0.25 meters
    Vector2f rounded_start, rounded_goal;
    rounded_start = {round(start[0] * 4) / 4, round(start[1] * 4) / 4};
    rounded_goal = {round(goal[0] * 4) / 4, round(goal[1] * 4) / 4};

    // Parent of start
    parent[rounded_start] = start;
    // Cost of start
    cost[rounded_start] = 0;

    // A* path planning from start to goal
    // frontier.push({(rounded_goal - rounded_start).norm(), rounded_start});
    // int T = 2;
    frontier.push({0, rounded_start});
    while (!frontier.empty()) {
        Vector2f current = frontier.top().second;
        frontier.pop();
        // if (T-- < 0) {
        //     break;
        // }
        // Print current
        // std::cout << "Current: " << current[0] << ", " << current[1] << std::endl;
        if ((current - rounded_goal).norm() < 1e-6) {
            path.push_back(rounded_goal);
            break;
        }
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                Vector2f neighbor = current + Vector2f(i * 0.25, j * 0.25);
                // Print neighbor
                // std::cout << "Neighbor: " << neighbor[0] << ", " << neighbor[1] << std::endl;
                // Check if line from current to neighbor intersects with map
                line2f ray(current, neighbor);
                bool intersects = false;
                for (size_t j = 0; j < map_.lines.size(); ++j) {
                    const line2f& map_line = map_.lines[j];
                    Eigen::Vector2f intersection_point;
                    if (ray.Intersection(map_line, &intersection_point)) {
                        // std::cout << "Intersection at " << intersection_point[0] << ", " << intersection_point[1] << std::endl;
                        intersects = true;
                    }
                }
                if (intersects) {
                    continue;
                }
                // Get new cost
                float new_cost = cost[current] + (neighbor - current).norm();
                // Print old and new cost
                // std::cout << "Old cost: " << cost[current] << std::endl;
                // std::cout << "New cost: " << new_cost << std::endl;
                if (cost.find(neighbor) == cost.end() || new_cost < cost[neighbor]) {
                    cost[neighbor] = new_cost;
                    parent[neighbor] = current;
                    frontier.push({new_cost + (neighbor - rounded_goal).norm(), neighbor});
                }
            }
        }
    }

    // Reconstruct path
    Vector2f current = rounded_goal;
    while (current != rounded_start) {
        path.push_back(current);
        current = parent[current];
    }

    std::cout << "Path planned" << std::endl;
}