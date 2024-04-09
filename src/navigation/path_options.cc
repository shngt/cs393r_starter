#include "path_options.h"
#include <cstdio>
#include <iostream>
// #include <cmath>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

float run1DTimeOptimalControl(float dist_to_go, float current_speed, bool reverse, const navigation::NavigationParams& robot_config) {
    // Set current_speed to absolute value
    current_speed = abs(current_speed);
    float max_accel = robot_config.max_accel;
    float max_decel = robot_config.max_decel;
    float max_vel = robot_config.max_vel;
    float dt = robot_config.dt;
    float cruise_stopping_dist = pow(current_speed, 2) / (2 * max_decel);
    float accel_stopping_dist = pow(current_speed + dt * max_accel, 2) / (2 * max_decel);
    // std::cout << "Current Speed: " << current_speed << std::endl;
    // std::cout << "Max Velocity: " << max_vel << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "Cruise Stopping Distance: " << cruise_stopping_dist << std::endl;
    // std::cout << "Dist to go: " << dist_to_go << std::endl;

    // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
    float new_speed = 0;
    if (dist_to_go > accel_stopping_dist && current_speed < max_vel) {
        new_speed = std::min(max_vel, current_speed + max_accel * dt);
    }
    else if (dist_to_go > cruise_stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
                                                                        // probably needs hysteresis
        new_speed = current_speed;
    }
    else {  // otherwise needs to decelerate
        new_speed = std::max(current_speed - max_decel * dt, 0.0f);
    }

    if (reverse) 
        new_speed = -new_speed;
    
    return new_speed;
}



// set curvature, free path length, obstruction for a path option
void setPathOption(navigation::PathOption& path_option,
                        float curvature, bool reverse, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {
    path_option.curvature = curvature;
    path_option.reverse = reverse;
    // Hardcoded values in the reverse case to start
    if (reverse) {
        path_option.free_path_length = 0.8;
        path_option.obstruction = Eigen::Vector2f::Zero();
        // Iterate through point cloud and set distance of closest point as clearance
        path_option.clearance = 5.0;
        path_option.closest_point = Eigen::Vector2f::Zero();
        for (auto p: point_cloud) {
            if (p.norm() < path_option.clearance) {
                path_option.clearance = p.norm();
                path_option.closest_point = p;
            }
        }
        // path_option.clearance = 0;
        
        return;
    }
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    // distance from base link to back bumper
    float h_back = robot_config.base_link_offset;
    if (curvature == 0) {
        if (!reverse) {
            path_option.free_path_length = 5.0; // some large number
            for (auto p: point_cloud) {
                // Check for forward obstruction
                if (robot_config.width/2 + robot_config.safety_margin >= abs(p[1])
                    && p[0] < path_option.free_path_length) {
                    if (p[0] - h - robot_config.safety_margin < path_option.free_path_length) {
                        path_option.free_path_length = p[0] - h - robot_config.safety_margin;
                        // printf("Free Path Length: %f\n", path_option.free_path_length);
                        path_option.obstruction = p;
                        // printf("Obstruction: %f, %f\n", p[0], p[1]);
                    }
                }
            }
            path_option.clearance = path_option.free_path_length;
            // Check for forward clearance
            for (auto p: point_cloud) {
                if (p[0] >= 0 && p[0] < path_option.free_path_length) {
                    float clearance_p = abs(p[1]) - robot_config.width / 2 - robot_config.safety_margin;
                    if (clearance_p < path_option.clearance) {
                        path_option.clearance = clearance_p;
                        path_option.closest_point = p;
                    }
                }
            }
        }
        else {
            cout << "reverse" << endl;
            path_option.free_path_length = 5.0; // some large number
            for (auto p: point_cloud) {
                // Check for backward obstruction
                if (robot_config.width/2 + robot_config.safety_margin >= abs(p[1])
                    && p[0] < 0 && abs(p[0]) < path_option.free_path_length) {
                    path_option.free_path_length = std::min(path_option.free_path_length, (p[0]) - h_back - robot_config.safety_margin);
                    path_option.obstruction = p;
                }
            }
            // cout << "free path length " << path_option.free_path_length << endl;
            for (auto p: point_cloud) {
                // Check for backward clearance
                if (p[0] < 0 && abs(p[0]) < path_option.free_path_length) {
                    float clearance_p = abs(p[1]) - robot_config.width / 2 - robot_config.safety_margin;
                    if (clearance_p < path_option.clearance) {
                        path_option.clearance = clearance_p;
                        path_option.closest_point = p;
                    }
                }
            }
            // cout << "clearance " << path_option.clearance << endl;
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - robot_config.width / 2 - robot_config.safety_margin;
    float r_outer = c.norm() + robot_config.width / 2 + robot_config.safety_margin;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_bl = (Vector2f(0, r_inner) - Vector2f(robot_config.base_link_offset + robot_config.safety_margin, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset + robot_config.safety_margin, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c.norm(), 5.0);  // some large number
    // float omega = atan2(h, r_inner);

    float theta_br = asin(robot_config.base_link_offset + robot_config.safety_margin / r_br); // angle where back right would hit when going forward
    // angle where front right would hit
    // float theta_fr = asin(h + robot_config.safety_margin / r_tr);
    float phi = 0;
//	cout << "curvature " << curvature << endl;
//	bool front_side = false, outer_side = false, inner_side = false;
    // For forward path
    if (!reverse) {
        for (unsigned int i = 0; i < point_cloud.size(); i++) {
            Vector2f p = point_cloud[i];
            float r_p = (c-p).norm();
            float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
            float length = 5.0;
            // cout << "curvature " << curvature << endl;
            if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
                    phi = acos(r_inner / r_p);
                    length = (theta - phi) * c.norm();
                // inner_side = true;
                    // cout << "inner side hit" << endl;
            }
            if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
                phi = acos(r_p / (c.norm() + robot_config.width / 2));
                length = (theta - phi) * c.norm();
            // outer_side = true;
            // cout << "outer side hit" << endl;
            }

            if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
                phi = asin(h / r_p);
                length = (theta - phi) * c.norm();
            // front_side = true;
            // cout << "front side hit" << endl;
            }
            if (length < path_option.free_path_length && length > 0) {
                path_option.free_path_length = length;
                path_option.obstruction = p;
            }
        }
    }
    // For backward path
    else {
        for (unsigned int i = 0; i < point_cloud.size(); i++) {
            Vector2f p = point_cloud[i];
            float r_p = (c-p).norm();
            float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
            float length = 5.0;
            // cout << "curvature " << curvature << endl;
            if (r_inner <= r_p && r_p <= r_bl) {    // inner side hit
                phi = acos(r_inner / r_p);
                length = std::min(length, (abs(theta) - abs(phi)) * c.norm());
                // inner_side = true;
                    // cout << "inner side hit" << endl;
            }
            // if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            //     phi = acos(r_p / (c.norm() + robot_config.width / 2));
            //     length = (theta - phi) * c.norm();
            // // outer_side = true;
            // // cout << "outer side hit" << endl;
            // }

            if (r_bl <= r_p && r_p <= r_br) {    // front side hit
                phi = asin(h_back / r_p);
                length = std::min(length, (abs(theta) - abs(phi)) * c.norm());
                // front_side = true;
                // cout << "front side hit" << endl;
            }
            // Outer side hit
            if ((r_br <= r_p && r_p <= r_tr)) {
                phi = acos(r_p / (c.norm() + robot_config.width / 2));
                length = std::min(length, (abs(theta) - abs(phi)) * c.norm());
            }
            if (length < path_option.free_path_length && length > 0) {
                path_option.free_path_length = length;
                path_option.obstruction = p;
            }
        }
    }

	// if (inner_side)
	//  	cout << "intersecting particle found with inner side" << endl;
	// if (outer_side)
	//	cout << "intersecting particle found with outer side" << endl;
	//if (front_side)
	//	cout << "intersecting particle found with front side" << endl;

    // float theta = M_PI / 2;
    // if (path_option.obstruction != Eigen::Vector2f::Zero()) {
    //     theta = curvature < 0 ? atan2(path_option.obstruction[0], path_option.obstruction[1]- c[1]) :
    //         atan2(path_option.obstruction[0], c[1] - path_option.obstruction[1]);
    // }
    // clearance
    // path_option.clearance = 100; // some large number
    if (!reverse) {
        for (auto p: point_cloud) {
            float theta_p =  curvature < 0 ? atan2(p[0], p[1]- c[1]) :
                atan2(p[0], c[1] - p[1]);
            float path_len_p = theta_p * (p-c).norm();
            if (path_len_p >=0 and path_len_p < path_option.free_path_length) {  // if p is within the fp length
                float inner = abs((c - p).norm() - r_inner);
                float outer = abs((c - p).norm() - r_tr);
                float clearance_p = std::min(inner, outer);
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    path_option.closest_point = p;
                }
            }
        }
    }
    else {
        path_option.clearance = 0; // zero for now
        path_option.closest_point = Eigen::Vector2f::Zero();
    }
}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> samplePathOptions(int num_options,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config) {
    static vector<navigation::PathOption> path_options(2 * num_options);
    // path_options.clear();
    float max_curvature = robot_config.max_curvature;

    // loop through curvature from max to -max
    // forward options
    bool reverse = false;
    for (int i = 0; i < num_options; i++) { 
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        
        navigation::PathOption path_option;
        setPathOption(path_option, curvature, reverse, point_cloud, robot_config);
        path_options[i] = path_option;
    }
    // backward options
    reverse = true;
    // Allow less curvature on the back for tie breaking
    max_curvature = max_curvature / 2;
    for (int i = 0; i < num_options; i++) {
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        navigation::PathOption path_option;
        setPathOption(path_option, curvature, reverse, point_cloud, robot_config);
        path_options[num_options + i] = path_option;
    }
    // loop 
    // exit(0);
    return path_options;
}

float score(float free_path_length, float curvature, float clearance) {
    const float w1 = 0.7;
    const float w2 = 0;
    const float w3 = 0.3;
    return w1 * free_path_length + w2 * abs(1/curvature) + w3 * clearance;
}

// float carrot_path_distance(const Vector2f& carrot_point, const Vector2f& c, float& free_path_length, bool reverse) {
//     float theta_c = atan2(-c[0], -c[1]);
//     float theta_carrot = atan2(carrot_point[0] - c[0], carrot_point[1] - c[1]); 
//     float theta = free_path_length / c.norm();

// }

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
// if there are multiple paths with the same free path length, return the one with the smallest curvature
int selectPath(const vector<navigation::PathOption>& path_options, const Vector2f& carrot_point) {
    int selected_path = 0;
    // printf("--------------\n");
    // printf("Carrot Point: %f, %f\n", carrot_point[0], carrot_point[1]);
    float best_score = INFINITY;
    // float distance_from_carrot = geometry::
    for (unsigned int i = 0; i < path_options.size(); i++) {
	if (path_options[i].free_path_length == 0.0)
		continue;
        float path_curvature = path_options[i].curvature;
        float r = 1 / (path_curvature + 1e-6);
        Eigen::Vector2f c = Vector2f(0, r);
        // printf("Curvature point: %f, %f\n", c[0], c[1]);
        float carrot_r = (c - carrot_point).norm();
        // float carrot_path_distance = abs(carrot_r - abs(1 / (path_curvature + 1e-6)));
        float a = fabs(path_options[i].free_path_length * path_curvature); 
        // Vector2f end_point = c + Vector2f(sin(-a), cos(-a)) * 1 / (path_curvature + 1e-6);
        // printf("End Point: %f, %f\n", end_point[0], end_point[1])
        ;
        float a0, a1;
        if (!path_options[i].reverse) {
            a0 = (path_curvature > 0.0f) ? -M_PI_2 : (M_PI_2 - a);
        } else {
            a0 = (path_curvature > 0.0f) ? (-M_PI_2 - a) : (M_PI_2);
        }
        if (!path_options[i].reverse) {
            a1 = (path_curvature > 0.0f) ? (-M_PI_2 + a) : (M_PI_2);
        } else {
            a1 = (path_curvature > 0.0f) ? -M_PI_2 : (M_PI_2 + a);
        }
        // printf("a0: %f, a1: %f\n", a0, a1);
        float carrot_path_distance = 0.0f;
        Vector2f end_point;
        if (!path_options[i].reverse) {
            if (r > 0)
                end_point = c + Eigen::Rotation2Df(a1 - a0) * Vector2f(0, -r);
            else
                end_point = c + Eigen::Rotation2Df(a0 - a1) * Vector2f(0, -r);
        } else {
            if (r > 0)
                end_point = c + Eigen::Rotation2Df(a0 - a1) * Vector2f(0, -r);
            else
                end_point = c + Eigen::Rotation2Df(a1 - a0) * Vector2f(0, -r);
        }

        // int rot_sign = 1;
        // float radius = abs(1 / (path_curvature + 1e-6));
        // Vector2f carrot_point_dummy = carrot_point + Vector2f(0, 1e-6);
        // float carrot_path_distance = geometry::MinDistanceLineArc(carrot_point, 
        //     carrot_point_dummy, 
        //     c, 
        //     radius, a0, a1, rot_sign);
        float carrot_point_theta = atan2(carrot_point[1] - c[1], carrot_point[0] - c[0]);
        // carrot point outside the arc
        if ((a0 < a1 && (carrot_point_theta < a0 || carrot_point_theta > a1)) || (a0 > a1 && (carrot_point_theta > a0 || carrot_point_theta < a1))) {
            carrot_path_distance = std::min((carrot_point).norm(), (carrot_point - end_point).norm());
        // carrot point inside the arc
        } else {
            carrot_path_distance = abs(carrot_r - abs(r));
        }
        // printf("Carrot Point wrt c: %f, %f\n", carrot_point[0] - c[0], carrot_point[1] - c[1]);
        // printf("Carrot Point: %f, %f\n", carrot_point[0], carrot_point[1]);
        // printf("Carrot Point Theta: %f\n", carrot_point_theta);
        // printf("Carrot Path Distance: %f for curvature: %f with reverse: %d\n", carrot_path_distance, path_curvature, path_options[i].reverse);
        float s = -score(path_options[i].free_path_length, path_curvature, path_options[i].clearance) + 5.0 * carrot_path_distance;
        if (s < best_score) {
            best_score = s;
            selected_path = i;
            // printf("Best Score so far: %f at curvature: %f\n", best_score, path_curvature);
        }
    }
    return selected_path;
}

