#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hmcl_msgs/LaneArray.h>
#include <hmcl_msgs/Lane.h>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <algorithm>
#include <Eigen/Dense>

// Function prototypes
double distance(double x1, double y1, double x2, double y2);
int calculate_distance_pose2local();
int calculate_distance_pose2local_1();
std::pair<double, double> cartesian_to_frenet(const std::vector<std::vector<double>>& centerline, const std::vector<double>& point);
std::pair<double, double> frenet_to_cartesian(const std::vector<std::vector<double>>& centerline, double s, double l);
visualization_msgs::MarkerArray visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints);

#endif // UTILS_H