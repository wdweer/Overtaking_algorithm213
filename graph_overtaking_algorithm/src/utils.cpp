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
#include <hmcl_msgs/BehaviorFactor.h>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "utils.h"



double distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}



std::pair<double, double> cartesian_to_frenet(const std::vector<std::vector<double>>& centerline, const std::vector<double>& point) {
    Eigen::MatrixXd centerline_mat(centerline.size(), 2);
    for (size_t i = 0; i < centerline.size(); ++i) {
        centerline_mat(i, 0) = centerline[i][0];
        centerline_mat(i, 1) = centerline[i][1];
    }

    Eigen::MatrixXd diffs = centerline_mat.bottomRows(centerline.size() - 1) - centerline_mat.topRows(centerline.size() - 1);
    Eigen::VectorXd dists = diffs.rowwise().norm();
    Eigen::VectorXd arclength(dists.size() + 1);
    arclength(0) = 0.0;
    for (int i = 0; i < dists.size(); ++i) {
        arclength(i + 1) = arclength(i) + dists(i);
    }

    Eigen::Vector2d point_vec(point[0], point[1]);
    double min_dist = std::numeric_limits<double>::infinity();
    double s = 0, l = 0;

    for (int i = 0; i < diffs.rows(); ++i) {
        Eigen::Vector2d p1 = centerline_mat.row(i);
        Eigen::Vector2d p2 = centerline_mat.row(i + 1);

        Eigen::Vector2d line_vec = p2 - p1;
        Eigen::Vector2d point_to_p1 = point_vec - p1;
        double line_len = line_vec.norm();
        double proj_length = point_to_p1.dot(line_vec) / line_len;
        Eigen::Vector2d proj_point = p1 + (proj_length / line_len) * line_vec;

        double dist = (point_vec - proj_point).norm();

        // 벡터 외적을 사용하여 방향 판단
        Eigen::Vector2d perp_vec(-line_vec.y(), line_vec.x());  // line_vec에 수직인 벡터
        double side = point_to_p1.dot(perp_vec);  // 점이 선분의 왼쪽에 있는지 오른쪽에 있는지를 결정

        if (dist < min_dist) {
            min_dist = dist;
            s = arclength(i) + proj_length;
            l = (side < 0) ? -dist : dist;  // 왼쪽이면 음수, 오른쪽이면 양수
        }
    }

    return std::make_pair(s, l);
}


std::pair<double, double> frenet_to_cartesian(const std::vector<std::vector<double>>& centerline, double s, double l) {
    Eigen::MatrixXd centerline_mat(centerline.size(), 2);
    for (size_t i = 0; i < centerline.size(); ++i) {
        centerline_mat(i, 0) = centerline[i][0];
        centerline_mat(i, 1) = centerline[i][1];
    }

    Eigen::MatrixXd diffs = centerline_mat.bottomRows(centerline.size() - 1) - centerline_mat.topRows(centerline.size() - 1);
    Eigen::VectorXd dists = diffs.rowwise().norm();
    Eigen::VectorXd arclength(dists.size() + 1);
    arclength(0) = 0.0;
    for (int i = 0; i < dists.size(); ++i) {
        arclength(i + 1) = arclength(i) + dists(i);
    }

    int segment_index = std::lower_bound(arclength.data(), arclength.data() + arclength.size(), s) - arclength.data() - 1;
    if (segment_index < 0) {
        segment_index = 0;
    } else if (segment_index >= centerline.size() - 1) {
        segment_index = centerline.size() - 2;
    }

    Eigen::Vector2d p1 = centerline_mat.row(segment_index);
    Eigen::Vector2d p2 = centerline_mat.row(segment_index + 1);

    Eigen::Vector2d segment_vector = p2 - p1;
    double segment_length = dists(segment_index);
    Eigen::Vector2d segment_unit_vector = segment_vector / segment_length;

    Eigen::Vector2d base_point = p1 + segment_unit_vector * (s - arclength(segment_index));

    Eigen::Vector2d normal_vector(-segment_unit_vector(1), segment_unit_vector(0));

    Eigen::Vector2d cartesian_point = base_point + normal_vector * l;

    return std::make_pair(cartesian_point(0), cartesian_point(1));
}


visualization_msgs::MarkerArray visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "local_path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = waypoints[i].pose.pose.position;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}



