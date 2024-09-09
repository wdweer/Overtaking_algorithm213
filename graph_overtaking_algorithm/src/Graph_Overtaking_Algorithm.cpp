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
#include <chrono>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <queue>
#include "utils.h"
struct VectorComparator {
    bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
        if (a.first == b.first) {
            return a.second < b.second;
        }
        return a.first < b.first;
    }
};

class GraphPlanner {
public:
    GraphPlanner();
    std::vector<std::pair<int,int>> dijkstra(std::pair<int,int> start_node, std::pair<int,int> target, std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost);
    void Cost_Generating();
    void Edge_Destroying();
    void detected_object_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);
    void globalTrajCallback(const hmcl_msgs::LaneArray::ConstPtr& lane_msg);
    void globalTrajCallback1(const hmcl_msgs::Lane::ConstPtr& msg);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void run();
    void detected_object_kinematic(int obj,double t);
    void OG_PUB();
    visualization_msgs::MarkerArray visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints);
    void Trajectory_Generation();
    void compute_local_path();
    void compute_local_path1();
    int calculate_distance_pose2local();
    int calculate_distance_pose2local_1();
    void local_lane_to_local_points();
    void local_lane_to_global_points();
    void Graph_Generating();
    void globalTrajCallback2(const hmcl_msgs::LaneArray::ConstPtr& lane_msg);
    void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
private:
    std::vector<int> obj; 
    autoware_msgs::DetectedObjectArray objects_data;
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber global_traj_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber global_traj1_sub_;
    ros::Subscriber global_traj2_sub_;
    ros::Publisher overtaking_traj_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber current_velocity_sub_;
    std::vector<std::vector<double> > local_points, optimal_points,global_points;
    int model_predicted_num;
    double dt;
    std::vector<double> control;
    std::vector<double> state;
    double qx, qy, qz, qw;  
    double x, y;
    std::map<int, std::vector<double>> target_vel_x, target_vel_y; // Assuming use of id as key
    std::map<int, double> target_dimension_x,target_dimension_y,target_x, target_y, target_velocity_x, target_velocity_y, target_orientation_x, target_orientation_y, target_orientation_z, target_orientation_w, target_yaw_veh, target_angular_z_veh;
    std::vector<double> cx, cy, cqx, cqy, cqz, cqw, global_cx,global_cy;
    std::map<int, double> target_veh_dic_x, target_veh_dic_y;
    double current_v_x,current_v_y;
    double target_velocity;
    double target_angular_z;
    hmcl_msgs::LaneArray global_lane_array;
    hmcl_msgs::LaneArray global_lane_array1;
    hmcl_msgs::Lane local_lane;
    hmcl_msgs::Lane local_lane1;
    double pose_x, pose_y;
    bool global_traj_available = false;
    bool init_obs = false;
    int graph_vertical=3;
    int graph_horizon=6;
    std::map<std::pair<int, int>, std::pair<double, double>, VectorComparator> graph_position;
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>, VectorComparator> graph_edge;
    std::map<std::pair<int, int>, double, VectorComparator> node_cost;
    std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost;


};

GraphPlanner::GraphPlanner() : model_predicted_num(5), dt(0.1) {
    target_sub_ = nh_.subscribe("/tracking_side/objects", 1, &GraphPlanner::detected_object_callback, this);
    global_traj_sub_ = nh_.subscribe("/optimal_traj", 1, &GraphPlanner::globalTrajCallback, this);
    global_traj1_sub_ = nh_.subscribe("/local_traj", 1, &GraphPlanner::globalTrajCallback1, this);
    global_traj2_sub_ = nh_.subscribe("/global_traj", 1, &GraphPlanner::globalTrajCallback2, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &GraphPlanner::currentPoseCallback, this);
    current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &GraphPlanner::currentVelocityCallback, this);
    overtaking_traj_pub_ = nh_.advertise<hmcl_msgs::Lane>("/local_traj1", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/over_traj_viz", 1);
    // behavior_factor_sub = nh_.subscribe('/behavior_factor', 1, &GraphPlanner::behaviorCallback, this);
}

void GraphPlanner::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;
    x = msg->pose.position.x;
    pose_x=msg->pose.position.x;
    y = msg->pose.position.y;
    pose_y=msg->pose.position.y;
    // std::cout << pose_x << ", " << pose_y << std::endl;
}
// void GraphPlanner::behaviorCallback(const hmcl_msgs::BehaviorFactor& msg){
//     sEgo = msg->sEgo 
// }
void GraphPlanner::currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_v_x= msg->twist.linear.x;
    current_v_y= msg->twist.linear.y;
}

void GraphPlanner::detected_object_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg) {
    // std::cout << "Object_detected" << std::endl;
    // std::cout << "Number of objects detected: " << msg->objects.size() << std::endl; // 오브젝트 개수 출력

    target_veh_dic_x.clear();
    target_veh_dic_y.clear();
    obj.clear();
    objects_data = *msg;  
    for (const auto& obj : msg->objects) {
        int obj_key = obj.id; // Using ID as key

        target_velocity_x[obj_key] = obj.velocity.linear.x;
        target_velocity_y[obj_key] = obj.velocity.linear.y;
        target_x[obj_key] = obj.pose.position.x;
        target_y[obj_key] = obj.pose.position.y;
        target_dimension_x[obj_key]=obj.dimensions.x;
        target_dimension_y[obj_key]=obj.dimensions.y;
        target_orientation_x[obj_key] = obj.pose.orientation.x;
        target_orientation_y[obj_key] = obj.pose.orientation.y;
        target_orientation_z[obj_key] = obj.pose.orientation.z;
        target_orientation_w[obj_key] = obj.pose.orientation.w;
        target_yaw_veh[obj_key] = obj.velocity.angular.z;

        // detected_object_kinematic(obj_key); // Custom function to handle object kinematics based on label or other identifiers
        
        // Additional logic for velocity history and LSTM model might be included here.
    }
    if (global_traj_available){
    compute_local_path();
    auto start_time = std::chrono::high_resolution_clock::now();

    // 측정하고자 하는 함수 호출
    Graph_Generating();

    // 시간 측정 종료 (종료 시간 기록)
    auto end_time = std::chrono::high_resolution_clock::now();

    // 경과 시간 계산 (밀리초 단위로 측정)
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

    // 경과 시간 출력
    std::cout << "Graph_Generating() 함수 실행 시간: " << elapsed.count() << " ms" << std::endl; // Assuming another method handles potential field calculation
}
}
void GraphPlanner::detected_object_kinematic(int obj,double t) {
    double magnify=2;
    double model_prediction_x = target_x[obj];
    double target_vel_x = target_velocity_x[obj];
    double target_yaw = target_yaw_veh[obj];

    double model_prediction_y = target_y[obj];
    double target_vel_y = target_velocity_y[obj];
    double target_angular_z = target_angular_z_veh[obj];


    for (int i = 0; i < model_predicted_num; ++i) {
        model_prediction_x += target_vel_x*pow(0.1,magnify);
        target_yaw += target_angular_z*pow(0.1,magnify);
        target_vel_x = pow((pow(target_vel_x,2)+pow(target_vel_y,2)),1/2) * std::cos(target_yaw);
    }
    target_veh_dic_x[obj] = model_prediction_x;

    for (int j = 0; j < model_predicted_num; ++j) {
        model_prediction_y += target_vel_y*pow(0.1,magnify);
        target_yaw += target_angular_z*pow(0.1,magnify);  // Note: This recalculates target_yaw; ensure this is the desired behavior
        target_vel_y = pow((pow(target_vel_x,2)+pow(target_vel_y,2)),1/2) * std::sin(target_yaw);
    }
    target_veh_dic_y[obj] = model_prediction_y;
}

void GraphPlanner::globalTrajCallback1(const hmcl_msgs::Lane::ConstPtr& data) {
    

    // Clear the vectors before populating them
    cx.clear();
    cy.clear();
    cqx.clear();
    cqy.clear();
    cqz.clear();
    cqw.clear();
    local_points.clear();  // 추가된 local_points 벡터 초기화

    // Iterate through the waypoints and populate the vectors
    for (const auto& waypoint : data->waypoints) {
        cx.push_back(waypoint.pose.pose.position.x);
        cy.push_back(waypoint.pose.pose.position.y);
        cqx.push_back(waypoint.pose.pose.orientation.x);
        cqy.push_back(waypoint.pose.pose.orientation.y);
        cqz.push_back(waypoint.pose.pose.orientation.z);
        cqw.push_back(waypoint.pose.pose.orientation.w);
    }

    // Populate local_points with x and y positions
    for (const auto& waypoint : data->waypoints) {
        local_points.push_back({waypoint.pose.pose.position.x, waypoint.pose.pose.position.y});
    }
    // std::cout << "local_points size: " << local_points.size() << std::endl;
}
int GraphPlanner::calculate_distance_pose2local(){
  float min_dist = 1000.0;
  float dist = 1000.0;
  int min_idx = 0;

  for(int i=0; i<local_lane.waypoints.size(); i++){
    dist = distance(local_lane.waypoints[i].pose.pose.position.x,local_lane.waypoints[i].pose.pose.position.y,pose_x,pose_y);
    if(min_dist > dist){
      min_dist = dist;
      min_idx=i;
    }
  }

  return min_idx;
}

int GraphPlanner::calculate_distance_pose2local_1(){
  float min_dist = 1000.0;
  float dist = 1000.0;
  int min_idx = 0;

  for(int i=0; i<local_lane1.waypoints.size(); i++){
    dist = distance(local_lane1.waypoints[i].pose.pose.position.x,local_lane1.waypoints[i].pose.pose.position.y,pose_x,pose_y);
    if(min_dist > dist){
      min_dist = dist;
      min_idx=i;
    }
  }

  return min_idx;
}
void GraphPlanner::globalTrajCallback(const hmcl_msgs::LaneArray::ConstPtr& lane_msg){ 
    // std::cout << "optimal_traj" << std::endl;
    cx.clear();
    cy.clear();
    cqx.clear();
    cqy.clear();
    cqz.clear();
    cqw.clear();
    optimal_points.clear();
    global_lane_array = *lane_msg;

    global_traj_available = true;
    // Iterate through all lanes and waypoints in the received message
    for (int i=0; i<    global_lane_array.lanes[0].waypoints.size(); i++) {
            cx.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.position.x);
            cy.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.position.y);
            cqx.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.orientation.x);
            cqy.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.orientation.y);
            cqz.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.orientation.z);
            cqw.push_back(    global_lane_array.lanes[0].waypoints[i].pose.pose.orientation.w);
        }
    for (const auto& lane : lane_msg->lanes) {
        for (const auto& waypoint : lane.waypoints) {
            optimal_points.push_back({waypoint.pose.pose.position.x, waypoint.pose.pose.position.y});
        }
    }

    
    //  std::cout << cx.size() << std::endl;
    
}
void GraphPlanner::globalTrajCallback2(const hmcl_msgs::LaneArray::ConstPtr& lane_msg){ 
    // std::cout << "global_traj" << std::endl;
    global_cx.clear();
    global_cy.clear();
    global_points.clear();
    global_lane_array1 = *lane_msg;
    // Iterate through all lanes and waypoints in the received message
    for (int i=0; i<    global_lane_array1.lanes[1].waypoints.size(); i++) {
            global_cx.push_back(    global_lane_array1.lanes[1].waypoints[i].pose.pose.position.x);
            global_cy.push_back(    global_lane_array1.lanes[1].waypoints[i].pose.pose.position.y);
        }
    for (const auto& lane : global_lane_array1.lanes) {
        for (const auto& waypoint : lane.waypoints) {
            global_points.push_back({waypoint.pose.pose.position.x, waypoint.pose.pose.position.y});
        }
    }

    
    // std::cout << global_cx.size() << std::endl;
}
void GraphPlanner::compute_local_path(){
//   std::cout << "compute_local_path" << std::endl;
  int local_size = 100;
  local_lane = global_lane_array.lanes[0];
  int minidx = calculate_distance_pose2local();
  local_lane.waypoints.clear();
  local_lane.header.frame_id = "map";
  local_lane.header.stamp = ros::Time::now();
  for(int i=minidx+1; i<global_lane_array.lanes[0].waypoints.size(); i++){
    if(global_lane_array.lanes[0].waypoints.size()<minidx+2){
      ROS_INFO("LACK OF POINTS FOR LOCAL");
      break;
    }
    hmcl_msgs::Waypoint wp;
    double wp1x = global_lane_array.lanes[0].waypoints[i-1].pose.pose.position.x;
    double wp1y = global_lane_array.lanes[0].waypoints[i-1].pose.pose.position.y;
    double wp2x = global_lane_array.lanes[0].waypoints[i].pose.pose.position.x;
    double wp2y = global_lane_array.lanes[0].waypoints[i].pose.pose.position.y;
    double dist = distance(wp1x,wp1y,wp2x,wp2y);
    if(dist > 0.5){
      int n = static_cast<int>(dist/0.5);
      // ROS_INFO("ADD %d WAYPOINTS", n);
      for(int j = 0; j < n-1; j++){
        wp =     global_lane_array.lanes[0].waypoints[i-1];
        wp.pose.pose.position.x = wp1x+(wp2x-wp1x)/n*(j+1);
        wp.pose.pose.position.y = wp1y+(wp2y-wp1y)/n*(j+1);
        local_lane.waypoints.push_back(wp);
        if(local_lane.waypoints.size()>local_size) break;        
      }
    }
    wp =     global_lane_array.lanes[0].waypoints[i];
    local_lane.waypoints.push_back(wp);
    if(local_lane.waypoints.size()>local_size) break;        

  }
//   ROS_INFO("local size: %zu",local_lane.waypoints.size());
  local_lane_to_local_points();
}
void GraphPlanner::compute_local_path1(){
//   std::cout << "compute_local_path" << std::endl;
  int local_size = 100;
  local_lane1 = global_lane_array1.lanes[1];
  int minidx = calculate_distance_pose2local_1();
  local_lane1.waypoints.clear();
  local_lane1.header.frame_id = "map";
  local_lane1.header.stamp = ros::Time::now();
  for(int i=minidx+1; i<global_lane_array1.lanes[1].waypoints.size(); i++){
    if(global_lane_array1.lanes[1].waypoints.size()<minidx+2){
      ROS_INFO("LACK OF POINTS FOR LOCAL");
      break;
    }
    hmcl_msgs::Waypoint wp;
    double wp1x = global_lane_array1.lanes[1].waypoints[i-1].pose.pose.position.x;
    double wp1y = global_lane_array1.lanes[1].waypoints[i-1].pose.pose.position.y;
    double wp2x = global_lane_array1.lanes[1].waypoints[i].pose.pose.position.x;
    double wp2y = global_lane_array1.lanes[1].waypoints[i].pose.pose.position.y;
    double dist = distance(wp1x,wp1y,wp2x,wp2y);
    if(dist > 0.5){
      int n = static_cast<int>(dist/0.5);
      // ROS_INFO("ADD %d WAYPOINTS", n);
      for(int j = 0; j < n-1; j++){
        wp =     global_lane_array1.lanes[1].waypoints[i-1];
        wp.pose.pose.position.x = wp1x+(wp2x-wp1x)/n*(j+1);
        wp.pose.pose.position.y = wp1y+(wp2y-wp1y)/n*(j+1);
        local_lane1.waypoints.push_back(wp);
        if(local_lane1.waypoints.size()>local_size) break;        
      }
    }
    wp =     global_lane_array1.lanes[1].waypoints[i];
    local_lane1.waypoints.push_back(wp);
    if(local_lane1.waypoints.size()>local_size) break;        

  }
//   ROS_INFO("local size: %zu",local_lane1.waypoints.size());
  local_lane_to_global_points();
}
void GraphPlanner::local_lane_to_local_points() {

    local_points.clear();
    
    for (const auto& waypoint : local_lane.waypoints) { // 실제 사용 중인 waypoints 벡터로 변경해야 함
        local_points.push_back({waypoint.pose.pose.position.x, waypoint.pose.pose.position.y});
    }
    
    // std::cout << "local_points size: " << local_points.size() << std::endl;
}
void GraphPlanner::local_lane_to_global_points() {

    global_points.clear();
    
    for (const auto& waypoint : local_lane1.waypoints) { // 실제 사용 중인 waypoints 벡터로 변경해야 함
        global_points.push_back({waypoint.pose.pose.position.x, waypoint.pose.pose.position.y});
    }
    
    // std::cout << "local_points size: " << local_points.size() << std::endl;
}

void GraphPlanner::Graph_Generating(){
  std::cout << "Graph_Generating" << std::endl;

    double pose_s,pose_d;
    double graph_s,graph_d;
    double graph_x,graph_y;
    
    std::tie(pose_s,pose_d)= cartesian_to_frenet(global_points,{pose_x,pose_y});
    graph_position[{-1,0}]={pose_x,pose_y};
    for(int j=0; j <= graph_horizon; j++){
      graph_s=pose_s+(j+1)*10;
      for(int i=-graph_vertical; i<=graph_vertical; i++){
        graph_d=pose_d+i;
        std::tie(graph_x,graph_y) = frenet_to_cartesian(global_points,graph_s,graph_d);
        graph_position[{j,i}]={graph_x, graph_y};
        }
    
}
    for(int k=-graph_vertical; k<=graph_vertical; k++){

      graph_edge[{-1,0}].push_back({0,k});
    }
    for(int j=0; j <= graph_horizon-1; j++){
      for(int i=-graph_vertical; i<=graph_vertical; i++){
       for(int k=-graph_vertical; k<=graph_vertical; k++){
        graph_edge[{j,i}].push_back({j+1,k});
      }
    }
  }
  Edge_Destroying();
}
void GraphPlanner::Edge_Destroying(){
  std::cout << "Edge_Destroying" << std::endl;
    std::vector<double> target_s_list,target_d_list,target_x_list,target_y_list,target_dim_x_list,target_dim_y_list,target_yaw_list;
    for (const auto& obj : objects_data.objects) {
        int obj_key = obj.id;
        double target_s, target_d; // Using ID as key
        std::tie(target_s,target_d)=cartesian_to_frenet(global_points,{target_x[obj_key],target_y[obj_key]});
        target_s_list.push_back(target_s);
        target_d_list.push_back(target_d);
        target_dim_x_list.push_back(target_dimension_x[obj_key]);
        target_dim_y_list.push_back(target_dimension_y[obj_key]);
        target_x_list.push_back(target_x[obj_key]);
        target_y_list.push_back(target_y[obj_key]);
        target_yaw_list.push_back(atan(target_velocity_y[obj_key]/target_velocity_x[obj_key]));
    }
    for (int i=0; i< target_s_list.size(); i++){
        double pose_s, pose_d;
        std::tie(pose_s,pose_d)= cartesian_to_frenet(global_points,{pose_x,pose_y});
        double target_s=target_s_list[i];
        double target_d=target_d_list[i];
        double target_dim_x=target_dimension_x[i];
        double target_dim_y=target_dimension_y[i];
        double target_yaw=target_yaw_list[i];
        std::pair<double, double> first_point, second_point, third_point, fourth_point;
        double first_s, second_s, third_s, fourth_s;
        double first_d, second_d, third_d, fourth_d;
        std::vector<std::pair<double, double>> points_cartesian;
        std::vector<std::pair<double, double>> points_frenet;
        first_point.first=pose_x+target_dim_x*cos(target_yaw)-target_dim_y*sin(target_yaw);
        first_point.second=pose_y+target_dim_x*sin(target_yaw)+target_dim_y*cos(target_yaw);
        second_point.first=pose_x-(target_dim_x*cos(target_yaw)-target_dim_y*sin(target_yaw));
        second_point.second=pose_y+target_dim_x*sin(target_yaw)+target_dim_y*cos(target_yaw);
        third_point.first=pose_x-(target_dim_x*cos(target_yaw)-target_dim_y*sin(target_yaw));
        third_point.second=pose_y-(target_dim_x*sin(target_yaw)+target_dim_y*cos(target_yaw));
        fourth_point.first=pose_x+target_dim_x*cos(target_yaw)-target_dim_y*sin(target_yaw);
        fourth_point.second=pose_y-(target_dim_x*sin(target_yaw)+target_dim_y*cos(target_yaw));
        points_cartesian.push_back(first_point);
        points_cartesian.push_back(second_point);
        points_cartesian.push_back(third_point);
        points_cartesian.push_back(fourth_point);
        std::tie(first_s,first_d)= cartesian_to_frenet(global_points,{first_point.first,first_point.second});
        std::tie(second_s,second_d)= cartesian_to_frenet(global_points,{second_point.first,second_point.second});
        std::tie(third_s,third_d)= cartesian_to_frenet(global_points,{third_point.first,third_point.second});
        std::tie(fourth_s,fourth_d)= cartesian_to_frenet(global_points,{fourth_point.first,fourth_point.second});
        points_frenet.push_back({first_s,first_d});
        points_frenet.push_back({second_s,second_d});
        points_frenet.push_back({third_s,third_d});
        points_frenet.push_back({fourth_s,fourth_d});
        int min_graph_horizon=(std::min({first_s,second_s,third_s,fourth_s})-pose_s)/10;
        int max_graph_horizon=(std::max({first_s,second_s,third_s,fourth_s})-pose_s)/10;
        for (int k=-graph_vertical; k<=graph_vertical; k++){
          for(int j=min_graph_horizon; j<=max_graph_horizon; j++){
              double graph_x, graph_y;
              std::vector<double> angle_list;
              std::tie(graph_x,graph_y)=graph_position[{j,k}];
              for(std::pair<double, double> point :points_cartesian){
                double angle=atan((point.second-graph_y)/(point.first-graph_x));
                if(point.first-graph_x>0){
                angle_list.push_back(atan((point.second-graph_y)/(point.first-graph_x)));
                }
              }
              double angle_max=0;
              if (!angle_list.empty()) { // Ensure the vector is not empty to avoid dereferencing end iterator
                auto max_iter = std::max_element(angle_list.begin(), angle_list.end());
                 double angle_max = *max_iter; // Dereference iterator to get the value
              } else {
              // Handle the case where angle_list is empty if necessary
              double angle_max = 0; // Default or error value
              }
              double angle_min=0;
              if (!angle_list.empty()) { // Ensure the vector is not empty to avoid dereferencing end iterator
                auto min_iter = std::min_element(angle_list.begin(), angle_list.end());
                double angle_min = *min_iter; // Dereference iterator to get the value
              } else {
              // Handle the case where angle_list is empty if necessary
              double angle_min = 0; // Default or error value
              }
              for(std::pair<int,int> point :graph_edge[{j,k}]){
                double target_graph_x, target_graph_y;
                std::tie(target_graph_x, target_graph_y)=graph_position[{point.first,point.second}];
                double angle=atan((target_graph_y-graph_y)/(target_graph_x-graph_x));
                if (angle_min< angle && angle< angle_max){
                  graph_edge[{j,k}].erase(std::remove(graph_edge[{j,k}].begin(), graph_edge[{j,k}].end(), std::make_pair(point.first, point.second)), graph_edge[{j,k}].end());
                }
              }
          }
        }


    }
    Cost_Generating();
}
void GraphPlanner::Cost_Generating(){
  std::cout << "Cost_Generating" << std::endl;
  for (int k=-graph_vertical; k<=graph_vertical; k++){
    double graph_x,graph_y;
    std::tie(graph_x,graph_y)=graph_position[{-1,0}];
    double target_graph_x,target_graph_y;
    std::tie(target_graph_x, target_graph_y)=graph_position[{0,k}];
    double cost=distance(graph_x,graph_y,target_graph_x,target_graph_y);
    edge_cost[{-1,0}][{0, k}] = cost;
    std::cout << "Edge from (" << -1 << ", " << 0 << ") to ("
                          << 0 << ", " << k << ")"
                          << " has a cost: " << cost << std::endl;
  }
  for (int k=-graph_vertical; k<=graph_vertical; k++){
    for(int j=0; j<=graph_horizon; j++){
      double graph_x,graph_y;
      std::tie(graph_x,graph_y)=graph_position[{j,k}];
      for(std::pair<int,int> point :graph_edge[{j,k}]){
        double target_graph_x,target_graph_y;
        std::tie(target_graph_x, target_graph_y)=graph_position[{point.first,point.second}];
        double cost=distance(graph_x,graph_y,target_graph_x,target_graph_y);
        edge_cost[{j,k}][{point.first, point.second}] = cost;
        std::cout << "Edge from (" << j << ", " << k << ") to ("
                          << point.first << ", " << point.second << ")"
                          << " has a cost: " << cost << std::endl;


      
      }
    }
  }
  std::vector<std::pair<int,int>> path=dijkstra({-1,0},{graph_horizon,0},edge_cost);\
  std::cout << "Path: ";
    for (const auto& node : path) {
        std::cout << "(" << node.first << ", " << node.second << ") ";
    }
    std::cout << std::endl;
}
std::vector<std::pair<int,int>> GraphPlanner::dijkstra(std::pair<int,int> start_node, std::pair<int,int> target, std::map<std::pair<int, int>, std::map<std::pair<int, int>, double>> edge_cost){
  std::map<std::pair<int,int>,std::pair<int,int>,VectorComparator> previous;
  std::priority_queue<std::pair<double, std::pair<int,int>>, std::vector<std::pair<double, std::pair<int,int>>>, std::greater<std::pair<double, std::pair<int,int>>>> pq;
  // std::map<std::pair<int, int>, double, VectorComparator> node_cost;
  for (int j=0; j<=graph_horizon; j++){
    for (int k=-graph_vertical; k<=graph_vertical; k++){
      node_cost[{j,k}]=10000000;
    }

  }
  
  node_cost[start_node]=0;
  pq.push({0,start_node});
  while(!pq.empty()) {
    double current_distance =pq.top().first;
    std::pair<int,int> current_node=pq.top().second;
    pq.pop();
    if (current_distance >node_cost[{current_node.first,current_node.second}]) continue;
    for (std::pair<int,int> next_node : graph_edge[{current_node.first,current_node.second}]){
      double new_cost = current_distance + edge_cost[{next_node.first,next_node.second}][{current_node.first,current_node.second}];
      if (new_cost < node_cost[{next_node.first,next_node.second}]) {
                node_cost[{next_node.first,next_node.second}] = new_cost;
                previous[{next_node.first,next_node.second}] = current_node;
                pq.push({new_cost, next_node});
            }
    }
  }
  // if (node_cost[{target.first,target.second}] ==10000000 ) {
  //       std::cout << "No path exists from node " << start_node << " to node " << target << std::endl;
  //       return {};
  //   }
  std::vector<std::pair<int,int>> path;
    for (std::pair<int,int> at = target; at != start_node; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start_node);
    std::reverse(path.begin(), path.end());
    return path;

}



void GraphPlanner::OG_PUB() {
    hmcl_msgs::Lane trajectory;
    int i = 0;
    // std::cout<<cx.size()<<std::endl;

    while (i < cx.size()) {
        hmcl_msgs::Waypoint waypoint;
        waypoint.pose.pose.position.x = cx[i];
        waypoint.pose.pose.position.y = cy[i];
        waypoint.pose.pose.orientation.x = cqx[i];
        waypoint.pose.pose.orientation.y = cqy[i];
        waypoint.pose.pose.orientation.z = cqz[i];
        waypoint.pose.pose.orientation.w = cqw[i];
        trajectory.waypoints.push_back(waypoint);
        i++;
    }
    overtaking_traj_pub_.publish(trajectory);

    // Visualize the local path using markers
    visualization_msgs::MarkerArray marker_array = visualize_local_path(trajectory.waypoints);
    marker_pub_.publish(marker_array);
    // std::cout << "work2222" << std::endl;
}

visualization_msgs::MarkerArray GraphPlanner::visualize_local_path(const std::vector<hmcl_msgs::Waypoint>& waypoints) {
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




void GraphPlanner::run() {
    ros::Rate loop_rate(10);  // Adjust the rate as necessary
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "GraphPlanner");
    GraphPlanner model_predictive;
    model_predictive.run();
    return 0;
}
