#include "ros_node.hpp"
#include "main.hpp"
#include "json.hpp"
#include <iostream>

nlohmann::json pose_to_json(const geometry_msgs::Pose& pose) {
    nlohmann::json position;
    position["x"] = pose.position.x;
    position["y"] = pose.position.y;
    position["z"] = pose.position.z;
    
    nlohmann::json orientation;
    orientation["x"] = pose.orientation.x;
    orientation["y"] = pose.orientation.y;
    orientation["z"] = pose.orientation.z;
    orientation["w"] = pose.orientation.w;

    nlohmann::json j;
    j["position"] = position;
    j["orientation"] = orientation;
    return j;
}

nlohmann::json pose_stamped_to_json(const geometry_msgs::PoseStamped& pose) {
    nlohmann::json j;
    j["pose"] = pose_to_json(pose.pose);
    return j;
}

nlohmann::json occ_grid_to_json(const nav_msgs::OccupancyGrid& grid) {
    nlohmann::json j;
    j["data"] = nlohmann::json(grid.data);
    nlohmann::json info;
    info["resolution"] = grid.info.resolution;
    info["width"] = grid.info.width;
    info["height"] = grid.info.height;
    info["origin"] = pose_to_json(grid.info.origin);
    j["info"] = info;
    return j;
}

void RosNode::map_topic(const nav_msgs::OccupancyGrid& grid) {

    ws_server->send("ros_map", occ_grid_to_json(grid));
}

void RosNode::pose_topic(const geometry_msgs::PoseStamped& pose) {
    ws_server->send("ros_pose", pose_stamped_to_json(pose));
}

RosNode::RosNode(int argc, char** argv) {
    ros::init(argc, argv, "farm_server");
    ros::NodeHandle nh;
    ros::Subscriber map_topic_sub = nh.subscribe("map", 1, RosNode::map_topic);
    ros::Subscriber pose_topic_sub = nh.subscribe("tracked_pose", 1, RosNode::pose_topic);
    std::cout << "[ros] node init finished, starting spin..." << std::endl;
    ros::spin();
}