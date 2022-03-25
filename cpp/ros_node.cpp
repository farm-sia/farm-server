#include "ros_node.hpp"
#include "main.hpp"
#include <iostream>

void RosNode::map_topic(const nav_msgs::OccupancyGrid& str) {
    ws_server->send("ros_map", "got map");
}

void RosNode::pose_topic(const std_msgs::StringConstPtr& str) {
    ws_server->send("ros_pose", str->data.c_str());
}

RosNode::RosNode(int argc, char** argv) {
    ros::init(argc, argv, "farm_server");
    ros::NodeHandle nh;
    ros::Subscriber map_topic_sub = nh.subscribe("map", 1, RosNode::map_topic);
    //ros::Subscriber pose_topic_sub = nh.subscribe("tracked_pose", 1, RosNode::pose_topic);
    std::cout << "[ros] node init finished, starting spin..." << std::endl;
    ros::spin();
}