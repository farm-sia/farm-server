#include "ros_node.hpp"
#include <iostream>

void RosNode::map_topic(const std_msgs::StringConstPtr& str) {

}

RosNode::RosNode(int argc, char** argv) {
    ros::init(argc, argv, "farm_server");
    ros::NodeHandle nh;
    ros::Subscriber map_topic_sub = nh.subscribe("map", 1, RosNode::map_topic);
    std::cout << "[ros] node init finished" << std::endl;
}