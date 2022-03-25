#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

class RosNode {
public:
    static void map_topic(const std_msgs::StringConstPtr& str);
    RosNode(int argc, char** argv);
};