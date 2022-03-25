#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>

class RosNode {
public:
    static void map_topic(const nav_msgs::OccupancyGrid& grid);
    static void pose_topic(const std_msgs::StringConstPtr& str);
    RosNode(int argc, char** argv);
};