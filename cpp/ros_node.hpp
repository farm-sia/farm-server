#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class RosNode {
public:
    static void map_topic(const nav_msgs::OccupancyGrid& grid);
    static void pose_topic(const geometry_msgs::PoseStamped& pose);
    static void vel_topic(const geometry_msgs::Twist& twist);
    RosNode(int argc, char** argv);
};
