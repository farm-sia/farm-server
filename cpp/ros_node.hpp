#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RosNode {
public:
	MoveBaseClient* ac;

    static void map_topic(const nav_msgs::OccupancyGrid& grid);
    static void pose_topic(const geometry_msgs::PoseStamped& pose);
    static void vel_topic(const geometry_msgs::Twist& twist);
	void set_move_goal(float pos_x, float pos_y, float pos_z);
    RosNode(int argc, char** argv);
};
