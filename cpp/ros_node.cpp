#include "ros_node.hpp"
#include "main.hpp"
#include "json.hpp"
#include <iostream>
#include <thread>

#include <boost/thread.hpp>

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

nlohmann::json vec3_to_json(const geometry_msgs::Vector3& vec) {
	nlohmann::json j;
	j["x"] = vec.x;
	j["y"] = vec.y;
	j["z"] = vec.z;
	return j;

}

nlohmann::json twist_to_json(const geometry_msgs::Twist& twist) {
	nlohmann::json j;
	j["linear"] = vec3_to_json(twist.linear);
	j["angular"] = vec3_to_json(twist.angular);
	return j;
}

void RosNode::vel_topic(const geometry_msgs::Twist& twist) {
    ws_server->send("ros_vel", twist_to_json(twist));
}

void send_move_goal(move_base_msgs::MoveBaseGoal g, MoveBaseClient* accc) {
 //wait for the action server to come up
	MoveBaseClient ac("move_base", false);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

}

void RosNode::set_move_goal(float pos_x, float pos_y, float pos_z) {
	std::cout << "publish goal start" << std::endl;
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = pos_x;
	goal.target_pose.pose.position.y = pos_y;
	goal.target_pose.pose.position.z = pos_z;
	goal.target_pose.pose.orientation.w = 1.0;
	std::cout << "publish goal created goal" << std::endl;

	std::thread t(send_move_goal, goal, nullptr);
	t.detach();
}

void ros_spin() {
	ros::spin();
}

RosNode::RosNode(int argc, char** argv) {
    ros::init(argc, argv, "farm_server");
    ros::NodeHandle nh;
    ros::Subscriber map_topic_sub = nh.subscribe("map", 1, RosNode::map_topic);
    ros::Subscriber pose_topic_sub = nh.subscribe("tracked_pose", 1, RosNode::pose_topic);
	ros::Subscriber vel_topic_sub = nh.subscribe("cmd_vel", 1, RosNode::vel_topic);

	boost::thread spin_thread(&ros_spin);

	std::cout << "[ros] node init finished, starting spin..." << std::endl;
	spin_thread.join();
}
