#pragma once // sorgt dafür, dass diese Datei nur einmal importiert wird

// Importieren der Dateien/Bibliotheken, die in der Datei ros_node.cpp gebraucht werden
#include <ros/ros.h> // ermöglicht Zugriff auf die ROS API
#include <std_msgs/String.h> // Typdefinitionen für die ROS Message "String"
#include <nav_msgs/OccupancyGrid.h> // Typdefinitionen für die ROS Message "OccupancyGrid"
#include <geometry_msgs/PoseStamped.h> // Typdefinitionen für die ROS Message "PoseStamped"
#include <geometry_msgs/Twist.h> // Typdefinitionen für die ROS Message "Twist"
#include <geometry_msgs/Vector3.h> // Typdefinitionen für die ROS Message "Vector3"
#include <move_base_msgs/MoveBaseAction.h> // Typdefinitionen für die ROS Message "MoveBaseAction"
#include <actionlib/client/simple_action_client.h> // Ermöglicht Zugriff auf die Actionlib API

// Typdefinition für den Typ MoveBaseClient um den Code lesbarer zu machen
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// deklarieren der Klasse RosNode
class RosNode {
public:
	MoveBaseClient* ac; // Pointer, der die Instanz des MoveBaseClient speichert

	// Prototypen der Methoden, die in der ros_node.cpp definiert werden
    static void map_topic(const nav_msgs::OccupancyGrid& grid);
    static void pose_topic(const geometry_msgs::PoseStamped& pose);
    static void vel_topic(const geometry_msgs::Twist& twist);
	void set_move_goal(float pos_x, float pos_y, float pos_z);
    RosNode(int argc, char** argv);
};
