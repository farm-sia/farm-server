#include "ros_node.hpp"
#include "main.hpp"
#include "json.hpp" // ermöglicht (de-)serialisieren von JSON Strings und Objekten
#include <iostream> // ermöglicht Input und Output des Programms
#include <thread> // ermöglicht das Starten von Threads mithilfe der cpp Standard Library

#include <boost/thread.hpp> // ermöglicht das Starten von Threads mithilfe der boost Library

// Funktion, die das Umwandeln von der ROS Message "pose" zu einem JSON Objekt ermöglicht. Der Aufbau dieser Datenstruktur ist hier zu finden: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
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

// Funktion, die die ROS Message "poseStamped" zu JSON Objekten konvertiert. Aufbau der ROS Message "poseStamped": https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html
nlohmann::json pose_stamped_to_json(const geometry_msgs::PoseStamped& pose) {
	nlohmann::json j;
	j["pose"] = pose_to_json(pose.pose);
	return j;
}

// Funktion, die die ROS Message "OccupancyGrid" zu JSON Objekten konvertiert. Aufbau der ROS Message "OccupancyGrid": https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html_
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

// Methode, die ein OccupancyGrid über den WebSocket Server versendet
void RosNode::map_topic(const nav_msgs::OccupancyGrid& grid) {
	ws_server->send("ros_map", occ_grid_to_json(grid));
}

// Methode, die eine Pose über den WebSocket Server versendet
void RosNode::pose_topic(const geometry_msgs::PoseStamped& pose) {
	ws_server->send("ros_pose", pose_stamped_to_json(pose));
}

// Funktion, die einen 3 dimensionalen Vector im ROS Format (https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Vector3.html) zu einem JSON Objekt umwandelt
nlohmann::json vec3_to_json(const geometry_msgs::Vector3& vec) {
	nlohmann::json j;
	j["x"] = vec.x;
	j["y"] = vec.y;
	j["z"] = vec.z;
	return j;
}

// Funktion, die die ROS Message "Twist" zu JSON Objekten konvertiert. Aufbau der ROS Message "Twist": https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html_
nlohmann::json twist_to_json(const geometry_msgs::Twist& twist) {
	nlohmann::json j;
	j["linear"] = vec3_to_json(twist.linear);
	j["angular"] = vec3_to_json(twist.angular);
	return j;
}

// Methode, die einen Twist über den WebSocket Server versendet
void RosNode::vel_topic(const geometry_msgs::Twist& twist) {
	ws_server->send("ros_vel", twist_to_json(twist));
}

// Funktion, die das Ziel für das Path Finding über die actionlib setzt
void send_move_goal(move_base_msgs::MoveBaseGoal g) {
	MoveBaseClient ac("move_base", false); // erstelle eine Instanz der MoveBaseClient Klasse mit dem "Ziel" move_base
	while(!ac.waitForServer(ros::Duration(5.0))){ // Schleife, die wartet, bis der ROS Server gestartet hat, falls er es noch nicht hat
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Sending goal"); // Schreibe Info in die Konsole
	ac.sendGoal(g); // sende MoveBaseGoal Message über die Action API (https://docs.ros.org/en/api/actionlib/html/index.html)

	ac.waitForResult(); // da es in einem eigenen Thread ist, kann auf die Antwort gewartet werden, ohne dass die anderen Teile des Servers geblockt werden

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { // wenn die Antwort auf einen Erfolg schließen lässt
		ROS_INFO("Hooray, the base moved 1 meter forward"); // Schreibe dies in die Konsole
		ws_server->send("goal_reached"); // und sende es an den WebSocket Client
	} else {
		ROS_INFO("The base failed to move forward 1 meter for some reason"); // bei einem Fehler schreibe diesen auch in die Konsole
		ws_server->send("goal_failed"); // und sende dies auch an den WebSocket Client
	}
}

// Methode, die das Ziel für das Path Finding setzen kann
void RosNode::set_move_goal(float pos_x, float pos_y, float pos_z) {
	std::cout << "publish goal start" << std::endl; // Schreibe zum Debuggen Text in die Konsole
	move_base_msgs::MoveBaseGoal goal; // erstelle eine Variable vom Typ MoveBaseGoal

	goal.target_pose.header.frame_id = "base_link"; // Setzte den Namen des Teils, das sich bewegen soll (angegeben in dem URDF Plan, hier "base_link" => Grundplatte)
	goal.target_pose.header.stamp = ros::Time::now(); // Setze den Zeitpunkt der gesendeten Nachricht auf den aktuellen Zeitpunkt

	// Setze die Ppsition und Roation der gewünschten Position
	goal.target_pose.pose.position.x = pos_x;
	goal.target_pose.pose.position.y = pos_y;
	goal.target_pose.pose.position.z = pos_z;
	goal.target_pose.pose.orientation.w = 1.0;
	std::cout << "publish goal created goal " << pos_x << " " << pos_y << std::endl;

	std::thread t(send_move_goal, goal); // starte einen neuen Thread, um die Actionlib zu benutzen
	t.detach(); // Detache den Thread, somit das Senden der Message nicht den aktuellen Thread blockiert
}

// Funktion, die den ROS Spin startet, der sich um eingehende Events kümmert und dann die richtigen Callbacks aufruft
void ros_spin() {
	ros::spin();
}

// Konstruktor der RosNode Klasse
RosNode::RosNode(int argc, char** argv) {
	ros::init(argc, argv, "farm_server"); // erstellt die eigene ROS Node und "Registriert die beim ROS Server"
	ros::NodeHandle nh; // erstellt einen NodeHandler, der sich um die Verschiedenen Events, die von Nodes ausgehen können, kümmern kann
	ros::Subscriber map_topic_sub = nh.subscribe("map", 1, RosNode::map_topic); // registriert einen Callback für das Topic namens "map", die regelmäßig die aktuelle Karte veröffentlicht
	ros::Subscriber pose_topic_sub = nh.subscribe("tracked_pose", 1, RosNode::pose_topic); // registriert einen Callback für das Topic namens "tracked_pose", das ebenfalls regelmäßig die aktuelle Position veröffentlicht
	ros::Subscriber vel_topic_sub = nh.subscribe("cmd_vel", 1, RosNode::vel_topic); // registriert einen Callback für das Topic namens "cmd_vel", das ebenfalls regelmäßig die aktuellen Bewegungsbefehle veröffentlicht

	boost::thread spin_thread(&ros_spin); // startet einen boost::thread für den ros::spin

	std::cout << "[ros] node init finished, starting spin..." << std::endl;
	spin_thread.join(); // führt join() auf den Thread aus, so dass er den aktuellen Thread blockiert
}
