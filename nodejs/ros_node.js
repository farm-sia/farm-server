const rosnodejs = require("rosnodejs")
const ws = require("./websocket_server")

function init_ros_node() {
	rosnodejs.initNode("/farm_server").then(() => {
		console.log("[ros] node up")
	})

	rosnodejs.nh.subscribe("/map", "nav_msgs/OccupancyGrid", msg => {
		ws.send_message("occupancy_grid", JSON.stringify(msg))
	})

	rosnodejs.nh.subscribe("tracked_pose", "geometry_msgs/PoseStamped", msg => {
		ws.send_message("tracked_pose", JSON.stringify(msg))
	})
}

module.exports = {
	init_ros_node: init_ros_node
}
