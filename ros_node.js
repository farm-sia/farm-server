const rosnodejs = require("rosnodejs")

function init_ros_node() {
	rosnodejs.initNode("/farm_server").then(() => {
		console.log("[ros] node up")
	})
}

module.exports = {
	init_ros_node: init_ros_node
}
