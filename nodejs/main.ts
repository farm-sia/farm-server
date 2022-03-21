import * as websocket_server from "./websocket_server"
import * as raspi_pins from "./raspi_pins"
const ros_node = require("./ros_node")

websocket_server.init_ws()
raspi_pins.init_pi()
ros_node.init_ros_node()
