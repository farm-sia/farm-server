import * as WebSocket from "ws"
import * as raspi_pins from "./raspi_pins"

export var wss: WebSocket.Server;

export function init_ws() {
	wss = new WebSocket.Server({ port: 5555 })
	wss.on("listening", () => {
		console.log("[ws] server up")
	})

	wss.on('connection', (ws, req) => {
		ws.on('message', data => {
			handle_message(data.toString())
		})

		console.log("[ws] connected to client " + req.socket.remoteAddress)

		ws.on('close', raspi_pins.reset_motors)
		ws.on('error', raspi_pins.reset_motors)
	})
}

function handle_message(msg: string) {
	var content = msg.slice(msg.indexOf(" ") + 1)
	var command = msg.slice(0, msg.indexOf(" "))
	switch (command) {
		case "direction":
			if (["n", "l", "r"].includes(content)) {
				raspi_pins.update_pwm_pins(content, undefined)
			}
			break
		case "speed":
			if (parseFloat(content) >= -1 && parseFloat(content) <= 1) {
				raspi_pins.update_pwm_pins(undefined, parseFloat(content))
			}
			break
		default:
			console.log("wrong command")
	}
}

export function send_message(header: string, content: string) {
	wss.clients.forEach(function(client) {
		client.send(`${header} ${content}`)
	})
}
