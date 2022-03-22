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
	var content = JSON.parse(msg)
	switch (content["packet"]) {
		case "steering":
			if (content["direction"]) {
				if (["n", "l", "r"].includes(content["direction"])) {
					raspi_pins.update_pwm_pins(content["direction"], undefined)
				}
			}
			if (content["speed"]) {
				if (parseFloat(content["speed"]) >= -1 && parseFloat(content["speed"]) <= 1) {
					raspi_pins.update_pwm_pins(undefined, parseFloat(content["speed"]))
				}
			}
			break;
	}
}

export function send_message(header: string, content: string) {
	wss.clients.forEach(function(client) {
		client.send(`${header} ${content}`)
	})
}
