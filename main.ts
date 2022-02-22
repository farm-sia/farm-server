import * as WebSocket from "ws"
import * as raspi from 'raspi'
import * as pwm from 'raspi-soft-pwm'

const wss = new WebSocket.Server({ port: 5555 })
wss.on("listening", () => {
	console.log("[ws] server up")
})
raspi.init(() => console.log("[raspi] init finished"))

wss.on('connection', (ws, req) => {
	ws.on('message', data => {
		handle_message(data.toString())
	})

	console.log("[ws] connected to client " + req.socket.remoteAddress)
})

function handle_message(msg: string) {
	var content = msg.slice(msg.indexOf(" ") + 1)
	var command = msg.slice(0, msg.indexOf(" "))
	switch (command) {
		case "direction":
			console.log("direction: %s", content)
			break
		case "speed":
			console.log("speed: %s", content)
			break
		default:
			console.log("wrong command")
	}
}
