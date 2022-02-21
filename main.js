const WebSocket = require("ws")

const wss = new WebSocket.Server({ port: 5555 });

wss.on('connection', ws => {
	ws.on('message', data => {
		handle_message(data.toString())
	})

	console.log("connected to client " + ws._socket.remoteAddress)
})

function handle_message(msg) {
	content = msg.slice(msg.indexOf(" ") + 1)
	command = msg.slice(0, msg.indexOf(" "))
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
