import * as WebSocket from "ws"
import * as raspi from 'raspi'
import * as pwm from 'raspi-soft-pwm'

var IN1A:pwm.SoftPWM
var IN2A:pwm.SoftPWM

var direction = "n"
var speed = 0

const wss = new WebSocket.Server({ port: 5555 })
wss.on("listening", () => {
	console.log("[ws] server up")
})
raspi.init(() => {
	console.log("[raspi] init finished")	
	IN1A = new pwm.SoftPWM({pin: 'GPIO27', frequency: 2000})
	IN2A = new pwm.SoftPWM({pin: 'GPIO22', frequency: 2000})
})

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
			if (["n", "l", "r"].includes(content)) {
				direction = content
				update_pwm_pins()
			}
			break
		case "speed":
			if (parseFloat(content) >= -1 && parseFloat(content) <= 1) {
				speed = parseFloat(content)
				update_pwm_pins()
			}
			break
		default:
			console.log("wrong command")
	}
}

function update_pwm_pins() {
	console.log(speed, direction)
	if (speed > 0) {
		IN1A.write(speed)
		IN2A.write(0)
	} else {	
		IN1A.write(0)
		IN2A.write(Math.abs(speed))
	}
}
