import * as WebSocket from "ws"
import * as raspi from 'raspi'
import * as pwm from 'raspi-soft-pwm'

var IN1A:pwm.SoftPWM
var IN2A:pwm.SoftPWM

var IN1B:pwm.SoftPWM
var IN2B:pwm.SoftPWM

var direction = "n"
var speed = 0

const wss = new WebSocket.Server({ port: 5555 })
wss.on("listening", () => {
	console.log("[ws] server up")
})

raspi.init(() => {
	console.log("[raspi] init finished")	
	IN1A = new pwm.SoftPWM({pin: 'GPIO23', frequency: 2000})
	IN2A = new pwm.SoftPWM({pin: 'GPIO24', frequency: 2000})
	
	IN1B = new pwm.SoftPWM({pin: 'GPIO27', frequency: 2000})
	IN2B = new pwm.SoftPWM({pin: 'GPIO22', frequency: 2000})
})

function reset_motors() {
	direction = "n"
	speed = 0
	update_pwm_pins()
	console.log("[ws] client disconnected, stopping motors")
}

wss.on('connection', (ws, req) => {
	ws.on('message', data => {
		handle_message(data.toString())
	})

	console.log("[ws] connected to client " + req.socket.remoteAddress)


	ws.on('close', reset_motors)
	ws.on('error', reset_motors)
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
	var r_speed = speed
	var l_speed = speed

	if (direction == "r") {
		r_speed *= 0.5
	} else if (direction == "l") {
		l_speed *= 0.5
	}

	if (r_speed > 0) {
		IN1A.write(0)
		IN2A.write(r_speed)
	} else {	
		IN1A.write(Math.abs(r_speed))
		IN2A.write(0)
	}

	if (l_speed > 0) {
		IN1B.write(0)
		IN2B.write(l_speed)
	} else {	
		IN1B.write(Math.abs(l_speed))
		IN2B.write(0)
	}
}
