import * as main from "./main"
import * as raspi from 'raspi'
import * as pwm from 'raspi-soft-pwm'

var direction = "n"
var speed = 0

var IN1A:pwm.SoftPWM
var IN2A:pwm.SoftPWM

var IN1B:pwm.SoftPWM
var IN2B:pwm.SoftPWM

export function init_pi() {
	raspi.init(() => {
		console.log("[raspi] init finished")	
		IN1A = new pwm.SoftPWM({pin: 'GPIO23', frequency: 2000})
		IN2A = new pwm.SoftPWM({pin: 'GPIO24', frequency: 2000})
		
		IN1B = new pwm.SoftPWM({pin: 'GPIO27', frequency: 2000})
		IN2B = new pwm.SoftPWM({pin: 'GPIO22', frequency: 2000})
	})
}

export function reset_motors() {
	update_pwm_pins("n", 0)
	console.log("[ws] client disconnected, stopping motors")
}

export function update_pwm_pins(_direction: string | undefined, _speed: number | undefined) {
	if (_direction != undefined) direction = _direction
	if (_speed != undefined) speed = _speed

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
