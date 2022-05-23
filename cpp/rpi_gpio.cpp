#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>

#include"rpi_gpio.hpp"

#define MAX_PWM 100

RpiGpio::RpiGpio() {
	if (wiringPiSetupGpio() != 0) std::cout << "[rpi] error initializing raspi" << std::endl;

	softPwmCreate(IN1A, 0, MAX_PWM);
	softPwmCreate(IN2A, 0, MAX_PWM);
	softPwmCreate(IN1B, 0, MAX_PWM);
	softPwmCreate(IN2B, 0, MAX_PWM);

	std::cout << "[rpi] init finished" << std::endl;
}

void RpiGpio::update_pwm_pins(float l_speed, float r_speed) {
	if (r_speed > 0) {
		softPwmWrite(IN1A, 0);
		softPwmWrite(IN2A, r_speed * MAX_PWM);
	} else {	
		softPwmWrite(IN1A, std::abs(r_speed) * MAX_PWM);
		softPwmWrite(IN2A, 0);
	}
	if (l_speed > 0) {
		softPwmWrite(IN1B, 0);
		softPwmWrite(IN2B, l_speed * MAX_PWM);
	} else {	
		softPwmWrite(IN1B, std::abs(l_speed) * MAX_PWM);
		softPwmWrite(IN2B, 0);
	}
}

void RpiGpio::reset_motors () {
	update_pwm_pins(0, 0);
	std::cout << "[rpi] reset motors" << std::endl;
}
