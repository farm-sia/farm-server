#include <iostream>
#include <pigpio.h>
#include <cmath>

#include"rpi_gpio.hpp"

RpiGpio::RpiGpio() {
	gpioInitialise();

	gpioSetMode(IN1A, PI_OUTPUT);
	gpioSetMode(IN2A, PI_OUTPUT);
	gpioSetMode(IN1B, PI_OUTPUT);
	gpioSetMode(IN2B, PI_OUTPUT);

	std::cout << "[rpi] init finished" << std::endl;
}

void RpiGpio::update_pwm_pins() {
	float r_speed = speed;
	float l_speed = speed;

	if (direction == 'r') {
		r_speed *= 0.5;
	} else if (direction == 'l') {
		l_speed *= 0.5;
	}

	if (r_speed > 0) {
		gpioPWM(IN1A, 0);
		gpioPWM(IN2A, r_speed * 255);
	} else {	
		gpioPWM(IN1A, std::abs(r_speed) * 255),
		gpioPWM(IN2A, 0);
	}

	if (l_speed > 0) {
		gpioPWM(IN1B, 0);
		gpioPWM(IN2B, l_speed * 255);
	} else {	
		gpioPWM(IN1B, std::abs(l_speed) * 255);
		gpioPWM(IN2B, 0);
	}
}

void RpiGpio::reset_motors () {
	direction = 'n';
	speed = 0;
	update_pwm_pins();
	std::cout << "[rpi] reset motors" << std::endl;
}
