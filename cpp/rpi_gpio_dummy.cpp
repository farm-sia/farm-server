#include <iostream>
#include <cmath>

#include"rpi_gpio.hpp"

#define MAX_PWM 100

RpiGpio::RpiGpio() {
	std::cout << "[rpi] dummy init finished" << std::endl;
}

void RpiGpio::update_pwm_pins() {

}

void RpiGpio::reset_motors () {
	std::cout << "[rpi] dummy reset motors" << std::endl;
}
