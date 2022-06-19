#include <iostream>
#include <cmath>

#include"rpi_gpio.hpp"

#define MAX_PWM 100

/*
	Diese Datei implementiert dieselben Methoden wie die Datei
	rpi_gpio.cpp, nur ohne dass diese Methoden irgendwas ausführen.
	Dies ermöglicht das Ausführen des Servers auf PCs, die keine Raspberry PIs sind
*/

RpiGpio::RpiGpio() {
	std::cout << "[rpi] dummy init finished" << std::endl;
}

void RpiGpio::update_pwm_pins() {

}

void RpiGpio::reset_motors () {
	std::cout << "[rpi] dummy reset motors" << std::endl;
}
