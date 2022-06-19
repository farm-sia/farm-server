// Importieren der Nötigen Dateien/Bibliotheken
#include <iostream> // Input und Output durch die cpp Standard Library
#include <wiringPi.h> // Kontrolle über das GPIO (General Purpose Input Output) des Raspberry PIs
#include <softPwm.h> // Zweiter Teil der Raspi Bibliothek
#include <cmath> // Mathematische Hilfsfunktionen aus der cpp Standard Library

#include"rpi_gpio.hpp" // eigene Headerdatei für diese Datei

#define MAX_PWM 100 // Definiert den maximalen Wert, den die PWM Pins annehmen können

// Konstruktor der RpiGpio Klasse
RpiGpio::RpiGpio() {
	if (wiringPiSetupGpio() != 0) std::cout << "[rpi] error initializing raspi" << std::endl; // Initialisiere die wiringPi Bibliothek und überprüfe diesen Vorgang auf Fehler

	// Setze den Modus der Motor Pins
	softPwmCreate(IN1A, 0, MAX_PWM);
	softPwmCreate(IN2A, 0, MAX_PWM);
	softPwmCreate(IN1B, 0, MAX_PWM);
	softPwmCreate(IN2B, 0, MAX_PWM);

	std::cout << "[rpi] init finished" << std::endl;
}

// Methode, die die PWM Pins einer Geschwindigkeit anpasst
void RpiGpio::update_pwm_pins(float l_speed, float r_speed) {
	if (r_speed > 0) { // Setze die Pins für die angeschlossene H-Brücke
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

// Methode, die die Motoren stoppt
void RpiGpio::reset_motors () {
	update_pwm_pins(0, 0);
	std::cout << "[rpi] reset motors" << std::endl;
}
