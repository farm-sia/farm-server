#pragma once //Sorgt dafür, dass diese Datei nur einmal importiert wird

// Definieren der Nummern der Pins, die an die H-Brücke angeschlossen sind
#define IN1A 23
#define IN2A 24

#define IN1B 27
#define IN2B 22

// Deklarieren der Klasse RpiGpio
class RpiGpio {
public:
	// Prototypen der Methoden, die in der rpi_gpio.cpp definiert werden
	RpiGpio();	
	void update_pwm_pins(float l_speed, float r_speed);
	void reset_motors();
};
