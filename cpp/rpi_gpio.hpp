#pragma once

#define IN1A 23
#define IN2A 24

#define IN1B 27
#define IN2B 22

class RpiGpio {
public:
	char direction = 'n';
	float speed = 0;
	
	RpiGpio();
	
	void update_pwm_pins();
	void reset_motors();
};
