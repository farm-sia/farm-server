#pragma once

#define IN1A 23
#define IN2A 24

#define IN1B 27
#define IN2B 22

class RpiGpio {
public:
	RpiGpio();
	
	void update_pwm_pins(float l_speed, float r_speed);
	void reset_motors();
};
