#include "motors_speed.h"

int foward   = HIGH;
int backward = LOW;

int left_pwm  = 0; //30;
int right_pwm = 0; //50;

void setup() { 
	set_motors(); 
} 

void loop() {
	
	move_motors(foward, left_pwm, foward, right_pwm);
	delay(20);
}

