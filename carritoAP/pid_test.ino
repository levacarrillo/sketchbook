#include "motors_speed_pid.h"
#include "encoders.h"

void setup() { 
  
	set_encoders_input();
	set_pid_parameters();
} 

void loop() {
  
	motors_speed_pid(0.0, 0.0);
  
	delay(20);
}
