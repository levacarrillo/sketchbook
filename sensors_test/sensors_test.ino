#include "sensors.h"


void setup() {

	Serial.begin(9600);
}

void loop() {
	
	read_sensors_data();
	
	Serial.print(left_sharp());
	Serial.print("--");
	Serial.print(right_sharp());
	Serial.print("--");
	Serial.print(ldr0());
	Serial.print("--");
	Serial.print(ldr1());
	Serial.print("--");
	Serial.print(ldr2());
	Serial.print("--");
	Serial.println(ldr3());
	
	delay(200);	
}
