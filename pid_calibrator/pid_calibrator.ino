#include "PID_v1.h"
#include "encoders.h"
#include "motors_speed.h"


#define maxPID 175
#define minPID 0
#define maxPWM 255
#define minPWM 0

int leftFoward  = HIGH;
int rightFoward = HIGH;

int leftBackward  = LOW;
int rightBackward = LOW; 

int left_pwm  = 0;
int right_pwm = 0;

long time_old = 0;
float cycle_time = 40;

int pwmR = 0;
int pwmL = 0;

double RPMRight = 0.0;
double RPMLeft = 0.0;

volatile long rel_right_count = 0;
volatile long  rel_left_count = 0;

unsigned int pulses_per_turn = 600;

double goal_rpm_right = 0.0;
double goal_rpm_left  = 0.0;

float goal_speed_right = 0.;
float goal_speed_left  = 0.;

float wheel_diameter = (43.38)/1000;	

double  leftSetpoint,  leftInput,  leftOutput;
double rightSetpoint, rightInput, rightOutput;

PID leftPID(&leftInput, &leftOutput, &leftSetpoint, 0.05, 6, 0.009, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, 0.01, 5, 0.01, DIRECT);



void set_pid_parameters() {

	Serial.begin(9600);

	leftInput = RPMLeft;
	leftSetpoint = goal_rpm_left;
	leftPID.SetMode(AUTOMATIC);
	leftPID.SetOutputLimits(minPID, maxPID);

	rightInput = RPMRight;
	rightSetpoint = goal_rpm_right;
	rightPID.SetMode(AUTOMATIC);
	rightPID.SetOutputLimits(minPID, maxPID);
}

void compute_rpm() {
	if(millis() -time_old >= cycle_time) {
		rel_left_count = left_count() - rel_left_count;
		RPMLeft = 60 * abs(rel_left_count) / pulses_per_turn * 1000 / (millis() - time_old);
		rel_left_count = left_count();


		rel_right_count = right_count() - rel_right_count;
		RPMRight = 60 * abs(rel_right_count) / pulses_per_turn * 1000 / (millis() - time_old);
		rel_right_count = right_count();


		time_old = millis();
	}
}

void pid() {

  compute_rpm();
  
	goal_rpm_left  = goal_speed_left * 60/(3.1416 * wheel_diameter);
	goal_rpm_right = goal_speed_right * 60/(3.1416 * wheel_diameter);


	leftInput = RPMLeft;
	leftSetpoint = abs(goal_rpm_left);
	leftPID.Compute();

	rightInput = RPMRight;
	rightSetpoint = abs(goal_rpm_right);
	rightPID.Compute();

  /*Serial.print("leftInput");
  Serial.print(" ");
  Serial.print(leftInput);
  Serial.print(" ");
  Serial.print("setPoint");
  Serial.print(" ");
  Serial.println(leftSetpoint);//*/


	left_pwm = map(leftOutput, minPID, maxPID, minPWM, maxPWM); 
/*
	Serial.print("rightInput");
  Serial.print(" ");
	Serial.print(rightInput);
  Serial.print(" ");
  Serial.print("setPoint");
	Serial.print(" ");
	Serial.println(rightSetpoint);//*/

	right_pwm = map(rightOutput, minPID, maxPID, minPWM, maxPWM); 
	
	if(goal_speed_left  < 0) leftFoward  = LOW;
	if(goal_speed_right < 0) rightFoward = LOW;

	move_motors(leftFoward, left_pwm, rightFoward, right_pwm);
}


void setup() { 
  
	set_motors(); 
	set_encoders_input();
	set_pid_parameters();
} 

void loop() {
  
	pid();
  
	delay(20);
}
