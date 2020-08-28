#include "Arduino.h"
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

float wheel_diameter = (43.38)/1000;	

double  leftSetpoint,  leftInput,  leftOutput;
double rightSetpoint, rightInput, rightOutput;

PID leftPID(&leftInput, &leftOutput, &leftSetpoint, 0.05, 6, 0.009, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, 0.01, 5, 0.01, DIRECT);

void set_pid_parameters() {

	set_motors();

	leftInput  =  RPMLeft;
	rightInput = RPMRight;
	leftSetpoint  =  goal_rpm_left;
	rightSetpoint = goal_rpm_right;

	leftPID.SetMode(AUTOMATIC);
	rightPID.SetMode(AUTOMATIC);
	leftPID.SetOutputLimits(minPID,  maxPID);
	rightPID.SetOutputLimits(minPID, maxPID);
}

void compute_rpm() {
	if(millis() -time_old >= cycle_time) {

		rel_left_count  = left_count()  -  rel_left_count;
		rel_right_count = right_count() - rel_right_count;
		RPMLeft  = 60 * abs(rel_left_count)  / pulses_per_turn * 1000 / (millis() - time_old);
		RPMRight = 60 * abs(rel_right_count) / pulses_per_turn * 1000 / (millis() - time_old);		

		rel_left_count  =  left_count();
		rel_right_count = right_count();

		time_old = millis();
	}
}

void motors_speed_pid(float goal_speed_left, float goal_speed_right) {

    compute_rpm();
  
	goal_rpm_left  = goal_speed_left  * 60/(3.1416 * wheel_diameter);
	goal_rpm_right = goal_speed_right * 60/(3.1416 * wheel_diameter);


	leftInput  =  RPMLeft;
	rightInput = RPMRight;

	leftSetpoint  = abs(goal_rpm_left);
	rightSetpoint = abs(goal_rpm_right);

	leftPID.Compute();
	rightPID.Compute();

	left_pwm  = map(leftOutput,  minPID, maxPID, minPWM, maxPWM); 
	right_pwm = map(rightOutput, minPID, maxPID, minPWM, maxPWM); 
	
	if(goal_speed_left  < 0) leftFoward  = LOW;
    else leftFoward = HIGH;
    
	if(goal_speed_right < 0) rightFoward = LOW;
    else rightFoward = HIGH;

	move_motors(leftFoward, left_pwm, rightFoward, right_pwm);
}
