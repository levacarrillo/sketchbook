#include "ros.h"
#include "sensors.h"
#include "encoders.h"
#include "motors_speed_pid.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#define BAUD 200000

ros::NodeHandle nh;

float goal_speed[2];
int sharp_sensor[2];
int light_sensor[4];
long encoder_data[2];

std_msgs::Int32MultiArray encoders_msg;
std_msgs::Int16MultiArray light_sensors_msgs;
std_msgs::Int16MultiArray sharp_sensors_msgs;

void speedMotorsCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher odomPub("/encoders_data", &encoders_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msgs);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msgs);
ros::Subscriber<std_msgs::Float32MultiArray> subSpeedMotors("/speed_motors", speedMotorsCallback);



void publish_odom() {

	encoders_msg.data_length = 2;
	encoder_data[0] =   left_count();
	encoder_data[1] =  right_count();
	encoders_msg.data = encoder_data;

	odomPub.publish(&encoders_msg);
}


void publish_sensors_data() {
	
	read_sensors_data();
	sharp_sensors_msgs.data_length = 2;
	light_sensors_msgs.data_length = 4;

	light_sensor[0] = ldr0();
	light_sensor[1] = ldr1();
	light_sensor[2] = ldr2();
	light_sensor[3] = ldr3();
	sharp_sensor[0] =  left_sharp();
	sharp_sensor[1] = right_sharp();
	
	light_sensors_msgs.data = light_sensor;
	sharp_sensors_msgs.data = sharp_sensor;

	lightSensorsPub.publish(&light_sensors_msgs);
	sharpSensorsPub.publish(&sharp_sensors_msgs);
}


	
void speedMotorsCallback(const std_msgs::Float32MultiArray& msg) {
	
	goal_speed[0] = msg.data[0];
	goal_speed[1] = msg.data[1];
}


void setup() { 
  
	nh.getHardware()->setBaud(BAUD);
	nh.initNode();
	
	nh.advertise(odomPub);
	nh.advertise(lightSensorsPub);
	nh.advertise(sharpSensorsPub);
	nh.subscribe(subSpeedMotors);

	set_encoders_input();
	set_pid_parameters();
} 

void loop() {
  
	
	publish_odom();
	publish_sensors_data();
	motors_speed_pid(goal_speed[0], goal_speed[1]);

	nh.spinOnce();
	delay(20);
}
