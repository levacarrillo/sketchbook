#include "ros.h"
#include "sensors.h"
#include "encoders.h"
#include "motors_speed_pid.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#define BAUD 200000

ros::NodeHandle nh;

std_msgs::Int32MultiArray encoders_msg;
std_msgs::Int16MultiArray light_sensors_msgs;
std_msgs::Int16MultiArray sharp_sensors_msgs;

void speedMotorsCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher odomPub("/encoders_data", &encoders_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msgs);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msgs);
ros::Subscriber<std_msgs::Float32MultiArray> subSpeedMotors("/speed_motors", speedMotorsCallback);

void publish_sensors_data() {
	
	read_sensors_data();
	sharp_sensors_msgs.data_length = 2;
	light_sensors_msgs.data_length = 4;

	light_sensors_msgs.data[0] = ldr0();
	light_sensors_msgs.data[1] = ldr1();
	light_sensors_msgs.data[2] = ldr2();
	light_sensors_msgs.data[3] = ldr3();
	sharp_sensors_msgs.data[0] =  left_sharp();
	sharp_sensors_msgs.data[1] = right_sharp();

	lightSensorsPub.publish(&light_sensors_msgs);
	sharpSensorsPub.publish(&sharp_sensors_msgs);
}

void publish_odom() {

	encoders_msg.data_length = 2;
	encoders_msg.data[left_count()];
	encoders_msg.data[right_count()];

	odomPub.publish(&encoders_msg);
}
	
void speedMotorsCallback(const std_msgs::Float32MultiArray& msg) {
	
	motors_speed_pid(msg.data[0], msg.data[1]);
}


void setup() { 
  
	nh.getHardware()->setBaud(BAUD);
	nh.initNode();
	
	nh.advertise(odomPub);
	nh.advertise(lightSensorsPub);
	nh.advertise(sharpSensorsPub);

	set_encoders_input();
	set_pid_parameters();
} 

void loop() {
  
	
	publish_odom();
	publish_sensors_data();

	nh.spinOnce();
	delay(20);
}
