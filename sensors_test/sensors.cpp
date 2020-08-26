#include "Arduino.h"

int  leftSharp;
int rightSharp;
int ldr_0, ldr_1, ldr_2, ldr_3;

void read_sensors_data() {

    leftSharp  = analogRead(0);
    rightSharp = analogRead(1);
    
    ldr_0 = analogRead(2);
    ldr_1 = analogRead(3);
    ldr_2 = analogRead(4);
    ldr_3 = analogRead(5);
}

int left_sharp() {
    return leftSharp;
}

int right_sharp() {
    return rightSharp;
}

int ldr0() {
    return ldr_0;
}

int ldr1() {
    return ldr_1;
}
int ldr2() {
    return ldr_2;
}
int ldr3() {
    return ldr_3;
}
