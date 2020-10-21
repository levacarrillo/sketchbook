#include "Arduino.h"

int ldr_0, ldr_1, ldr_2, ldr_3, ldr_4, ldr_5, ldr_6, ldr_7;
int sharp_0, sharp_1, sharp_2, sharp_3, sharp_4, sharp_5, sharp_6, sharp_7;

void read_sensors_data() {

    ldr_0 = analogRead(2);
    ldr_1 = analogRead(3);
    ldr_2 = analogRead(4);
    ldr_3 = analogRead(5);
    ldr_4 = analogRead(0);
    ldr_5 = analogRead(1);
    ldr_6 = analogRead(6);
    ldr_7 = analogRead(7);	

    sharp_0 = analogRead(8);
    sharp_1 = analogRead(9);
    sharp_2 = analogRead(10);
    sharp_3 = analogRead(11);
    sharp_4 = analogRead(12);
    sharp_5 = analogRead(13);
    sharp_6 = analogRead(14);
    sharp_7 = analogRead(15);

}

int ldr0() { return ldr_0; }
int ldr1() { return ldr_1; }
int ldr2() { return ldr_2; }
int ldr3() { return ldr_3; }
int ldr4() { return ldr_4; }
int ldr5() { return ldr_5; }
int ldr6() { return ldr_6; }
int ldr7() { return ldr_7; }

int sharp0() { return sharp_0; }
int sharp1() { return sharp_1; }
int sharp2() { return sharp_2; }
int sharp3() { return sharp_3; }
int sharp4() { return sharp_4; }
int sharp5() { return sharp_5; }
int sharp6() { return sharp_6; }
int sharp7() { return sharp_7; }
