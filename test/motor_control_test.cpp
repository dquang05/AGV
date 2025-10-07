#include <Arduino.h>
#include "motor_control.h"

void setup() {
    Serial.begin(9600);
    motor_begin();
    motor_speed_set(0,0);
    motor_run(true);
    
}

void loop() {
    // speed_control();
    Serial.print(cur_speed1);
    Serial.print("--");
    Serial.print(cur_speed2);
    Serial.print("--");
    Serial.print(output1);
    Serial.print("--");
    Serial.print(output2);
    Serial.print("--");
    Serial.print(error1);
    Serial.print("--");
    Serial.print(error2);
    Serial.println();
    delay(50);
}