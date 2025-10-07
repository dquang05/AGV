#include <Arduino.h>
#include "pwm_gen.h"

pwm_gen m1;
pwm_gen m2;
pwm_gen m3;

void setup() {
    Serial.begin(9600);
    m1.begin(1);
    m2.begin(2);
    // m3.begin(3);
    m1.start_pwm();
    m2.start_pwm();
    // m1.capture_setting();
    pinMode(2,OUTPUT);
    pinMode(4,OUTPUT);
    // pinMode(15,OUTPUT);
    // pinMode(17,OUTPUT);
    pinMode(23,OUTPUT);
    pinMode(5,OUTPUT);
    digitalWrite(23,HIGH);
    digitalWrite(4,LOW);
    digitalWrite(2,HIGH);
    digitalWrite(5,LOW);

     
    m3.start_pwm();
}

void loop() {
    // The PWM signal is generated continuously by the hardware
    delay(1000);
}