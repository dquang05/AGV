#include <Arduino.h>
#include "pwmgen.h"
#include "esp_intr_alloc.h"
pwm_gen m1;
pwm_gen m2;
// pwm_gen m3;

void setup() {
    Serial.begin(9600);
    m1.begin(1);
    m2.begin(2);

    m1.start_pwm();
    m1.set_duty(100);
    m1.capture_config(0);

    m2.start_pwm();
    m2.set_duty(100);
    m2.capture_config(1);

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

}

void loop() {
    // The PWM signal is generated continuously by the hardware
    Serial.print(signal_count_0);
    Serial.print("-");
    Serial.println(signal_count_1);
    delay(100);
}