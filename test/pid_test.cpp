#include <Arduino.h>
#include "pid.h"

float desire = 0;
float actual = -10;

void setup() {
    Serial.begin(9600);
    pid_begin(0.1, 1, 1, 1, &desire, &actual,-20,20);
    pid_timer_set(NULL);


}

void loop() {
    actual = 0.01*output;
    Serial.print(actual);
    Serial.print("-");
    Serial.println(output);
    delay(100);
}