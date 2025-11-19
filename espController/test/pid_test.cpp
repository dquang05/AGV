#include <Arduino.h>
// Test both motors (L298N style). Edit PWM_ENA_PIN / PWM_ENB_PIN to actual EN pins (or -1 to skip PWM)
#define IN_A1 2   // IN1 (motor phải)
#define IN_A2 4   // IN2 (motor phải)
#define IN_B1 5   // IN3 (motor trái)
#define IN_B2 23  // IN4 (motor trái)

#define PWM_ENA_PIN -1  // set to PWM pin for ENA (motor right) or -1 if EN jumper tied HIGH
#define PWM_ENB_PIN -1  // set to PWM pin for ENB (motor left) or -1 if EN jumper tied HIGH

// PWM settings (for ESP32 ledc)
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;
const int PWM_FREQ = 2000;
const int PWM_RES = 8; // 8-bit resolution (0-255)
const int PWM_DUTY = 200; // duty for motion (0-255)

unsigned long wait_ms = 2000; // thời gian cho mỗi trạng thái

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(IN_A1, OUTPUT);
  pinMode(IN_A2, OUTPUT);
  pinMode(IN_B1, OUTPUT);
  pinMode(IN_B2, OUTPUT);

  if (PWM_ENA_PIN != -1) {
    ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_ENA_PIN, PWM_CH_A);
  }
  if (PWM_ENB_PIN != -1) {
    ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_ENB_PIN, PWM_CH_B);
  }

  // ensure all low initially (stopped)
  digitalWrite(IN_A1, LOW);
  digitalWrite(IN_A2, LOW);
  digitalWrite(IN_B1, LOW);
  digitalWrite(IN_B2, LOW);
  if (PWM_ENA_PIN != -1) ledcWrite(PWM_CH_A, 0);
  if (PWM_ENB_PIN != -1) ledcWrite(PWM_CH_B, 0);

  Serial.println("== Motor test start ==");
  delay(500);
}

void logPins(const char* tag) {
  Serial.print("["); Serial.print(tag); Serial.print("] ");
  Serial.print("IN_A1="); Serial.print(digitalRead(IN_A1));
  Serial.print(" IN_A2="); Serial.print(digitalRead(IN_A2));
  Serial.print(" | IN_B1="); Serial.print(digitalRead(IN_B1));
  Serial.print(" IN_B2="); Serial.print(digitalRead(IN_B2));
  if (PWM_ENA_PIN != -1) {
    Serial.print(" | ENA_duty="); Serial.print(ledcRead(PWM_CH_A));
  } else Serial.print(" | ENA=NA");
  if (PWM_ENB_PIN != -1) {
    Serial.print(" ENB_duty="); Serial.print(ledcRead(PWM_CH_B));
  } else Serial.print(" ENB=NA");
  Serial.println();
}

void motorRightForward()  { digitalWrite(IN_A1, HIGH); digitalWrite(IN_A2, LOW); if (PWM_ENA_PIN!=-1) ledcWrite(PWM_CH_A, PWM_DUTY); }
void motorRightBackward() { digitalWrite(IN_A1, LOW);  digitalWrite(IN_A2, HIGH); if (PWM_ENA_PIN!=-1) ledcWrite(PWM_CH_A, PWM_DUTY); }
void motorRightStop()     { digitalWrite(IN_A1, LOW);  digitalWrite(IN_A2, LOW);  if (PWM_ENA_PIN!=-1) ledcWrite(PWM_CH_A, 0); }

void motorLeftForward()   { digitalWrite(IN_B1, HIGH); digitalWrite(IN_B2, LOW);  if (PWM_ENB_PIN!=-1) ledcWrite(PWM_CH_B, PWM_DUTY); }
void motorLeftBackward()  { digitalWrite(IN_B1, LOW);  digitalWrite(IN_B2, HIGH); if (PWM_ENB_PIN!=-1) ledcWrite(PWM_CH_B, PWM_DUTY); }
void motorLeftStop()      { digitalWrite(IN_B1, LOW);  digitalWrite(IN_B2, LOW);  if (PWM_ENB_PIN!=-1) ledcWrite(PWM_CH_B, 0); }

void testMotorRight() {
  Serial.println("--- Test Motor RIGHT (OUT1-OUT2) ---");
  motorRightForward(); logPins("R forward"); delay(wait_ms);
  motorRightStop();    logPins("R stop");    delay(500);
  motorRightBackward();logPins("R backward");delay(wait_ms);
  motorRightStop();    logPins("R stop");    delay(500);
}

void testMotorLeft() {
  Serial.println("--- Test Motor LEFT (OUT3-OUT4) ---");
  motorLeftForward();  logPins("L forward"); delay(wait_ms);
  motorLeftStop();     logPins("L stop");    delay(500);
  motorLeftBackward(); logPins("L backward");delay(wait_ms);
  motorLeftStop();     logPins("L stop");    delay(500);
}

void loop() {
  testMotorRight();
  delay(1000);
  testMotorLeft();
  delay(3000);

  // Additional checks: swap IN pins in software to see if behavior follows pins
  Serial.println(">> Swap IN pins in software (simulate wiring swap) to check driver channel");
  // swap (simulate) right->left mapping and test again
  // we only simulate by toggling both motors to opposite directions together:
  digitalWrite(IN_A1, HIGH); digitalWrite(IN_A2, LOW);
  digitalWrite(IN_B1, LOW);  digitalWrite(IN_B2, HIGH);
  if (PWM_ENA_PIN != -1) ledcWrite(PWM_CH_A, PWM_DUTY);
  if (PWM_ENB_PIN != -1) ledcWrite(PWM_CH_B, PWM_DUTY);
  logPins("Sim mixed"); delay(wait_ms);
  motorRightStop(); motorLeftStop();
  delay(500);

  Serial.println("=== One loop done. Resetting for next loop ===");
  delay(2000);
}
