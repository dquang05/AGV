#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08
#define SDA_PIN 18  // GPIO21 for SDA
#define SCL_PIN 19  // GPIO22 for SCL

// Channels for ADC
#define ADC_LEFT 16  
#define ADC_RIGHT 17  

// Delay func
bool nonBlockingDelay(unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}


void requestEvent() {
  // Read analog values from ADC channels
  uint16_t leftValue = analogRead(ADC_LEFT);   
  uint16_t rightValue = analogRead(ADC_RIGHT);  

  // For debugging
  Serial.print("Left: "); Serial.print(leftValue);
  Serial.print(" | Right: "); Serial.println(rightValue);

  // Data (2 uint16_t, 4 byte)
  uint8_t buffer[4];
  buffer[0] = (leftValue >> 8) & 0xFF; // High Byte 
  buffer[1] = leftValue & 0xFF;        // Low Byte 
  buffer[2] = (rightValue >> 8) & 0xFF;
  buffer[3] = rightValue & 0xFF;


  // Send data to master
  Wire.write(buffer, 4);
}

void setup() {
   Serial.begin(115200);

  // Init I2C Slave (SDA=GPIO21, SCL=GPIO22)
  //Wire.begin(SLAVE_ADDRESS, SDA_PIN, SCL_PIN);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent); // Register request event handler

pinMode(ADC_LEFT, INPUT);   // Set ADC_LEFT pin as input
pinMode(ADC_RIGHT, INPUT);  // Set ADC_RIGHT pin as input
}

void loop() {
  if(nonBlockingDelay(1000)) { 
    //Just for debugging, print a message every second
  }
}
