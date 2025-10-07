#include <esp_system.h>
#include <Arduino.h>
// Define GPIO pins for testing
const int ledPin = 2; // GPIO2 (often has onboard LED)
const int inputPin = 4; // GPIO4 for input testing
const int adcPin = 34; // GPIO34 for ADC testing

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Starting ESP32 Check ===");

  // Test 1: Program execution
  Serial.println("Test 1: Program is running - OK");

  // Test 2: GPIO Output
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  Serial.println("Test 2: GPIO Output (LED on GPIO2) - OK");

  // Test 3: GPIO Input
  pinMode(inputPin, INPUT_PULLUP);
  Serial.print("Test 3: GPIO Input (GPIO4, state): ");
  Serial.println(digitalRead(inputPin) ? "HIGH" : "LOW");
  Serial.println("Connect GPIO4 to GND and check state again via Serial Monitor.");

  // Test 4: Memory
  Serial.println("Test 4: Memory check...");
  Serial.print("Total heap: ");
  Serial.print(ESP.getHeapSize());
  Serial.println(" bytes");
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("Flash size: ");
  Serial.print(ESP.getFlashChipSize());
  Serial.println(" bytes");

  // Test 5: ADC
  Serial.println("Test 5: ADC on GPIO34...");
  int adcValue = analogRead(adcPin);
  Serial.print("ADC value: ");
  Serial.println(adcValue);
  Serial.println("Vary voltage on GPIO34 (0-3.3V) and check value again.");

  Serial.println("=== Check Completed ===");
}

void loop() {
  // Periodic check
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    Serial.println("ESP32 is still running...");
    lastCheck = millis();
  }
}