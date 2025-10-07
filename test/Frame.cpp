#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "motor_control.h"
#include "lidar.h"
#define SENSOR_THRESHOLD 500 // Threshold for line-following sensors
// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Core assignments for FreeRTOS tasks

#define LIDAR_TASK_CORE 1  // LIDAR task on core 1 to balance load
#define SENSOR_TASK_CORE 1 // Sensor task on core 1 with I2C
#define MOTOR_TASK_CORE 1  // Motor task on core 1
#define BLE_TASK_CORE 0    // Move BLE to Core 0 to avoid conflict with WiFi/BT core

// I2C slave address for line-following sensors
#define SLAVE_ADDRESS 0x08 // Matches slave Arduino at 0x08

// Lidar Pins
#define LIDAR_TX_PIN 10
#define LIDAR_RX_PIN 9

// Global variables
TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;  // Handle for motor task (unused)
TaskHandle_t lidarTaskHandle = NULL;  // Handle for LIDAR task
TaskHandle_t sensorTaskHandle = NULL; // Handle for sensor task

uint8_t state_data[82]; // Buffer for BLE LIDAR data
int v = 0;              // Left wheel speed
int w = 0;              // Right wheel speed
float angle[10];
float distance[10];
int buffer_index = 0;
bool buffer_full = false;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

// Flags
bool deviceConnected = false;
bool isAdvertising = false;
bool i2cHandle = false;
bool motor_state = false;

// Queues
QueueHandle_t bleQueue;
QueueHandle_t sensorQueue = NULL;
SemaphoreHandle_t lidarDataMutex = NULL;    // Mutex for angle and distance
SemaphoreHandle_t motorCommandMutex = NULL; // Mutex for motor commands

typedef struct
{
  int leftSensorValue;
  int rightSensorValue;
} sensor_data_t;

// Setup for BLE server callbacks
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    isAdvertising = false; // Stop advertising when connected
    Serial.println("Device connected, stopping advertising");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    isAdvertising = false;       // Stop advertising when disconnected
    pServer->startAdvertising(); // Restart advertising
    Serial.println("Device disconnected, restarting advertising");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 5 && bleQueue != nullptr && value[0] == 48 && value[4] == 48)
    { // Check if value is 5 bytes long and starts/ends with 48
      uint8_t buffer[5];
      memcpy(buffer, value.data(), 5); // Copy 5 bytes: [48, w, v, i2c_state, 48]

      xQueueSend(bleQueue, buffer, 0);
    }
  }
};

// BLE Task for handling data sending
void sendTask(void *parameter)
{
  // Configure watchdog for this task
  esp_task_wdt_add(NULL);

  Serial.print("BLE Task running on core ");
  Serial.println(xPortGetCoreID());

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // time interval
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  { // Reset watchdog timer
    esp_task_wdt_reset();

    if (deviceConnected == true && buffer_full)
    {
      if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        state_data[0] = 48;
        int index = 1;
        for (int i = 0; i < 10; i++)
        {
          memcpy(state_data + index, &angle[i], sizeof(float));
          index += 4;
          memcpy(state_data + index, &distance[i], sizeof(float));
          index += 4;
        }
        state_data[81] = 48;
        pCharacteristic->setValue(state_data, 82);
        pCharacteristic->notify();
        // Serial.println("Sent LIDAR data via BLE notify");
        // for(int i = 0; i < 82; i++)
        // {
        //   Serial.print(state_data[i]);
        //   Serial.print(" ");
        // }
        xSemaphoreGive(lidarDataMutex);
      }
    }
    // Use vTaskDelayUntil for precise timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Process received BLE data task
void recieveTask(void *parameter)
{
  // Configure watchdog for this task
  esp_task_wdt_add(NULL);

  Serial.print("Process Task running on core ");
  Serial.println(xPortGetCoreID());

  uint8_t receivedData[5];

  while (1)
  {
    // Reset watchdog timer
    esp_task_wdt_reset();
    if (xQueueReceive(bleQueue, &receivedData, pdMS_TO_TICKS(50)) == pdTRUE)
    {

      // Protect motor command variables with mutex
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        v = (int)(receivedData[1] - 80);
        w = (int)(receivedData[2] - 80);
        xSemaphoreGive(motorCommandMutex);
      }
      bool newSensorState = receivedData[3];
      if (newSensorState != i2cHandle)
      {
        i2cHandle = newSensorState;
        if (i2cHandle)
        {
          if (sensorTaskHandle != NULL)
            vTaskResume(sensorTaskHandle);
        }
        else
        {
          if (sensorTaskHandle != NULL)
            vTaskSuspend(sensorTaskHandle);
        }
      }
    }
    else
    {
      // No data received, continue
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void BLE_begin()
{
  BLEDevice::init("Quadrup");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
}

// Task to read LIDAR data and detect obstacles
void lidarTask(void *parameter)
{
  esp_task_wdt_add(NULL); // Register task with watchdog

  int Obstacle = 0; // Initialize obstacle counter

  while (1)
  {
    esp_task_wdt_reset(); // Reset watchdog
    coordinate point = rlidar.coordinate_take();
    if (point.distance > 0 && point.distance < 7000 && point.angle >= 0 && point.angle < 360)
    {
      // Protect shared variables with mutex
      if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        angle[buffer_index] = point.angle;       // Update angle
        distance[buffer_index] = point.distance; // Update distance
        buffer_index++;
        if (buffer_index >= 10)
        {
          buffer_index = 0;   // Reset index after 10 points
          buffer_full = true; // Set buffer full flag
          // Serial.println("Buffer full, ready to send LIDAR data");
        }
        xSemaphoreGive(lidarDataMutex);
      }

      if (point.distance < 50 && (point.angle < 45 || point.angle > 315))
      {
        Obstacle++;

        if (Obstacle > 10)
        {
          Serial.println("Obstacle detected, stopping LIDAR motor.");
          if (motor_state && motorTaskHandle != NULL)
          { // Notify motor task to stop
            xTaskNotifyGive(motorTaskHandle);
          }
        }
      }
      else
      {
        Obstacle = 0; // Reset counter if no obstacle detected
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // delay for LIDAR reading
  }
}

// Task to read line-following sensor data via I2C
void sensorTask(void *parameter)
{
  esp_task_wdt_add(NULL);   // Register task with watchdog
  int leftSensorValue = 0;  // Initialize left sensor value
  int rightSensorValue = 0; // Initialize right sensor value
  uint8_t buffer[4];        // Buffer for I2C data
  while (1)
  {
    esp_task_wdt_reset(); // Reset watchdog
    if (i2cHandle == false)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue; // Skip I2C reading if not enabled
    }
    else
    {
      Wire.requestFrom(SLAVE_ADDRESS, 4); // Request 4 bytes from slave
      if (Wire.available() == 4)
      {
        for (int i = 0; i < 4; i++)
        {
          buffer[i] = Wire.read();
        }
        leftSensorValue = (buffer[0] << 8) | buffer[1];  // Combine bytes for left sensor
        rightSensorValue = (buffer[2] << 8) | buffer[3]; // Combine bytes for right sensor
        sensor_data_t data = {leftSensorValue, rightSensorValue};
        if (xQueueSend(sensorQueue, &data, 0) != pdTRUE)
        {
          // Failed to send data to queue, handle error if needed
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
    }
  }
}

void motorTask(void *parameter)
{

  esp_task_wdt_add(NULL); // Register task with watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  sensor_data_t data;
  while (1)
  {
    esp_task_wdt_reset();

    if (ulTaskNotifyTake(pdTRUE, 0))
    {
      // Stop motors if notified by LIDAR task
      motor_speed_set(0, 0);
      motor_run(false);
      // Protect motor command variables with mutex
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        v = 0;
        w = 0;
        Serial.println("Motors stopped due to obstacle detection");
        xSemaphoreGive(motorCommandMutex);
      }
      motor_state = false;
    }
    else
    {
      motor_state = true; // Set motor state to running if not notified
      motor_run(true);    // Ensure motors are running
    }
    // Use BLE commands if device is connected
    if (deviceConnected)
    {
      // Protect motor command variables with mutex
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        int local_v = v;
        int local_w = w;
        // Serial.print("v = ");
        // Serial.print(local_v);
        // Serial.print(", w = ");
        // Serial.print(local_w);
        // Serial.println();
        xSemaphoreGive(motorCommandMutex);
        motor_speed_set(local_v, local_w); // Use BLE velocity commands
      }
      else
      {
        Serial.println("motor Stop");
        motor_speed_set(0, 0); // Stop motors if mutex not available
      }
    }
    else if (!deviceConnected && xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      // Process sensor data
      if (data.leftSensorValue < SENSOR_THRESHOLD && data.rightSensorValue < SENSOR_THRESHOLD)
      {
        motor_speed_set(50, 50); // Forward
      }
      else if (data.leftSensorValue < SENSOR_THRESHOLD)
      {
        motor_speed_set(30, 60); // Turn right
      }
      else if (data.rightSensorValue < SENSOR_THRESHOLD)
      {
        motor_speed_set(60, 30); // Turn left
      }
      else
      {
        motor_speed_set(0, 0); // Stop (if both sensors are off the line)
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
// Scan I2C bus to debug connectivity issues
void scanI2C()
{
  Wire.begin();
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("I2C device found at 0x");
      Serial.println(addr, HEX);
    }
  }
}

// Setup function
void setup()
{
  Serial.begin(115200);                                  // Initialize serial communication
  esp_task_wdt_init(5, true);                            // 5-second watchdog with panic
  sensorQueue = xQueueCreate(10, sizeof(sensor_data_t)); // Queue for sensor data
  bleQueue = xQueueCreate(10, 5 * sizeof(uint8_t));      // Queue for BLE data
  lidarDataMutex = xSemaphoreCreateMutex();              // Create mutex for lidar data
  motorCommandMutex = xSemaphoreCreateMutex();           // Create mutex for motor commands

  if (sensorQueue == NULL)
  {
    Serial.println("Failed to create sensor queue");
  }

  if (lidarDataMutex == NULL)
  {
    Serial.println("Failed to create lidar data mutex");
  }

  if (motorCommandMutex == NULL)
  {
    Serial.println("Failed to create motor command mutex");
  }

  // Initialize BLE
  BLE_begin();
  pService->start();
  pServer->startAdvertising(); // Bắt đầu quảng bá
  isAdvertising = true;
  Serial.println("BLE advertising started");

  // scanI2C();          // Debug I2C bus
  rlidar.begin();     // Initialize LIDAR
  rlidar.lidar_run(); // Start LIDAR motor

  motor_begin();         // Initialize motors
  motor_speed_set(0, 0); // Set initial motor speed
  motor_run(true);       // Enable motors
  motor_state = true;    // Set motor state to running
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sendTask, "Send Task", 4096, NULL, 2, &sendTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 1, &recieveTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 4096, NULL, 3, &motorTaskHandle, MOTOR_TASK_CORE);
  xTaskCreatePinnedToCore(lidarTask, "LIDAR Task", 4096, NULL, 4, &lidarTaskHandle, LIDAR_TASK_CORE);
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 5, &sensorTaskHandle, SENSOR_TASK_CORE);
}

// Main loop
void loop()
{
  esp_task_wdt_reset(); // Reset watchdog

  vTaskDelay(pdMS_TO_TICKS(10)); // Yield CPU
}