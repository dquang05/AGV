#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "motor_control.h"
#include "lidar.h"
#include "esp_mac.h"
// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Core assignments for FreeRTOS tasks

#define LIDAR_TASK_CORE 1 // LIDAR task on core 1 to balance load
#define MOTOR_TASK_CORE 1 // Motor task on core 1
#define BLE_TASK_CORE 0   // Move BLE to Core 0 to avoid conflict with WiFi/BT core

// Lidar Pins
#define LIDAR_TX_PIN 10
#define LIDAR_RX_PIN 9

// Global variables
TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL; // Handle for motor task (unused)
TaskHandle_t lidarTaskHandle = NULL; // Handle for LIDAR task

uint8_t state_data[202]; // Buffer for BLE LIDAR data
int v = 0;               // Left wheel speed
int w = 0;               // Right wheel speed
uint16_t angle[50];
uint16_t distance[50];
int buffer_index = 0;
bool buffer_full = false;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

// Flags
bool deviceConnected = false;
bool isAdvertising = false;
bool motor_state = false;

// Queues
QueueHandle_t bleQueue;
SemaphoreHandle_t lidarDataMutex = NULL;    // Mutex for angle and distance
SemaphoreHandle_t motorCommandMutex = NULL; // Mutex for motor commands

// Setup for BLE server callbacks
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    isAdvertising = false; // Stop advertising when connected
    Serial.println("Device connected, stopping advertising");
    uint16_t mtu = BLEDevice::getMTU();
    Serial.print("MTU negotiated: ");
    Serial.println(mtu);
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
    if (value.length() == 4 && bleQueue != nullptr && value[0] == 48 && value[3] == 48)
    { // Check if value is 4 bytes long and starts/ends with 48
      uint8_t buffer[4];
      memcpy(buffer, value.data(), 4); // Copy 4 bytes: [48, w, v, 48]

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
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // time interval
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
        for (int i = 0; i < 50; i++)
        {
          memcpy(state_data + index, &angle[i], sizeof(uint16_t));
          index += 2;
          memcpy(state_data + index, &distance[i], sizeof(uint16_t));
          index += 2;
        }
        state_data[201] = 48;
        pCharacteristic->setValue(state_data, 202); // Set the value of the characteristic
        // Serial.println("Sending LIDAR data over BLE");
        pCharacteristic->notify();
        buffer_full = false; // Reset buffer full flag

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

  uint8_t receivedData[4];

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

      else
      {
        // No data received, continue
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void BLE_begin()
{
  BLEDevice::init("Quadrup");
  BLEDevice::setMTU(517); // Set MTU size to 517 bytes
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
        angle[buffer_index] = (uint16_t)round(point.angle * 10);  // Update angle
        distance[buffer_index] = (uint16_t)round(point.distance); // Update distance

        buffer_index++;
        if (buffer_index >= 50)
        {
          buffer_index = 0;   // Reset index after 50 points
          buffer_full = true; // Set buffer full flag
          // Serial.println("LIDAR buffer full, ready to send");
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

void motorTask(void *parameter)
{

  esp_task_wdt_add(NULL); // Register task with watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
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

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Setup function
void setup()
{
  Serial.begin(115200);                             // Initialize serial communication
  esp_task_wdt_init(5, true);                       // 5-second watchdog with panic
  bleQueue = xQueueCreate(10, 5 * sizeof(uint8_t)); // Queue for BLE data
  lidarDataMutex = xSemaphoreCreateMutex();         // Create mutex for lidar data
  motorCommandMutex = xSemaphoreCreateMutex();      // Create mutex for motor commands

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
}

// Main loop
void loop()
{
  esp_task_wdt_reset(); // Reset watchdog

  vTaskDelay(pdMS_TO_TICKS(10)); // Yield CPU
}