#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Core assignments for FreeRTOS tasks
#define LIDAR_TASK_CORE 1  // LIDAR task on core 1
#define BLE_TASK_CORE 0    // BLE task on core 0

// Global variables
TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t lidarTaskHandle = NULL;

uint8_t state_data[202]; // Buffer for BLE LIDAR data (1 + 50*4 + 1 = 202 bytes)
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

// Queues and Mutex
QueueHandle_t bleQueue;
SemaphoreHandle_t lidarDataMutex = NULL;

// Struct for LIDAR data (giả lập)
typedef struct {
  uint16_t angle;
  uint16_t distance;
} coordinate;

// Setup for BLE server callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    isAdvertising = false;
    Serial.println("Device connected, stopping advertising");
    uint16_t mtu = BLEDevice::getMTU();
    Serial.print("MTU negotiated: ");
    Serial.println(mtu);
    if (mtu < 502) {
      Serial.println("Warning: MTU too low for 502-byte packets");
    }
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    isAdvertising = false;
    pServer->startAdvertising();
    Serial.println("Device disconnected, restarting advertising");
  }
};

// Setup for BLE characteristic callbacks
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 5 && bleQueue != nullptr && value[0] == 48 && value[4] == 48) {
      uint8_t buffer[5];
      memcpy(buffer, value.data(), 5);
      xQueueSend(bleQueue, buffer, 0);
    }
  }
};

// BLE Task for handling data sending
void sendTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.print("BLE Send Task running on core ");
  Serial.println(xPortGetCoreID());

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    esp_task_wdt_reset();
    if (deviceConnected && buffer_full) {
      Serial.printf("Sending at %lu ms\n", millis());
      if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state_data[0] = 48;
        int index = 1;
        for (int i = 0; i < 50; i++) {
          memcpy(state_data + index, &angle[i], sizeof(uint16_t));
          index += 2;
          memcpy(state_data + index, &distance[i], sizeof(uint16_t));
          index += 2;
        }
        state_data[201] = 48;
        pCharacteristic->setValue(state_data, 202); // Set the value of the characteristic
        pCharacteristic->notify();
        Serial.println("BLE data sent: 202 bytes");
        xSemaphoreGive(lidarDataMutex);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Process received BLE data task
void recieveTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.print("BLE Receive Task running on core ");
  Serial.println(xPortGetCoreID());

  uint8_t receivedData[5];
  while (1) {
    esp_task_wdt_reset();
    if (xQueueReceive(bleQueue, &receivedData, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.print("Received BLE data: ");
      for (int i = 0; i < 5; i++) {
        Serial.print(receivedData[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Simulate LIDAR data
coordinate simulateLidarData() {
  static float current_angle = 0.0;
  coordinate point;
  
  // Simulate LIDAR data: Random distance between 100 and 5000 mm
  point.angle = current_angle;
  point.distance = random(100, 5000);
  current_angle += 2.88; // Increment angle by 2.88 degrees (360 degrees / 125 points)
  if (current_angle >= 360) {
    current_angle -= 360; // Reset
  }
  
  return point;
}

// Task to simulate LIDAR data
void lidarTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.print("LIDAR Task running on core ");
  Serial.println(xPortGetCoreID());

  while (1) {
    esp_task_wdt_reset();
    coordinate point = simulateLidarData();
    // Serial.print("Simulated LIDAR: angle=");
    // Serial.print(point.angle);
    // Serial.print(", distance=");
    // Serial.println(point.distance);

    if (point.distance > 0 && point.distance < 7000 && point.angle >= 0 && point.angle < 360) {
      if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        angle[buffer_index] = (uint16_t)round(point.angle * 10);
        distance[buffer_index] = (uint16_t)round(point.distance);
        // Serial.print("Buffer index=");
        // Serial.print(buffer_index);
        // Serial.print(", angle=");
        // Serial.print(angle[buffer_index] / 10.0);
        // Serial.print(", distance=");
        // Serial.println(distance[buffer_index]);
        buffer_index++;
        if (buffer_index >= 50) {
          buffer_index = 0;
          buffer_full = true;
          Serial.println("LIDAR buffer full, ready to send");
        }
        xSemaphoreGive(lidarDataMutex);
      }
    } else {
      Serial.println("Invalid simulated LIDAR data, skipping");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Initialize BLE
void BLE_begin() {
  BLEDevice::init("Quadrup");
  BLEDevice::setMTU(517);
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

// Setup function
void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(5, true); // 5-second watchdog with panic

  // Initialize queues and mutex
  bleQueue = xQueueCreate(10, 5 * sizeof(uint8_t));
  lidarDataMutex = xSemaphoreCreateMutex();

  if (bleQueue == NULL) {
    Serial.println("Failed to create BLE queue");
  }
  if (lidarDataMutex == NULL) {
    Serial.println("Failed to create LIDAR data mutex");
  }

  // Initialize BLE
  BLE_begin();
  pService->start();
  pServer->startAdvertising();
  isAdvertising = true;
  Serial.println("BLE advertising started");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sendTask, "Send Task", 4096, NULL, 2, &sendTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 1, &recieveTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(lidarTask, "LIDAR Task", 4096, NULL, 4, &lidarTaskHandle, LIDAR_TASK_CORE);
}

// Main loop
void loop() {
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(10));
}