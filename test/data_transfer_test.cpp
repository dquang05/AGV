#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

#define BLE_TASK_CORE 0
#define LIDAR_TASK_CORE 1

TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t lidarTaskHandle = NULL;

uint8_t state_data[82];
float angle[10];
float distance[10];
int buffer_index = 0;
bool buffer_full = false;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

bool deviceConnected = false;
bool isAdvertising = false;
SemaphoreHandle_t lidarDataMutex = NULL;
QueueHandle_t bleQueue;

// Mock LIDAR coordinate struct
struct coordinate {
  float angle;
  float distance;
};

// Mock function to simulate LIDAR data
coordinate mock_coordinate_take() {
  coordinate point;
  point.angle = random(0, 360);
  point.distance = random(10, 5000);
  return point;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    isAdvertising = false;
    Serial.println("Device connected, stopping advertising");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    isAdvertising = false;
    pServer->startAdvertising();
    Serial.println("Device disconnected, restarting advertising");
  }
};

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

void sendTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.print("BLE Task running on core ");
  Serial.println(xPortGetCoreID());

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);

  while (1) {
    esp_task_wdt_reset();
    if (deviceConnected && buffer_full) {
      if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state_data[0] = 48;
        int index = 1;
        for (int i = 0; i < 10; i++) {
          memcpy(state_data + index, &angle[i], sizeof(float));
          index += 4;
          memcpy(state_data + index, &distance[i], sizeof(float));
          index += 4;
        }
        state_data[81] = 48;
        pCharacteristic->setValue(state_data, 82);
        pCharacteristic->notify();
        buffer_full = false; // Reset buffer full flag after sending
        xSemaphoreGive(lidarDataMutex);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void recieveTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.print("Receive Task running on core ");
  Serial.println(xPortGetCoreID());
  uint8_t receivedData[5];
  while (1) {
    esp_task_wdt_reset();
    if (xQueueReceive(bleQueue, &receivedData, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Process received data
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void lidarTask(void *parameter) {
  esp_task_wdt_add(NULL);
  while (1) {
    esp_task_wdt_reset();
    coordinate point = mock_coordinate_take();
    if (xSemaphoreTake(lidarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      angle[buffer_index] = point.angle;
      distance[buffer_index] = point.distance;
      buffer_index++;
      if (buffer_index >= 10) {
        buffer_index = 0;
        buffer_full = true;
      }
      xSemaphoreGive(lidarDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void BLE_begin() {
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

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  esp_task_wdt_init(5, true);
  bleQueue = xQueueCreate(10, 5 * sizeof(uint8_t));
  lidarDataMutex = xSemaphoreCreateMutex();

  BLE_begin();
  pService->start();
  pServer->startAdvertising();
  isAdvertising = true;
  Serial.println("BLE advertising started");

  xTaskCreatePinnedToCore(sendTask, "Send Task", 4096, NULL, 2, &sendTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 1, &recieveTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(lidarTask, "Lidar Task", 4096, NULL, 1, &lidarTaskHandle, LIDAR_TASK_CORE);
}

void loop() {
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(10));
}
