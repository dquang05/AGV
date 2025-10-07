#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include <Arduino.h>

// Define UUIDs
#define SERVICE_UUID        "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Core definitions
#define BLE_TASK_CORE 1        // Move BLE to Core 1 to avoid conflict with WiFi/BT core
#define PROCESS_TASK_CORE 0    // Process data on Core 0

// Global variables
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
QueueHandle_t bleQueue;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected, stopping advertising");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected, restarting advertising");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            if (bleQueue != nullptr) {
                xQueueSend(bleQueue, value.c_str(), 0); // Don't block indefinitely
            }
        }
    }
};

// BLE Task for handling data sending
void bleTask(void* parameter) {
    // Configure watchdog for this task
    esp_task_wdt_add(NULL);
    
    Serial.print("BLE Task running on core ");
    Serial.println(xPortGetCoreID());

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms interval
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Reset watchdog timer
        esp_task_wdt_reset();

        if (deviceConnected) {
            static uint8_t data = 0;
            pCharacteristic->setValue(&data, 1);
            pCharacteristic->notify();
            data++;
        }

        // Use vTaskDelayUntil for precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Process received BLE data task
void processDataTask(void* parameter) {
    // Configure watchdog for this task
    esp_task_wdt_add(NULL);
    
    Serial.print("Process Task running on core ");
    Serial.println(xPortGetCoreID());

    char receivedData[4];
    
    while(1) {
        // Reset watchdog timer
        esp_task_wdt_reset();

        if (xQueueReceive(bleQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.print("Received: ");
            Serial.println(receivedData);
        }
        
        // Small delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize watchdog
    esp_task_wdt_init(5, true); // 5 second timeout, panic if timeout
    
    // Create queue for BLE data
    bleQueue = xQueueCreate(10, sizeof(char[20]));
    
    // Initialize BLE
    BLEDevice::init("Quadrup");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );
                      
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    
    // Create FreeRTOS tasks on specific cores
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE Task",
        4096,
        NULL,
        5,              // Higher priority
        NULL,
        BLE_TASK_CORE
    );
    
    xTaskCreatePinnedToCore(
        processDataTask,
        "Process Data",
        4096,
        NULL,
        1,              // Lower priority
        NULL,
        PROCESS_TASK_CORE
    );
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Yield CPU in loop
}