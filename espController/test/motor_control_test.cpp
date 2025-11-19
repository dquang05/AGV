#include <Arduino.h>
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "math.h"

// BLE includes
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"

// Global variables for motor control
float v = 0; // Linear velocity
float w = 0; // Angular velocity
bool motor_state = false;
bool deviceConnected = false; // Changed to false initially

// BLE variables
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

// Mutex for motor commands
SemaphoreHandle_t motorCommandMutex = NULL;
SemaphoreHandle_t totalPwmMutex = NULL;


// Motor task handle
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client Connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client Disconnected");
        pServer->startAdvertising(); // Restart advertising
    }
};

// BLE Characteristic callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        if (value.length() == 4 && value[0] == 48 && value[3] == 48) {
            if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Convert BLE data to float (like in test.cpp)
                v = (value[1] - 80) * 0.1f;
                w = (value[2] - 80) * 0.1f;

                xSemaphoreGive(motorCommandMutex);
                
            }
        }
    }
};

// BLE Task - Send total_pwm1/2 to client
void bleTask(void *parameter) {
    esp_task_wdt_add(NULL);
    
    while (1) {
        esp_task_wdt_reset();
        
        if (deviceConnected && pCharacteristic != nullptr) {
            uint8_t txData[9] = {0}; // 9 bytes
            // Take mutex to read totals safely
            if (xSemaphoreTake(totalPwmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                uint32_t local_total1 = (uint32_t)total_count_0;
                uint32_t local_total2 = (uint32_t)total_count_1;
                xSemaphoreGive(totalPwmMutex);
   Serial.printf("BLE TX: PWM1=%lu, PWM2=%lu\n", local_total1, local_total2);
                txData[0] = 0x01;
                // little-endian pack
                txData[1] = (local_total1) & 0xFF;
                txData[2] = (local_total1 >> 8) & 0xFF;
                txData[3] = (local_total1 >> 16) & 0xFF;
                txData[4] = (local_total1 >> 24) & 0xFF;

                txData[5] = (local_total2) & 0xFF;
                txData[6] = (local_total2 >> 8) & 0xFF;
                txData[7] = (local_total2 >> 16) & 0xFF;
                txData[8] = (local_total2 >> 24) & 0xFF;

                pCharacteristic->setValue(txData, 9);
                pCharacteristic->notify();
                if(!deviceConnected){
                    Serial.println("Err: Notify failed, no client connected");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms
    }
}


void setupBLE() {
     BLEDevice::init("Quadrup");
  BLEDevice::setMTU(517); // Set MTU size to 517 bytes
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      // Less Characteristics for better compatibility
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
}



void motorTask(void *parameter)
{
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    
    while (1)
    {
        esp_task_wdt_reset();
        
       
        motor_state = true;
        motor_run(true);
        
        // // Apply motor commands từ autoDriveTask hoặc BLE
        // if (deviceConnected || true) // Always apply commands
        // {
        //     if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        //     {
        //         float local_v = v;
        //         float local_w = w;
                
        //         xSemaphoreGive(motorCommandMutex);
        //         motor_speed_set(local_v, local_w);  
        //     }
        // }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void autoDriveTask(void *parameter) {
    

    vTaskDelay(pdMS_TO_TICKS(5000));

     for (int i = 0; i < 4; i++) {
        move_straight(5000.0, 1);   // đi 500mm
        vTaskDelay(pdMS_TO_TICKS(300));
        // turn_angle(90, 1);     // quay 90 độ
        // vTaskDelay(pdMS_TO_TICKS(300));
    }

    motor_run(false);
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200);
    totalPwmMutex = xSemaphoreCreateMutex();
    // Initialize motor system
    motor_begin();
    motor_run(true);
    
    // Setup BLE
    setupBLE();
    pService->start();
  pServer->startAdvertising();
    // Create mutex for motor commands
    motorCommandMutex = xSemaphoreCreateMutex();
    if (motorCommandMutex == NULL) {
        Serial.println("Failed to create motor command mutex");
        return;
    }
    
    // Create motor task
    xTaskCreatePinnedToCore(
        motorTask,
        "motorTask",
        4096,
        NULL,
        2,
        &motorTaskHandle,
        1
    );
    
    // // Create BLE task
    // xTaskCreatePinnedToCore(
    //     bleTask,
    //     "bleTask",
    //     4096,
    //     NULL,
    //     1,
    //     &bleTaskHandle,

    //     0
    // );
    
    // Create auto drive task
    xTaskCreatePinnedToCore(
        autoDriveTask,
        "autoDriveTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );

}

void loop() {
    esp_task_wdt_reset();

    vTaskDelay(pdMS_TO_TICKS(1000));
}