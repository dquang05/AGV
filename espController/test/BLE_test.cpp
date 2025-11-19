#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "freertos/stream_buffer.h"

// #include <Arduino.h>
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
#define STREAM_BUF_SIZE 4096 // Buffer size (depends on LIDAR speed and BLE throughput)
#define TRIGGER_LEVEL 1      // unblock when there are at least 1 byte in the buffer

// Global variables
TaskHandle_t sendTaskHandle = NULL;
TaskHandle_t recieveTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;     // Handle for motor task (unused)
TaskHandle_t lidarTaskHandle = NULL;     // Handle for LIDAR task
StreamBufferHandle_t lidarStream = NULL; // Handle for LIDAR stream task

float v = 0; // Linear wheel speed
float w = 0; // Angular wheel speed

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEService *pService = nullptr;

// Flags
bool deviceConnected = false;
bool isAdvertising = false;
bool motor_state = false;

// Queues
QueueHandle_t bleQueue;
SemaphoreHandle_t motorCommandMutex = NULL; // Mutex for motor commands

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
  uint8_t recv_buf[512];

  // Debug variables
  uint32_t pointsSent = 0; // Tổng số điểm đã gửi
  uint32_t bytesSent = 0;  // Tổng số byte đã gửi
  // TickType_t lastDebugTime = xTaskGetTickCount(); // Thời gian debug cuối

  while (1)
  { // Reset watchdog timer
    esp_task_wdt_reset();

    if (deviceConnected == true)
    {
      // Lấy dữ liệu từ stream buffer
      size_t n = xStreamBufferReceive(lidarStream, recv_buf, sizeof(recv_buf), 0);
      if (n > 0)
      {
        // Mỗi điểm LIDAR = 4 bytes (angle + distance)
        uint32_t pointsInBuffer = n / 4;
        pointsSent += pointsInBuffer;
        bytesSent += n;

        size_t maxChunkSize = 200;
        size_t offset = 0;
        while (offset < n)
        {
          size_t chunk = (n - offset > maxChunkSize) ? maxChunkSize : (n - offset);
          pCharacteristic->setValue(recv_buf + offset, chunk);
          pCharacteristic->notify();
          offset += chunk;
        }
      }
    }

    // Debug: In thống kê mỗi giây
    // TickType_t currentTime = xTaskGetTickCount();
    // if ((currentTime - lastDebugTime) >= pdMS_TO_TICKS(1000)) // 1 giây
    // {
    //   Serial.print("\n LIDAR Points/sec: ");
    //   Serial.print(pointsSent);
    //   Serial.print(", Bytes/sec: ");
    //   Serial.print(bytesSent);

    //   // Reset counters
    //   pointsSent = 0;
    //   bytesSent = 0;
    //   lastDebugTime = currentTime;
    // }

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
        v = (receivedData[2] - 80) * 0.1f; // -8.0 to +17.5
        w = (receivedData[1] - 80) * 0.1f;
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
      // Less Characteristics for better compatibility
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
}


// Task to read LIDAR data
void lidarTask(void *parameter)
{
  esp_task_wdt_add(NULL); // Register task with watchdog
  // Check uart speed
  unsigned long lastPrint = millis();
  int pointCount = 0;
  int sentCount = 0;
  int dropCount = 0;

  while (1)
  {
    esp_task_wdt_reset(); // Reset watchdog
    coordinate point = rlidar.coordinate_take();
    // coordinate point = simulateLidar();
    if (point.distance > 0 && point.distance < 7000 &&
        point.angle >= 0 && point.angle < 360)
    {
      // pointCount++;
      // Mỗi điểm: 4 byte {angle(uint16_t), distance(uint16_t)}
      uint16_t ang = (uint16_t)round(point.angle * 10);
      uint16_t dist = (uint16_t)round(point.distance);

      uint8_t pkt[4];
      pkt[0] = ang & 0xFF;
pkt[1] = ang >> 8;
pkt[2] = dist & 0xFF;
pkt[3] = dist >> 8;

      // Gửi vào stream buffer, block nếu buffer đầy
      size_t sent = xStreamBufferSend(lidarStream, pkt, sizeof(pkt), pdMS_TO_TICKS(5));
      // if (sent < sizeof(pkt))
      // {
      //   dropCount++;
      // }
      // if (sent == sizeof(pkt))
      // {
      //   sentCount++; // Count only successfully stored packets
      // }
    }


    vTaskDelay(pdMS_TO_TICKS(1)); // delay for LIDAR reading
  }
}
int total_pwm1 = 0;
int total_pwm2 = 0;
void motorTask(void *parameter)
{

  esp_task_wdt_add(NULL); // Register task with watchdog
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  while (1)
  {
    esp_task_wdt_reset();

    motor_state = true; // Set motor state to running
    motor_run(true);    // Ensure motors are running
    speed_control();
    if (deviceConnected)
    {
      // Protect motor command variables with mutex
      if (xSemaphoreTake(motorCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        float local_v = v;
        float local_w = w;
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
        motor_speed_set(0, 0); // Stop motors if mutex not available
      }
    }
    else
    {
      motor_speed_set(0, 0); // Stop motors if no BLE connection
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Setup function
void setup()
{
  Serial.begin(115200);                                              // Initialize serial communication
  esp_task_wdt_init(5, true);                                        // 5-second watchdog with panic
  bleQueue = xQueueCreate(10, 5 * sizeof(uint8_t));                  // Queue for BLE data
  motorCommandMutex = xSemaphoreCreateMutex();                       // Create mutex for motor commands
  lidarStream = xStreamBufferCreate(STREAM_BUF_SIZE, TRIGGER_LEVEL); // Create stream buffer for LIDAR data

  if (motorCommandMutex == NULL)
  {
    Serial.println("Failed to create motor command mutex");
  }
  if (lidarStream == NULL)
  {
    Serial.println("Failed to create LIDAR stream buffer");
  }
  // Initialize BLE
  BLE_begin();
  pService->start();
  pServer->startAdvertising(); // Bắt đầu quảng bá
  isAdvertising = true;
  Serial.println("BLE advertising started");

  // Initialize LIDAR
  rlidar.begin();
  rlidar.lidar_run();          // Start LIDAR motor

  motor_begin();         // Initialize motors
  motor_speed_set(0, 0); // Set initial motor speed
  motor_run(true);       // Enable motors
  motor_state = true;    // Set motor state to running
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sendTask, "Send Task", 4096, NULL, 5, &sendTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(recieveTask, "Receive Task", 4096, NULL, 1, &recieveTaskHandle, BLE_TASK_CORE);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 4096, NULL, 3, &motorTaskHandle, MOTOR_TASK_CORE);
  xTaskCreatePinnedToCore(lidarTask, "LIDAR Task", 4096, NULL, 4, &lidarTaskHandle, LIDAR_TASK_CORE);
}

// Main loop
void loop()
{
  esp_task_wdt_reset(); // Reset watchdog

  vTaskDelay(pdMS_TO_TICKS(100)); // Yield CPU
}