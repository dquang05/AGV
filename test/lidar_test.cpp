#include <Arduino.h>
#include "lidar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#define STREAM_BUF_SIZE 4096  // Buffer size in bytes

StreamBufferHandle_t lidarStream = NULL;

// Stats variables
int pointCount = 0;
int recvCount = 0;
int dropCount = 0;
unsigned long lastPrint = 0;



coordinate simulateLidar() {
    static float angle = 0.0f;
    coordinate pt;

    // Giả lập góc quét từ 0 -> 360
    pt.angle = angle;
    angle += 1.0f; // mỗi lần tăng 1 độ
    if (angle >= 360.0f) angle = 0.0f;

    // Giả lập khoảng cách dao động hình sin (500 ~ 2000 mm)
    pt.distance = 1000.0f + 500.0f * sin(angle * DEG_TO_RAD);

    return pt;
}

// Task to read LIDAR and push data into buffer
void LidarTask(void *pvParameters) {
    for (;;) {
        // coordinate point = rlidar.coordinate_take();
        coordinate point = simulateLidar();
        if (point.distance > 0 && point.distance < 7000 &&
            point.angle >= 0 && point.angle < 360)
        {
            pointCount++;

            uint16_t ang = (uint16_t)round(point.angle * 10);   // angle * 0.1 deg
            uint16_t dist = (uint16_t)round(point.distance);

            uint8_t pkt[4];
            memcpy(pkt, &ang, 2);
            memcpy(pkt + 2, &dist, 2);

            // Try to push into buffer
            size_t sent = xStreamBufferSend(lidarStream, pkt, sizeof(pkt), 0);
            if (sent < sizeof(pkt)) {
                dropCount++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // small delay
    }
}

// Task to read from buffer and process data
void ProcessTask(void *pvParameters) {
    uint8_t pkt[4];
    for (;;) {
        // Try to receive a packet (blocking up to 20ms)
        size_t recvd = xStreamBufferReceive(lidarStream, pkt, sizeof(pkt), pdMS_TO_TICKS(20));
        if (recvd == sizeof(pkt)) {
            recvCount++;
            // Example: reconstruct angle & distance
            uint16_t ang, dist;
            memcpy(&ang, pkt, 2);
            memcpy(&dist, pkt + 2, 2);

            float angle = ang / 10.0f;
            float distance = (float)dist;

            // You can process or send over BLE/WiFi here instead of printing
        }

        // Print statistics every second
        if (millis() - lastPrint >= 1000) {
            Serial.print("LIDAR points read: ");
            Serial.print(pointCount);
            Serial.print(" | Pushed to buffer: ");
            Serial.print(recvCount);
            Serial.print(" | Dropped: ");
            Serial.println(dropCount);

            pointCount = 0;
            recvCount = 0;
            dropCount = 0;
            lastPrint = millis();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting LIDAR test with FreeRTOS...");

    // Init LIDAR
    rlidar.begin();
    rlidar.lidar_run();

    // Create stream buffer
    lidarStream = xStreamBufferCreate(STREAM_BUF_SIZE, 4);
    if (lidarStream == NULL) {
        Serial.println("Failed to create stream buffer!");
        while (1);
    }

    // Create tasks
    xTaskCreatePinnedToCore(LidarTask, "LidarTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(ProcessTask, "ProcessTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // Empty, everything runs in tasks
}
