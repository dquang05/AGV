#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// CPU core definitions
#define JOYSTICK_TASK_CORE 0
#define BLE_TASK_CORE 1
// Joystick pin definitions
#define JOYSTICK_X_PIN GPIO_NUM_2
#define JOYSTICK_Y_PIN GPIO_NUM_4

#define SERVICE_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define CHARACTERISTIC_UUID "a15b89c6-1042-4c05-af06-52bb41e51c1e"
#define SERVER_NAME "Quadrup"

// Handle
TaskHandle_t joystickTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;

// Tyoedef for joystick data
struct JoystickData
{
    int v;
    int w;
};

// Setup BLE client and characteristic
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
bool deviceConnected = false;
static BLEAddress *pServerAddress = nullptr;
QueueHandle_t joystickQueue;

// Callback for client connection events
class MyClientCallbacks : public BLEClientCallbacks
{
    void onConnect(BLEClient *pClient)
    {
        deviceConnected = true;
        Serial.println("Connected to server");
    }
    void onDisconnect(BLEClient *pClient)
    {
        deviceConnected = false;
        BLEDevice::startAdvertising(); // Restart advertising
        Serial.println("Disconnected from server");
    }
};

// Callback for advertised devices during scan
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        Serial.printf("Found device: %s\n", advertisedDevice.getName().c_str());
        if (advertisedDevice.getName() == SERVER_NAME)
        {
            Serial.println("Found Servo server");
            pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            BLEDevice::getScan()->stop();
        }
    }
};

// Callback for characteristic notifications
void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                    uint8_t *pData, size_t length, bool isNotify)
{
    // Check if data length is greater than 0
    if (length > 0)
    {
        // Allocate buffer to copy notification data
        uint8_t *notifyData = new uint8_t[length];
        memcpy(notifyData, pData, length);

        // Print notification length
        Serial.print("Notify received, length: ");
        Serial.println(length);

        // Print raw data in hex for debugging
        Serial.print("Raw data: ");
        for (size_t i = 0; i < length; i++)
        {
            Serial.printf("%02X ", notifyData[i]);
        }
        Serial.println();

        // Free allocated memory
        delete[] notifyData;
    }
    else
    {
        Serial.println("Empty notification received");
    }
}

// Function to connect to the server
bool connectToServer()
{
    // Check if server address is found
    if (pServerAddress == nullptr)
    {
        Serial.println("Server address not found");
        return false;
    }

    // Print connection attempt
    Serial.print("Connecting to ");
    Serial.println(pServerAddress->toString().c_str());

    // Create BLE client
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks());

    // Attempt to connect
    if (!pClient->connect(*pServerAddress))
    {
        Serial.println("Connection failed");
        return false;
    }

    // Request larger MTU
    pClient->setMTU(100);
    Serial.println("Requested MTU: 100");

    // Get service
    BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr)
    {
        Serial.println("Failed to find service UUID");
        pClient->disconnect();
        return false;
    }

    // Get characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr)
    {
        Serial.println("Failed to find characteristic UUID");
        pClient->disconnect();
        return false;
    }

    // Register for notifications if available
    if (pRemoteCharacteristic->canNotify())
    {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        Serial.println("Registered for notifications");
    }

    return true;
}

// Converting joystick data to motor commands
int convertJoystickToVW(int x, int y, int &v_out, int &w_out)
{
    // Normalize joystick values to -1.0 to 1.0 range
    float normX = (x - 2048) / 2048.0;
    float normY = (y - 2048) / 2048.0;

    // Constrain
    normX = constrain(normX, -1.0, 1.0);
    normY = constrain(normY, -1.0, 1.0);

    // Calculate v and w based on normalized values
    float v = normY * 80;
    float w = normX * 80;

    // Round and typecast to int
    v_out = (int)round(v);
    w_out = (int)round(w);

    return 0; // Success
}

// Task to read joystick data
void joystickTask(void *parameter)
{
    // Print task core
    Serial.print("Joystick Task running on core: ");
    Serial.println(xPortGetCoreID());

    while (1)
    {
        // Read analog values from joystick
        int xRaw = analogRead(JOYSTICK_X_PIN); // X axis for v
        int yRaw = analogRead(JOYSTICK_Y_PIN); // Y axis for w

        // Print raw values for debugging
        Serial.printf("Joystick: X=%d, Y=%d\n", xRaw, yRaw);

        // Placeholder for custom algorithm to calculate v and w
        JoystickData data;
        convertJoystickToVW(xRaw, yRaw, data.v, data.w);

        // Print calculated v, w for debugging
        Serial.printf("Calculated: v=%d, w=%d\n", data.v, data.w);

        // Send data to queue
        if (joystickQueue != nullptr)
        {
            xQueueSend(joystickQueue, &data, portMAX_DELAY);
        }

        // Delay 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task to handle BLE communication
void bleTask(void *parameter)
{
    // Print task core
    Serial.print("BLE Task running on core: ");
    Serial.println(xPortGetCoreID());

    // Initialize BLE
    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);

    while (1)
    {
        if (deviceConnected)
        {
            // Receive data from queue
            JoystickData data;
            if (xQueueReceive(joystickQueue, &data, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                // Prepare 4-byte data packet
                uint8_t bleData[4] = {0x00, (uint8_t)(data.v + 80), (uint8_t)(data.w + 80), 0x00};
                pRemoteCharacteristic->writeValue(bleData, 4);
                // Print sent data for debugging
                Serial.printf("Sent data: %02X%02X%02X%02X\n", bleData[0], bleData[1], bleData[2], bleData[3]);
            }

            // Delay 100ms
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            if (pServerAddress == nullptr)
            {
                // Start scanning for server
                Serial.println("Scanning for server...");
                pBLEScan->start(5, false);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            else if (connectToServer())
            {
                // Connection successful
                Serial.println("Connected to server");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else
            {
                // Connection failed, reset address and scan again
                Serial.println("Connection failed, scanning again...");
                pServerAddress = nullptr;
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting BLE Client");

    // Configure GPIO pins for joystick
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);

    // Create queue for joystick data
    joystickQueue = xQueueCreate(10, sizeof(JoystickData));
    if (joystickQueue == nullptr)
    {
        Serial.println("Failed to create joystick queue");
        while (1)
            ;
    }

    // Tạo các task
    xTaskCreatePinnedToCore(
        joystickTask,
        "Joystick Task",
        2048,
        NULL,
        1,
        &joystickTaskHandle,
        JOYSTICK_TASK_CORE);

    xTaskCreatePinnedToCore(
        bleTask,
        "BLE Task",
        4096,
        NULL,
        2,
        &bleTaskHandle,
        BLE_TASK_CORE);
}

void loop()
{

    vTaskDelay(portMAX_DELAY);
}