#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Flex sensor
const int flexPin = A0;
float minAnalog = 0;
float maxAnalog = 4095;
float minAngle = 0;
float maxAngle = 90;

// MPU-6050
Adafruit_MPU6050 mpu;
float gyro_scale = 131.0; // Sensitivity for gyroscope (degrees/s per LSB)

// Bluetooth
BLEServer* pServer;
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.begin();

    // Initialize Bluetooth
    BLEDevice::init("ESP32 Flex and Motion");
    BLEServer* pServer = BLEDevice::createServer();
    BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180D));
    pCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)0x2A37),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pService->start();
    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}

void loop() {
    // Read flex sensor value
    int flexValue = analogRead(flexPin);
    float flexAngle = minAngle + (maxAngle - minAngle) * (flexValue - minAnalog) / (maxAnalog - minAnalog);

    // Read accelerometer data
    sensors_event_t accel;
    mpu.getAcceleration(&accel);
    float accel_x = accel.acceleration.x;
    float accel_y = accel.acceleration.y;
    float accel_z = accel.acceleration.z;

    // Read gyroscope data
    sensors_event_t gyro;
    mpu.getGyro(&gyro);
    float gyro_x = gyro.gyro.x / gyro_scale;
    float gyro_y = gyro.gyro.y / gyro_scale;
    float gyro_z = gyro.gyro.z / gyro_scale;

    // Calculate walking stability (customize this part)
    float stability = calculateStability(accel_x, accel_y, accel_z);

    // Send data to phone via Bluetooth
    if (deviceConnected) {
        String dataToSend = "Flex Angle: " + String(flexAngle) + " deg\n";
        dataToSend += "Gyro X: " + String(gyro_x) + " deg/s\n";
        dataToSend += "Gyro Y: " + String(gyro_y) + " deg/s\n";
        dataToSend += "Gyro Z: " + String(gyro_z) + " deg/s\n";
        dataToSend += "Stability: " + String(stability) + "\n";
        pCharacteristic->setValue(dataToSend.c_str());
        pCharacteristic->notify();
    }

    delay(1000); // Delay for 1 second
}

float calculateStability(float ax, float ay, float az) {
    // Implement your stability calculation logic here
    // For example, you can use the magnitude of acceleration
    return sqrt(ax * ax + ay * ay + az * az);
}

// Bluetooth connection handling
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setupBluetooth() {
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    pServer->setCallbacks(new MyServerCallbacks());
}

