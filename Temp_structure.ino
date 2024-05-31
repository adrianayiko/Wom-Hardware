#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int flexPin = 36; // ADC1_CH0 on ESP32 DevKit v1

const float VCC = 3.3; // voltage at ESP32 3.3V line
const float R_DIV = 47000.0; // resistor used to create a voltage divider
const float flatResistance = 215000.0; // resistance when flat
const float bendResistance = 100000.0; // resistance at 90 deg

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicFlex = NULL; // For Flex Sensor Angle
BLECharacteristic* pCharacteristicMpu = NULL; // For MPU6050 Values

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// Define UUIDs for the two characteristics
#define FLEX_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // This remains the same for the service
#define FLEX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // New UUID for Flex Sensor Angle
#define MPU_SERVICE_UUID         "12345678-1234-5678-1234-56789abcdef0" // New UUID for MPU6050 Values
#define MPU_CHARACTERISTIC_UUID  "87654321-4321-5678-4321-210987654321" // New UUID for MPU6050 Values

Adafruit_MPU6050 mpu;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
 Serial.begin(115200);
 pinMode(flexPin, INPUT);

 // Initialize MPU6050 sensor
 if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
 }
 Serial.println("MPU6050 Found!");

 // Setup motion detection
 mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
 mpu.setMotionDetectionThreshold(1);
 mpu.setMotionDetectionDuration(20);
 mpu.setInterruptPinLatch(true); // Keep it latched. Will turn off when reinitialized.
 mpu.setInterruptPinPolarity(true);
 mpu.setMotionInterrupt(true);

 // Create the BLE Device
 BLEDevice::init("ESP32");

 // Create the BLE Server
 pServer = BLEDevice::createServer();
 pServer->setCallbacks(new MyServerCallbacks());

 // Create the BLE Services
 BLEService *pFlexService = pServer->createService(FLEX_SERVICE_UUID);
 BLEService *pMpuService = pServer->createService(MPU_SERVICE_UUID);

 // Create BLE Characteristics
 pCharacteristicFlex = pFlexService->createCharacteristic(
                      FLEX_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 pCharacteristicMpu = pMpuService->createCharacteristic(
                      MPU_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

 // Add descriptors
 pCharacteristicFlex->addDescriptor(new BLE2902());
 pCharacteristicMpu->addDescriptor(new BLE2902());

 // Start the services
 pFlexService->start();
 pMpuService->start();

 // Start advertising
 BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
 pAdvertising->addServiceUUID(FLEX_SERVICE_UUID);
 pAdvertising->addServiceUUID(MPU_SERVICE_UUID);
 pAdvertising->setScanResponse(false);
 pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
 BLEDevice::startAdvertising();
 Serial.println("Waiting for a client connection to notify...");
}

void loop() {
    // Read flex sensor data
    int ADCflex = analogRead(flexPin);
    float Vflex = ADCflex * VCC / 4095.0; // ESP32 has a 12-bit ADC, so the maximum value is 4095
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    float angle = map(Rflex, flatResistance, bendResistance, 0, 90.0);

    // Read sensor data
    if(mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Format sensor data as a string
        String sensorData = "X:" + String(a.acceleration.x) +
                            ",Y:" + String(a.acceleration.y) +
                            ",Z:" + String(a.acceleration.z) +
                            ",GyroX:" + String(g.gyro.x) +
                            ",GyroY:" + String(g.gyro.y) +
                            ",GyroZ:" + String(g.gyro.z) +
                            ",FlexAngle:" + String(angle); // Include flex angle in the data

        // Convert Arduino String to std::string
        std::string message = sensorData.c_str();

        // Notify changed value for Flex Sensor Angle
        if (deviceConnected) {
            pCharacteristicFlex->setValue(String(angle).c_str()); // Send only the angle
            pCharacteristicFlex->notify();
            value++;
            delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        }

        // Notify changed value for MPU6050 Values
        if (deviceConnected) {
            pCharacteristicMpu->setValue(message); // Send all sensor data
            pCharacteristicMpu->notify();
            value++;
            delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        }
    }

    // Disconnecting logic remains the same
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // Connecting logic remains the same
    if (deviceConnected &&!oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    delay(500);
}
