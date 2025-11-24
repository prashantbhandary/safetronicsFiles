#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>  // Important: Add this for the descriptor

BLECharacteristic *sensorChar;

void setup() {
  Serial.begin(115200);

  BLEDevice::init("Helmet01");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService("1234");

  sensorChar = service->createCharacteristic(
    "5678",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  // Add this line - it's crucial for notifications to work!
  sensorChar->addDescriptor(new BLE2902());

  service->start();
  
  // Set initial value
  sensorChar->setValue("0,0,0.0");
  
  BLEDevice::startAdvertising();
  Serial.println("BLE Server started - Helmet01");
}

void loop() {
  int heart = 78;
  int spo2  = 97;
  float temp = 36.7;
  String msg="prashant";

  String payload = String(heart) + "," + String(spo2) + "," + String(temp) + "," + msg;
  sensorChar->setValue(payload.c_str());
  sensorChar->notify();

  // Serial.println("Sent: " + payload);
  delay(1000);
}