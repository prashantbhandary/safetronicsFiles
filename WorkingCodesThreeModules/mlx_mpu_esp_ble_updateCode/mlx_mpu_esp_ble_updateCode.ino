#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>  // Important: Add this for the descriptor

//mpu
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

//temp mlx 90614
#include <Adafruit_MLX90614.h>
BLECharacteristic *sensorChar;
//mpu code 
MPU9250_asukiaaa mySensor;
//mlx 90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define MPU_ADDR 0x68
bool isMPUConnected() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}
bool impactDetected = false;
unsigned long impactTime = 0;


void setup() {
  Serial.begin(115200);
  //ble code
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
  // Serial.println("BLE Server started - Helmet01");

  //mpu code
  Wire.begin();
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  //mlx
  Wire.begin(21, 22); // SDA, SCL for ESP32
  if (!mlx.begin()) {
    Serial.println("MLx Sensor not found!");
    while (1);
  }
}

void loop() {
  int heart = 78;
  int spo2  = 97;
  // float temp = 36.7;
  String mpu_msg1;
  String mpu_msg2;
  //mpu code
    if (!isMPUConnected()) {
    Serial.println("ERROR: MPU9250 not detected!");
    while (1);
  }
  mySensor.accelUpdate();
  // mySensor.gyroUpdate();
  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();
  // float gx = mySensor.gyroX();
  // float gy = mySensor.gyroY();
  // float gz = mySensor.gyroZ();
  float g = sqrt(ax*ax + ay*ay + az*az);
  // 1. Impact detection (>2.5g)
  if (!impactDetected && g > 2.5) {
    impactDetected = true;
    impactTime = millis();
    // Serial.println("Impact detected...");
    mpu_msg1="Impact detected";
  }
  else{
    // mpu_msg1="Not detected";
  }
  // 2. Fall confirmation
  if (impactDetected) {
      if (millis() - impactTime < 2000) {
          // Serial.println("FALL DETECTED!");
          impactDetected = false;
          mpu_msg2="FALL DETECTED!";
        }
  else {
        impactDetected = false;
        // Serial.println("Fall not confirmed!");
        mpu_msg2="FALL not confirmed !";
      }
    }
  float AmbientTemp = mlx.readAmbientTempC();
  float BodyTemp = mlx.readObjectTempC();
  String payload = String(heart) + "," + String(spo2) + "," + String(AmbientTemp) + ","+ String(BodyTemp) + "," + mpu_msg1+","+mpu_msg2 ;
  sensorChar->setValue(payload.c_str());
  sensorChar->notify();
  // Serial.println("Sent: " + payload);
  delay(1000);
}