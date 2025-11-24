#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

#define MPU_ADDR 0x68

bool isMPUConnected() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

bool impactDetected = false;
unsigned long impactTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  Serial.println("MPU9250 ready!");
}

void loop() {

  if (!isMPUConnected()) {
    Serial.println("ERROR: MPU9250 not detected!");
    while (1);
  }

  mySensor.accelUpdate();
  mySensor.gyroUpdate();

  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();
  float gx = mySensor.gyroX();
  float gy = mySensor.gyroY();
  float gz = mySensor.gyroZ();

  float g = sqrt(ax*ax + ay*ay + az*az);
  // 1. Impact detection (>2.5g)
  if (!impactDetected && g > 2.5) {
    impactDetected = true;
    impactTime = millis();
    Serial.println("Impact detected...");
  }
  // 2. Fall confirmation
if (impactDetected) {
    if (millis() - impactTime < 2000) {
        Serial.println("FALL DETECTED!");
        impactDetected = false;
      }
 else {
      impactDetected = false;
      Serial.println("Fall not confirmed!");
    }
  }
  delay(50);
}
