#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define BUFFER_SIZE 100
#define FINGER_THRESHOLD 50000  // IR above this = finger present

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHR;

unsigned long minuteStart = 0;
int spo2Sum = 0;
int spo2Count = 0;

void setup() {
  Serial.begin(115200);
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while (1)
      ;
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  minuteStart = millis();
}

void loop() {
  static byte idx = 0;

  uint32_t irVal = particleSensor.getIR();
  uint32_t redVal = particleSensor.getRed();
  particleSensor.nextSample();

  // Finger detection
  bool fingerPresent = (irVal > FINGER_THRESHOLD);
  if (!fingerPresent) {
    Serial.println("No Finger Detected");
    idx = 0;      // reset buffer
    spo2Sum = 0;  // reset 1-min average
    spo2Count = 0;
    minuteStart = millis();
    delay(200);  // slow down repeated prints
    return;
  }

  // Store data for SPO2 calculation
  else {
    redBuffer[idx] = redVal;
  irBuffer[idx] = irVal;
  idx++;

  if (idx >= BUFFER_SIZE) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE,
                                           redBuffer,
                                           &spo2, &validSPO2,
                                           &heartRate, &validHR);
    if (validSPO2 && spo2 > 70 && spo2 <= 100) {
      spo2Sum += spo2;
      spo2Count++;
    }
    idx = 0;  // reset buffer
  }
}


  // 1-minute average SPO2
  if (millis() - minuteStart >= 60000 && spo2Count > 0) {
    int avgSPO2 = spo2Sum / spo2Count;
    Serial.print("1-Min Avg SPO2: ");
    Serial.print(avgSPO2);
    Serial.println(" %");

    spo2Sum = 0;
    spo2Count = 0;
    minuteStart = millis();
  }

  delay(5);
}
