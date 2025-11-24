#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// HR variables
long lastBeat = 0;
float beatsPerMinute;
byte RATE_SIZE = 4;           // For short-term averaging
byte rates[4];                 // Array to hold recent beats
byte rateSpot = 0;
byte beatAvg = 0;
bool beatDetected;

// SPO2 buffers
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t spo2;
int8_t validSPO2;
int32_t calcHR;
int8_t validHR;

// Minute averaging
unsigned long minuteStart = 0;
float hrSum = 0;
int hrCount = 0;
int spo2Sum = 0;
int spo2Count = 0;

void setup() {
  Serial.begin(115200);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor not detected");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);


  particleSensor.setPulseAmplitudeGreen(0);

  minuteStart = millis();
}

void loop() {
  long irValue = particleSensor.getIR();

  // ---------- Finger detection ----------
  bool fingerPresent = irValue > 30000;

  // ---------- HEART RATE PROCESS ----------
  beatDetected = checkForBeat(irValue);
  if (beatDetected) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute; // short-term average
      rateSpot %= RATE_SIZE;

      // Short-term avg
      byte shortAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++)
        shortAvg += rates[i];
      shortAvg /= RATE_SIZE;

      // Add to 1-minute sum
      hrSum += shortAvg;
      hrCount++;
    }
  }

  // ---------- SPO2 PROCESS (100 sample buffer) ----------
  static byte idx = 0;
  if (particleSensor.available()) {
    redBuffer[idx] = particleSensor.getRed();
    irBuffer[idx] = particleSensor.getIR();
    particleSensor.nextSample();
    idx++;

    if (idx >= 100) {
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, 100, redBuffer,
        &spo2, &validSPO2, &calcHR, &validHR
      );

      if (validSPO2 && spo2 > 70 && spo2 < 100) {
        spo2Sum += spo2;
        spo2Count++;
      }

      idx = 0; // reset buffer index
    }
  }

  // ---------- 1-minute averaging ----------
  if (millis() - minuteStart >= 60000) {
    int avgHR = (hrCount > 0) ? hrSum / hrCount : 0;
    int avgSPO2 = (spo2Count > 0) ? spo2Sum / spo2Count : 0;

    if (!fingerPresent) {
      Serial.println("No finger detected");
    } else {
      Serial.print("Avg HR: ");
      Serial.print(avgHR);
      Serial.print(" bpm  |  Avg SpO2: ");
      Serial.print(avgSPO2);
      Serial.println(" %");
    }
    // Reset for next minute
    minuteStart = millis();
    hrSum = 0;
    hrCount = 0;
    spo2Sum = 0;
    spo2Count = 0;
  }
  delay(5);
}
