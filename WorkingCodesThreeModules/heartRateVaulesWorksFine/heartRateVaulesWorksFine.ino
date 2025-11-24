#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Short-term averaging
byte rates[RATE_SIZE];     // Array of recent beats
byte rateSpot = 0;
long lastBeat = 0;         // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// ---------- 1-minute averaging ----------
unsigned long minuteStart = 0;
float hrSum = 0;
int hrCount = 0;

const long FINGER_THRESHOLD = 30000; // Adjust based on your sensor/finger

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  // Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); // Red LED
  particleSensor.setPulseAmplitudeGreen(0);  // Green LED off

  minuteStart = millis();
}

void loop()
{
  long irValue = particleSensor.getIR();

  // ---------- Finger detection ----------
  bool fingerPresent = (irValue > FINGER_THRESHOLD);

  if (!fingerPresent) {
    Serial.println("No finger detected");
    // reset HR sums to avoid wrong averaging
    hrSum = 0;
    hrCount = 0;
    lastBeat = millis(); // prevent huge delta on finger return
  }
  else
  {
    // ---------- HEART RATE PROCESS ----------
    if (checkForBeat(irValue) == true)
    {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute > 20 && beatsPerMinute < 255)
      {
        // Short-term averaging (optional)
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        // Add to 1-minute sum
        hrSum += beatsPerMinute;
        hrCount++;
      }
    }
  }

  // ---------- 1-minute averaging ----------
  if (millis() - minuteStart >= 60000)
  {
    if (hrCount > 0)
    {
      int avgHR = hrSum / hrCount;
      Serial.print("1-Min Avg BPM: ");
      Serial.println(avgHR);
    }

    // Reset for next minute
    minuteStart = millis();
    hrSum = 0;
    hrCount = 0;
  }

  delay(5);
}
