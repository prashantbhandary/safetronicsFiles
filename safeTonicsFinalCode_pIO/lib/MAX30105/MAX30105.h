/*
  MAX30105 Library Header
  This is a minimal header for MAX30105 sensor
*/

#ifndef MAX30105_H
#define MAX30105_H

#include <Arduino.h>
#include <Wire.h>

// MAX30105 I2C Address
#define MAX30105_ADDRESS 0x57

// Register addresses
#define MAX30105_FIFOWRITEPTR 0x04
#define MAX30105_FIFOOVERFLOW 0x05
#define MAX30105_FIFOREADPTR 0x06
#define MAX30105_FIFODATA 0x07
#define MAX30105_FIFOCONFIG 0x08
#define MAX30105_MODECONFIG 0x09
#define MAX30105_PARTICLECONFIG 0x0A
#define MAX30105_LED1_PULSEAMP 0x0C
#define MAX30105_LED2_PULSEAMP 0x0D
#define MAX30105_LED3_PULSEAMP 0x0E
#define MAX30105_MULTILEDCONFIG1 0x11
#define MAX30105_MULTILEDCONFIG2 0x12

class MAX30105 {
public:
  MAX30105();
  
  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30105_ADDRESS);
  
  uint32_t getRed();
  uint32_t getIR();
  uint32_t getGreen();
  
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
  void setPulseAmplitudeRed(uint8_t amplitude);
  void setPulseAmplitudeIR(uint8_t amplitude);
  void setPulseAmplitudeGreen(uint8_t amplitude);
  void setPulseAmplitudeProximity(uint8_t amplitude);
  
  void nextSample();
  uint16_t getWritePointer();
  uint16_t getReadPointer();
  
private:
  TwoWire *_i2cPort;
  uint8_t _i2caddr;
  
  void writeRegister8(uint8_t address, uint8_t value);
  uint8_t readRegister8(uint8_t address);
  
  struct {
    uint32_t red[32];
    uint32_t IR[32];
    uint32_t green[32];
    byte head;
    byte tail;
  } sense;
};

#endif