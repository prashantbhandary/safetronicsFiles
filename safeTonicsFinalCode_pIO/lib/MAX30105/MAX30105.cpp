#include "MAX30105.h"

MAX30105::MAX30105() {
  // Constructor
}

boolean MAX30105::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {
  _i2cPort = &wirePort;
  _i2caddr = i2caddr;
  
  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);
  
  // Check if sensor is responding
  _i2cPort->beginTransmission(_i2caddr);
  if (_i2cPort->endTransmission() != 0) {
    return false;
  }
  
  return true;
}

void MAX30105::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
  // Reset all configuration
  writeRegister8(MAX30105_MODECONFIG, 0x40);
  
  // Configure FIFO
  writeRegister8(MAX30105_FIFOCONFIG, 0x00);
  
  // Set LED mode
  writeRegister8(MAX30105_MODECONFIG, ledMode);
  
  // Set particle sensing configuration
  writeRegister8(MAX30105_PARTICLECONFIG, 0x27);
  
  // Set LED pulse amplitude
  writeRegister8(MAX30105_LED1_PULSEAMP, powerLevel);
  writeRegister8(MAX30105_LED2_PULSEAMP, powerLevel);
}

void MAX30105::setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(MAX30105_LED1_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(MAX30105_LED2_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeGreen(uint8_t amplitude) {
  writeRegister8(MAX30105_LED3_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8(MAX30105_LED1_PULSEAMP, amplitude);
}

uint32_t MAX30105::getRed() {
  // Read FIFO data
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(MAX30105_FIFODATA);
  _i2cPort->endTransmission(false);
  
  _i2cPort->requestFrom(_i2caddr, (uint8_t)3);
  
  uint32_t temp = 0;
  temp |= (uint32_t)_i2cPort->read() << 16;
  temp |= (uint32_t)_i2cPort->read() << 8;
  temp |= (uint32_t)_i2cPort->read();
  temp &= 0x03FFFF;
  
  return temp;
}

uint32_t MAX30105::getIR() {
  return getRed(); // Simplified for basic functionality
}

uint32_t MAX30105::getGreen() {
  return getRed(); // Simplified for basic functionality
}

void MAX30105::nextSample() {
  // Advance to next sample
}

uint16_t MAX30105::getWritePointer() {
  return readRegister8(MAX30105_FIFOWRITEPTR);
}

uint16_t MAX30105::getReadPointer() {
  return readRegister8(MAX30105_FIFOREADPTR);
}

void MAX30105::writeRegister8(uint8_t address, uint8_t value) {
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(address);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}

uint8_t MAX30105::readRegister8(uint8_t address) {
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(address);
  _i2cPort->endTransmission(false);
  
  _i2cPort->requestFrom(_i2caddr, (uint8_t)1);
  if (_i2cPort->available()) {
    return _i2cPort->read();
  }
  return 0;
}