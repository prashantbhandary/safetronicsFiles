#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// BLE Components
BLECharacteristic *sensorChar;
BLEServer *pServer;
BLEService *pService;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      // Serial.println("BLE Client Disconnected!");
    }
};

// Sensor Objects
MPU9250_asukiaaa mySensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

// Button and Buzzer Pins
const int button1Pin = 23;  // SOS button
const int button2Pin = 4;   // Cancel/Stop button
const int buzzerPin = 13;   // Buzzer

// Button and Alert Variables
bool sosButtonPressed = false;
bool cancelButtonPressed = false;
bool fallAlertActive = false;
bool buzzerActive = false;
bool shortBeepActive = false;
unsigned long shortBeepStartTime = 0;
const unsigned long SHORT_BEEP_DURATION = 200;  // 200ms beep
unsigned long fallDetectedTime = 0;
const unsigned long FALL_CANCEL_TIMEOUT = 3000;  // 3 seconds to cancel

// Shared Data Structure
struct SensorData {
  int heartRate;
  int spo2;
  float ambientTemp;
  float bodyTemp;
  String mpuStatus;
  String sosStatus;
  bool dataReady;
  unsigned long lastUpdate;
};



// Global variables with mutex protection
SensorData sharedData = {75, 98, 25.0, 36.5, "", "", true, 0};
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t i2cMutex;  // Mutex for I2C bus access
unsigned long bleTransmitStart = 0;  // Timer for 60-second BLE transmission
bool impactDetectedForBLE = false;   // Flag for immediate impact transmission
bool sosMessageForBLE = false;       // Flag for immediate SOS transmission

// Button Interrupt Service Routines
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

void IRAM_ATTR button1ISR() {
  button1Pressed = true;
}

void IRAM_ATTR button2ISR() {
  button2Pressed = true;
}

// Buzzer Control Functions
void buzzerShortBeep() {
  shortBeepActive = true;
  shortBeepStartTime = millis();
  digitalWrite(buzzerPin, HIGH);
  // Serial.println("SOS Buzzer beep started");
}

void startBuzzerAlert() {
  buzzerActive = true;
}

void stopBuzzerAlert() {
  buzzerActive = false;
  digitalWrite(buzzerPin, LOW);
}

// MPU variables
#define MPU_ADDR 0x68
bool impactDetected = false;
unsigned long impactTime = 0;

// Heart Rate variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
unsigned long hrMinuteStart = 0;
float hrSum = 0;
int hrCount = 0;
const long FINGER_THRESHOLD = 30000;

// SPO2 variables
#define SPO2_BUFFER_SIZE 100
#define SPO2_FINGER_THRESHOLD 50000
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int32_t spo2Value;
int8_t validSPO2;
int32_t heartRateFromSPO2;
int8_t validHR;
unsigned long spo2MinuteStart = 0;
int spo2Sum = 0;
int spo2Count = 0;
byte spo2BufferIdx = 0;

// Function prototypes
void heartRateTask(void *parameter);
void spo2Task(void *parameter);
void mpuTask(void *parameter);
void mlxTask(void *parameter);
void bleTask(void *parameter);
void buttonTask(void *parameter);
void buzzerTask(void *parameter);
bool isMPUConnected();
void initializeBLE();
void initializeSensors();

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing SafeTonics Multi-Sensor System...");
  
  // Create mutex for shared data and I2C
  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL || i2cMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while(1);
  }
  
  // Initialize button and buzzer pins
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  
  // Attach button interrupts
  attachInterrupt(digitalPinToInterrupt(button1Pin), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(button2Pin), button2ISR, FALLING);
  
  Serial.println("Buttons and buzzer initialized!");
  
  // Test buzzer on startup
  Serial.println("Testing buzzer...");
  digitalWrite(buzzerPin, HIGH);
  delay(300);
  digitalWrite(buzzerPin, LOW);
  Serial.println("Buzzer test complete!");
  
  // Initialize all sensors
  initializeSensors();
  // Initialize BLE
  initializeBLE();
  // Create RTOS tasks
  xTaskCreatePinnedToCore(
    heartRateTask,    // Task function
    "HeartRateTask",  // Task name
    4096,             // Stack size
    NULL,             // Parameter
    2,                // Priority
    NULL,             // Task handle
    0                 // Core (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    spo2Task,
    "SPO2Task",
    4096,
    NULL,
    2,
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    mpuTask,
    "MPUTask",
    4096,
    NULL,
    3,  // Higher priority for safety
    NULL,
    1
  );
  xTaskCreatePinnedToCore(
    mlxTask,
    "MLXTask",
    2048,
    NULL,
    1,
    NULL,
    1
  );
  xTaskCreatePinnedToCore(
    bleTask,
    "BLETask",
    4096,
    NULL,
    1,
    NULL,
    1
  );
  
  xTaskCreatePinnedToCore(
    buttonTask,
    "ButtonTask",
    2048,
    NULL,
    4,  // High priority for responsiveness
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    buzzerTask,
    "BuzzerTask",
    2048,
    NULL,
    3,  // High priority for alerts
    NULL,
    0
  );
  
  Serial.println("All tasks created successfully!");
}

void loop() {
  // Empty - all work done in tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void heartRateTask(void *parameter) {
  hrMinuteStart = millis();
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Stagger task start
  
  while(1) {
    // Read heart rate sensor with I2C mutex
    long irValue = 0;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Wire.setClock(400000);
      irValue = particleSensor.getIR();
      xSemaphoreGive(i2cMutex);
    }
    
    bool fingerPresent = (irValue > FINGER_THRESHOLD);
    
    if (!fingerPresent) {
      // Send NaN when finger not detected and reset data
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedData.heartRate = 0; // Send 0 instead of NaN for integer
        xSemaphoreGive(dataMutex);
      }
      hrSum = 0;
      hrCount = 0;
      lastBeat = millis();
    } else {
      if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        
        beatsPerMinute = 60 / (delta / 1000.0);
        
        if (beatsPerMinute > 20 && beatsPerMinute < 255) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
          
          hrSum += beatsPerMinute;
          hrCount++;
        }
      }
    }
    
    // Only update shared data every 60 seconds with average
    if (millis() - hrMinuteStart >= 60000) {
      if (hrCount > 0) {
        int avgHR = hrSum / hrCount;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedData.heartRate = avgHR;
          xSemaphoreGive(dataMutex);
        }
        // Serial.println("HR 60-sec average: " + String(avgHR) + " BPM");
      }
      hrMinuteStart = millis();
      hrSum = 0;
      hrCount = 0;
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Faster with I2C mutex protection
  }
}

void spo2Task(void *parameter) {
  spo2MinuteStart = millis();
  vTaskDelay(200 / portTICK_PERIOD_MS);  // Stagger task start
  
  while(1) {
    // Read SPO2 sensor with I2C mutex
    uint32_t irVal = 0, redVal = 0;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Wire.setClock(400000);
      irVal = particleSensor.getIR();
      redVal = particleSensor.getRed();
      particleSensor.nextSample();
      xSemaphoreGive(i2cMutex);
    }
    
    bool fingerPresent = (irVal > SPO2_FINGER_THRESHOLD);
    
    if (!fingerPresent) {
      // Send 0 when finger not detected and reset buffer
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedData.spo2 = 0; // Send 0 instead of NaN for integer
        xSemaphoreGive(dataMutex);
      }
      spo2BufferIdx = 0;
    } else {
      redBuffer[spo2BufferIdx] = redVal;
      irBuffer[spo2BufferIdx] = irVal;
      spo2BufferIdx++;
      
      if (spo2BufferIdx >= SPO2_BUFFER_SIZE) {
        maxim_heart_rate_and_oxygen_saturation(irBuffer, SPO2_BUFFER_SIZE,
                                               redBuffer,
                                               &spo2Value, &validSPO2,
                                               &heartRateFromSPO2, &validHR);
        
        if (validSPO2 && spo2Value > 70 && spo2Value <= 100) {
          spo2Sum += spo2Value;
          spo2Count++;
        }
        spo2BufferIdx = 0;
      }
    }
    
    // Only update shared data every 60 seconds with average
    if (millis() - spo2MinuteStart >= 60000) {
      if (spo2Count > 0) {
        int avgSPO2 = spo2Sum / spo2Count;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedData.spo2 = avgSPO2;
          xSemaphoreGive(dataMutex);
        }
        // Serial.println("SPO2 60-sec average: " + String(avgSPO2) + "%");
      }
      spo2Sum = 0;
      spo2Count = 0;
      spo2MinuteStart = millis();
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Faster with I2C mutex protection
  }
}

void mpuTask(void *parameter) {
  vTaskDelay(300 / portTICK_PERIOD_MS);  // Stagger task start
  
  while(1) {
    bool mpuConnected = false;
    float ax = 0, ay = 0, az = 0;
    
    // Read MPU sensor with I2C mutex
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Wire.setClock(400000);
      mpuConnected = isMPUConnected();
      
      if (mpuConnected) {
        mySensor.accelUpdate();
        ax = mySensor.accelX();
        ay = mySensor.accelY();
        az = mySensor.accelZ();
      }
      xSemaphoreGive(i2cMutex);
    }
    
    if (!mpuConnected) {
      Serial.println("ERROR: MPU9250 not detected!");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    
    float g = sqrt(ax*ax + ay*ay + az*az);
    
    String impactMsg = "";
    
    // Impact detection (>2.5g)
    if (!impactDetected && g > 2.5) {
      impactDetected = true;
      impactTime = millis();
      fallDetectedTime = millis();  // Start 3-second cancel window
      fallAlertActive = true;
      impactMsg = "Impact detected";
      
      Serial.println("IMPACT! G:" + String(g));
      // Serial.println("Press cancel button within 3 seconds to cancel alert");
      
      // Start buzzer immediately
      startBuzzerAlert();
    }
    
    // Check if 3 seconds passed without cancel - then send BLE ONCE
    if (fallAlertActive && (millis() - fallDetectedTime >= FALL_CANCEL_TIMEOUT)) {
      if (!impactDetectedForBLE) {  // Only set once
        impactDetectedForBLE = true;
        fallAlertActive = false;  // Stop the alert cycle immediately
        // Serial.println("3 seconds passed - sending fall alert via BLE (ONCE)");
      }
    }
    
    // Reset impact after 10 seconds total (give time for BLE transmission)
    if (impactDetected && (millis() - impactTime > 10000)) {
      impactDetected = false;
      fallAlertActive = false;
      stopBuzzerAlert();
      impactMsg = "";
    }
    
    // Update shared data - keep impact message available for BLE
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (impactDetected) {
        sharedData.mpuStatus = "Impact detected";
      } else {
        sharedData.mpuStatus = "";
      }
      xSemaphoreGive(dataMutex);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Faster with I2C mutex protection
  }
}

void mlxTask(void *parameter) {
  unsigned long mlxReadStart = millis();  // Timer for 60-second MLX reading
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Stagger task start
  
  while(1) {
    // Read MLX sensor only once every 60 seconds
    if (millis() - mlxReadStart >= 60000) {
      float ambientTemp = NAN, bodyTemp = NAN;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.setClock(100000);
        ambientTemp = mlx.readAmbientTempC();
        bodyTemp = mlx.readObjectTempC();
        xSemaphoreGive(i2cMutex);
      }
      
      // Check if readings are valid
      if (!isnan(ambientTemp) && !isnan(bodyTemp)) {
        // Update shared data
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          sharedData.ambientTemp = ambientTemp;
          sharedData.bodyTemp = bodyTemp;
          sharedData.dataReady = true;
          sharedData.lastUpdate = millis();
          xSemaphoreGive(dataMutex);
        }
        // Serial.println("MLX Temp 60-sec reading - Ambient: " + String(ambientTemp,1) + "C, Body: " + String(bodyTemp,1) + "C");
      }
      
      mlxReadStart = millis();  // Reset 60-second timer
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Check every second but only read every 60 seconds
  }
}



void bleTask(void *parameter) {
  bleTransmitStart = millis();  // Initialize BLE timer
  
  while(1) {
    // Handle BLE connection changes
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // Give time for bluetooth stack to get ready
      pServer->startAdvertising();
      // Serial.println("Start advertising again...");
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      // Serial.println("BLE Client connected!");
      oldDeviceConnected = deviceConnected;
    }
    
    bool shouldTransmit = false;
    
    // Check for transmission triggers
    bool isRegularTransmission = (millis() - bleTransmitStart >= 60000);
    bool isImpactTransmission = impactDetectedForBLE;
    bool isSOSTransmission = sosMessageForBLE;
    
    if (isRegularTransmission || isImpactTransmission || isSOSTransmission) {
      shouldTransmit = true;
      
      if (isRegularTransmission) {
        bleTransmitStart = millis();  // Reset 60-second timer
        // Serial.println("60-second BLE transmission");
      }
      
      if (isImpactTransmission) {
        impactDetectedForBLE = false;  // Reset IMMEDIATELY to prevent repeats
        // Serial.println("Immediate impact BLE transmission (ONE TIME ONLY)");
      }
      
      if (isSOSTransmission) {
        sosMessageForBLE = false;  // Reset IMMEDIATELY to prevent repeats
        // Serial.println("Immediate SOS BLE transmission (ONE TIME ONLY)");
      }
    }
    
    if (shouldTransmit) {
      // Read shared data
      SensorData localData;
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localData = sharedData;
        xSemaphoreGive(dataMutex);
      }
  
      // Create JSON payload using snprintf

      // String payload = String(localData.heartRate) + "," + 
      //                 String(localData.spo2) + "," + 
      //                 String(localData.ambientTemp, 1) + "," + 
      //                 String(localData.bodyTemp, 1) + "," + 
      //                 localData.mpuStatus + "," + 
      //                 localData.sosStatus;
      
      // // Set BLE characteristic value
      // sensorChar->setValue(payload.c_str())
      char payload[256];
      snprintf(payload, sizeof(payload),
        "{\"ID\":%d,\"heartRate\":%d,\"spo2\":%d,\"ambientTemp\":%.1f,\"bodyTemp\":%.1f,\"mpuStatus\":\"%s\",\"sosStatus\":\"%s\"}",
        1,
        localData.heartRate,
        localData.spo2,
        localData.ambientTemp,
        localData.bodyTemp,
        localData.mpuStatus.c_str(),
        localData.sosStatus.c_str()
      );
      
      // Set BLE characteristic value
      sensorChar->setValue(payload);
      
      // Send if client is connected, otherwise skip
      if (deviceConnected) {
        sensorChar->notify();
        Serial.println(payload);
      } else {
        // Serial.println("No BLE client - Data lost: " + payload);
      }
      
      // Clear status messages after successful transmission
      if (deviceConnected) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (isSOSTransmission) {
            sharedData.sosStatus = "";
          }
          if (isImpactTransmission) {
            sharedData.mpuStatus = "";
          }
          xSemaphoreGive(dataMutex);
        }
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void buttonTask(void *parameter) {
  while(1) {
    // Check SOS button (Button 1)
    if (button1Pressed) {
      button1Pressed = false;
      // Serial.println("SOS!");
      // Give feedback with short beep
      buzzerShortBeep();
      // Set SOS flag for immediate BLE transmission
      sosMessageForBLE = true;
      // Serial.println("SOS message queued for BLE transmission");
      // Update shared data
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedData.sosStatus = "SOS Alert";
        xSemaphoreGive(dataMutex);
      }
      else {
        sharedData.sosStatus= "";
      }
    }
    
    // Check Cancel button (Button 2)
    if (button2Pressed) {
      button2Pressed = false;
      // Serial.println("CANCEL BUTTON PRESSED!");
      
      // If fall alert is active and within cancel timeout
      if (fallAlertActive && (millis() - fallDetectedTime < FALL_CANCEL_TIMEOUT)) {
        Serial.println("CANCELLED");
        fallAlertActive = false;
        stopBuzzerAlert();
        
        // Clear impact status
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedData.mpuStatus = "";
          sharedData.sosStatus = "";
          xSemaphoreGive(dataMutex);
        }
        // Don't send BLE message for this fall
        impactDetectedForBLE = false;
      }
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Check buttons every 50ms
  }
}

void buzzerTask(void *parameter) {
  bool buzzerState = false;
  unsigned long lastToggle = 0;
  const unsigned long BUZZ_INTERVAL = 500;  // 500ms on/off for continuous alert
  
  while(1) {
    // Handle short beep (SOS feedback)
    if (shortBeepActive) {
      if (millis() - shortBeepStartTime >= SHORT_BEEP_DURATION) {
        digitalWrite(buzzerPin, LOW);
        shortBeepActive = false;
        // Serial.println("SOS Buzzer beep finished");
      }
    }
    // Handle continuous buzzing for fall alert (only if no short beep)
    else if (buzzerActive) {
      if (millis() - lastToggle >= BUZZ_INTERVAL) {
        buzzerState = !buzzerState;
        digitalWrite(buzzerPin, buzzerState ? HIGH : LOW);
        lastToggle = millis();
      }
    } else {
      // Ensure buzzer is off when not active
      digitalWrite(buzzerPin, LOW);
      buzzerState = false;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Faster check for responsive beeping
  }
}

bool isMPUConnected() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

void initializeSensors() {
  // Initialize I2C with slower speed for MLX90614 compatibility
  Wire.begin(21, 22); // SDA, SCL for ESP32
  Wire.setClock(100000); // 100kHz for better MLX90614 compatibility
  
  // Serial.println("Initializing sensors...");
  
  // Initialize MLX90614 first with multiple attempts
  bool mlxInitialized = false;
  for (int attempts = 0; attempts < 5; attempts++) {
    // Serial.println("MLX90614 init attempt " + String(attempts + 1));
    if (mlx.begin()) {
      mlxInitialized = true;
      // Serial.println("MLX90614 initialized successfully!");
      break;
    }
    delay(500);
  }
  
  if (!mlxInitialized) {
    Serial.println("MLX90614 failed to initialize!");
  }
  
  // Initialize MAX30105 with standard I2C speed
  Wire.setClock(400000); // 400kHz for MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  // Serial.println("MAX30105 initialized successfully!");
  
  // Initialize MPU9250
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  Serial.println("MPU9250 initialized successfully!");
  
  Serial.println("All sensors initialization completed!");
}

void initializeBLE() {
  BLEDevice::init("SafeTonics-Helmet01");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  pService = pServer->createService("1234");
  
  sensorChar = pService->createCharacteristic(
    "5678",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  
  sensorChar->addDescriptor(new BLE2902());
  pService->start();
  
  // Set initial value
  sensorChar->setValue("0,0,0.0,0.0,,");
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("1234");
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE Server started - SafeTonics-Helmet01");
  Serial.println("Advertising UUID: 1234");
  Serial.println("Characteristic UUID: 5678");
}

