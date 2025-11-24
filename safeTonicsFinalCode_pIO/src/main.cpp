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
      Serial.println("BLE Client Connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected!");
    }
};

// Sensor Objects
MPU9250_asukiaaa mySensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

// Shared Data Structure
struct SensorData {
  int heartRate;
  int spo2;
  float ambientTemp;
  float bodyTemp;
  String mpuStatus;
  String fallStatus;
  bool dataReady;
  unsigned long lastUpdate;
};

// Global variables with mutex protection
SensorData sharedData = {75, 98, 25.0, 36.5, "Normal", "No Fall", true, 0};
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t i2cMutex;  // Mutex for I2C bus access
unsigned long bleTransmitStart = 0;  // Timer for 60-second BLE transmission
bool impactDetectedForBLE = false;   // Flag for immediate impact transmission

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
        Serial.println("HR 60-sec average: " + String(avgHR) + " BPM");
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
        Serial.println("SPO2 60-sec average: " + String(avgSPO2) + "%");
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
      impactMsg = "Impact detected";
      impactDetectedForBLE = true;  // Set flag for immediate BLE transmission
      Serial.println("IMPACT DETECTED! G-force: " + String(g));
    }
    
    // Reset impact after 5 seconds (give time for BLE transmission)
    if (impactDetected && (millis() - impactTime > 5000)) {
      impactDetected = false;
      impactMsg = "";
    }
    
    // Update shared data - always set current status
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (impactDetected) {
        sharedData.mpuStatus = "Impact detected";
      } else {
        sharedData.mpuStatus = "";
      }
      sharedData.fallStatus = "";
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
        Serial.println("MLX Temp 60-sec reading - Ambient: " + String(ambientTemp,1) + "C, Body: " + String(bodyTemp,1) + "C");
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
      Serial.println("Start advertising again...");
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
    
    bool shouldTransmit = false;
    
    // Check if 60 seconds have passed OR impact detected
    if ((millis() - bleTransmitStart >= 60000) || impactDetectedForBLE) {
      shouldTransmit = true;
      
      if (millis() - bleTransmitStart >= 60000) {
        bleTransmitStart = millis();  // Reset 60-second timer
        Serial.println("60-second BLE transmission");
      }
      
      if (impactDetectedForBLE) {
        Serial.println("Immediate impact BLE transmission");
      }
    }
    
    if (shouldTransmit) {
      // Read shared data
      SensorData localData;
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        localData = sharedData;
        xSemaphoreGive(dataMutex);
      }
      
      // Send data
      String payload = String(localData.heartRate) + "," + 
                      String(localData.spo2) + "," + 
                      String(localData.ambientTemp, 1) + "," + 
                      String(localData.bodyTemp, 1) + "," + 
                      localData.mpuStatus + "," + 
                      localData.fallStatus;
      
      // Set BLE characteristic value
      sensorChar->setValue(payload.c_str());
      
      // Only notify if client is connected
      if (deviceConnected) {
        sensorChar->notify();
        Serial.println("BLE Sent: " + payload);
      } else {
        Serial.println("No BLE client - Data: " + payload);
      }
      
      // Reset impact flag after transmission attempt
      if (impactDetectedForBLE) {
        impactDetectedForBLE = false;
        Serial.println("Impact BLE flag reset after transmission");
      }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
  
  Serial.println("Initializing sensors...");
  
  // Initialize MLX90614 first with multiple attempts
  bool mlxInitialized = false;
  for (int attempts = 0; attempts < 5; attempts++) {
    Serial.println("MLX90614 init attempt " + String(attempts + 1));
    if (mlx.begin()) {
      mlxInitialized = true;
      Serial.println("MLX90614 initialized successfully!");
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
  Serial.println("MAX30105 initialized successfully!");
  
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

