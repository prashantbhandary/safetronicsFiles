# SafeTonics Multi-Sensor Helmet System

A comprehensive IoT safety monitoring system built with ESP32 and multiple sensors for real-time health and safety tracking.

## 🔍 Overview

SafeTonics is an intelligent helmet system that monitors vital signs and detects impacts in real-time. It combines multiple sensors to provide comprehensive safety monitoring for workers in hazardous environments.

## 🚀 Features

- **Real-time Health Monitoring**
  - Heart rate detection using MAX30105 sensor
  - Blood oxygen (SPO2) level measurement
  - Body and ambient temperature monitoring with MLX90614

- **Impact Detection**
  - Motion sensing with MPU9250 accelerometer
  - Automatic impact detection (>2.5g threshold)
  - Immediate alert transmission

- **Wireless Communication**
  - Bluetooth Low Energy (BLE) connectivity
  - 60-second regular data transmission
  - Instant impact alerts
  - Mobile device compatibility

- **Smart Data Management**
  - 60-second averaged sensor readings
  - Finger presence detection for accurate vitals
  - Thread-safe multi-sensor coordination

## 📋 System Architecture

### Hardware Components
- **ESP32** - Main microcontroller with dual-core processing
- **MAX30105** - Heart rate and SPO2 sensor
- **MPU9250** - 9-axis motion sensor for impact detection
- **MLX90614** - Non-contact infrared temperature sensor
- **I2C Bus** - Sensor communication protocol

### Software Framework
- **FreeRTOS** - Real-time operating system for task management
- **Multi-threading** - Separate tasks for each sensor and BLE
- **Mutex Protection** - Thread-safe data sharing
- **PlatformIO** - Development platform and dependency management

## 🔧 Technical Specifications

| Component | Specification | Purpose |
|-----------|---------------|---------|
| ESP32 | Dual-core 240MHz | Main processing & BLE |
| MAX30105 | Heart rate & SpO2 | Vital signs monitoring |
| MPU9250 | ±16g accelerometer | Impact detection |
| MLX90614 | -70°C to +380°C | Temperature monitoring |
| BLE 4.0+ | Low energy wireless | Data transmission |

## 📁 Project Structure

```
safetonicsFiles/
├── safeTonicsFinalCode_pIO/          # Main ESP32 project
│   ├── src/
│   │   └── main.cpp                  # Core application code
│   ├── lib/                          # Local sensor libraries
│   │   ├── MAX30105/                 # Heart rate sensor library
│   │   ├── heartRate/                # Heart rate algorithms
│   │   └── spo2_algorithm/           # SPO2 calculation
│   └── platformio.ini                # Project configuration
├── pythonFiles/                      # Testing utilities
│   ├── receive.py                    # BLE data receiver
│   └── macfinder.py                  # Device discovery
├── TestingCodes/                     # Individual sensor tests
└── WorkingCodesThreeModules/         # Original sensor modules
```

## 🛠️ Installation & Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) IDE or extension
- ESP32 development board
- Required sensors (MAX30105, MPU9250, MLX90614)

### Hardware Connections
```
ESP32 Connections:
├── I2C Bus (SDA: GPIO21, SCL: GPIO22)
│   ├── MAX30105 (0x57)
│   ├── MPU9250 (0x68)
│   └── MLX90614 (0x5A)
└── Power: 3.3V and GND to all sensors
```

### Software Installation
1. **Clone the repository**
   ```bash
   git clone https://github.com/prashantbhandary/safetonicsFiles
   cd safetonicsFiles/safeTonicsFinalCode_pIO
   ```

2. **Install dependencies**
   ```bash
   pio lib install
   ```

3. **Build and upload**
   ```bash
   pio run --target upload
   ```

4. **Monitor serial output**
   ```bash
   pio device monitor
   ```

## 📊 Data Format

### BLE Data Transmission
The system transmits data every 60 seconds or immediately on impact detection:

```
Format: "heartRate,spo2,ambientTemp,bodyTemp,impactStatus,fallStatus"
Example: "72,98,25.2,36.8,Impact detected,"
```

### Serial Output Examples
```
HR 60-sec average: 72 BPM
SPO2 60-sec average: 98%
MLX Temp 60-sec reading - Ambient: 25.2C, Body: 36.8C
IMPACT DETECTED! G-force: 3.2
BLE Sent: 72,98,25.2,36.8,Impact detected,
```

## 🔄 System Operation

### Task Management
The system runs 5 concurrent tasks using FreeRTOS:

1. **Heart Rate Task** - Monitors pulse detection and calculates BPM
2. **SPO2 Task** - Measures blood oxygen saturation levels
3. **MPU Task** - Detects impacts and sudden movements
4. **MLX Task** - Reads temperature once every 60 seconds
5. **BLE Task** - Handles wireless data transmission

### Data Collection Cycle
- **Continuous monitoring**: Heart rate, SPO2, and motion
- **60-second averaging**: Vital signs data for stable readings
- **Temperature sampling**: Once per minute to reduce I2C conflicts
- **Impact detection**: Real-time with immediate alert transmission

## 🛡️ Safety Features

- **Finger presence detection** - Ensures accurate vital sign readings
- **I2C mutex protection** - Prevents sensor communication conflicts
- **Watchdog monitoring** - System reliability and error recovery
- **Impact persistence** - 5-second alert window for reliable transmission
- **Connection resilience** - Automatic BLE reconnection handling

## 🔍 Testing & Debugging

### Python Test Scripts
Use the provided Python scripts to test BLE connectivity:

```bash
cd pythonFiles
python receive.py  # Monitor incoming BLE data
python macfinder.py  # Discover BLE devices
```

### Serial Monitor Commands
Monitor the system through serial output at 115200 baud rate:
- Heart rate detection events
- SPO2 calculation results
- Temperature readings
- Impact detection alerts
- BLE transmission status

## 📈 Performance Metrics

- **Response Time**: <100ms for impact detection
- **Battery Efficiency**: Optimized BLE and sensor polling
- **Data Accuracy**: 60-second averaging for stable readings
- **Transmission Range**: Up to 10 meters (BLE dependent)
- **Sensor Sampling**: 
  - Heart rate: 20Hz
  - SPO2: 20Hz
  - Motion: 10Hz
  - Temperature: Once per 60 seconds

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-sensor`)
3. Commit your changes (`git commit -am 'Add new sensor support'`)
4. Push to the branch (`git push origin feature/new-sensor`)
5. Create a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

For issues and questions:
- Check the [Issues](issues) section
- Review serial monitor output for debugging
- Verify sensor connections and power supply
- Ensure proper library dependencies

## 📚 Additional Resources

- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [Sensor Datasheets](docs/datasheets/) (if available)

---

**Built with ❤️ for workplace safety and health monitoring**
