---
publishDate: 2024-12-19T00:00:00Z
title: Project Drishti - AI-Enhanced Safety System
excerpt: An IoT-based crowd monitoring system that uses multiple sensors and AI analysis to detect dangerous crowd conditions and prevent stampedes in real-time.
image: cover.jpg
tags:
- iot
- safety
- crowd-monitoring
- public-security
---

> Real-time crowd safety monitoring with AI-powered risk assessment to prevent stampedes and ensure public safety.

## Acknowledgements

We thank IEEE, the MYOSA initiative, and Prof. Digant Parmar from Silver Oak University for their guidance and mentorship throughout the development of this project.

## Overview

Project Drishti is an intelligent IoT device designed to monitor crowd conditions in real-time and prevent dangerous situations like stampedes. The system combines multiple environmental sensors with AI-powered analysis to assess crowd risk levels and provide immediate alerts to safety personnel.

The device addresses the critical need for proactive crowd safety management in public spaces, events, and transportation hubs. By continuously monitoring vibration patterns, pressure changes, proximity data, and temperature variations, the CMU can detect early signs of crowd distress and overcrowding before they escalate into dangerous situations.

The system operates in two modes: LIVE mode for real-world deployment and DEMO mode for testing and presentations. It features automatic sensor calibration, wireless connectivity via WiFi and MQTT, and an integrated display for local status monitoring.

## Demo / Examples

### Images

<p align="center"><img src="cover.jpg" width="800"><br/><i>Project Drishti with OLED display showing real-time sensor data and risk assessment</i></p>

<p align="center"><img src="drishti-sensor-layout.jpeg" width="600"><br/><i>Internal sensor configuration: MPU6050 accelerometer, BMP085 pressure sensor, and APDS9960 proximity sensor</i></p>

<p align="center"><img src="drishti-dashboard.jpeg" width="800"><br/><i>Web dashboard displaying crowd risk indicators and historical data from multiple CMU devices</i></p>

### Videos

<video controls width="100%"><source src="drishti-demo.mp4" type="video/mp4"></video>

## Features (Detailed)

### Multi-Sensor Data Fusion
Project Drishti integrates three primary sensors to create a comprehensive crowd monitoring solution. The MPU6050 accelerometer detects ground vibrations caused by crowd movement, the BMP085 pressure sensor monitors atmospheric pressure changes in enclosed spaces, and the APDS9960 proximity sensor measures local crowd density. All sensor data is processed through advanced filtering algorithms to eliminate noise and provide accurate readings.

### AI-Enhanced Risk Assessment
The system incorporates artificial intelligence algorithms that analyze sensor patterns and historical data to predict crowd risk levels. The AI component calculates crowd count estimates, density measurements, and provides risk classifications (SAFE, CAUTION, WARNING, CRITICAL) based on learned crowd behavior patterns. This enables proactive intervention before dangerous conditions develop.

### Real-Time Communication and Alerts
Built-in WiFi connectivity enables the CMU to communicate with central monitoring systems via MQTT protocol. The device publishes real-time status updates, sensor readings, and risk assessments every 2 seconds. Critical alerts trigger immediate notifications to safety personnel, while an integrated buzzer provides local audio warnings when dangerous conditions are detected.

## Usage Instructions

1. **Initial Setup**: Power on the device and wait for the "MYOSA Initializing..." message on the OLED display
2. **Calibration**: The system automatically calibrates sensors during startup - keep the device still during this process
3. **Network Connection**: Ensure WiFi credentials are configured in the firmware before deployment
4. **Deployment**: Mount the device in the monitoring area at approximately 1.5m height for optimal sensor performance
5. **Monitoring**: Use the MQTT dashboard to monitor multiple devices and receive real-time alerts
6. **Manual Override**: Send MQTT commands to switch between LIVE and DEMO modes or manually set status levels

MQTT Command Examples:
```json
{"command": "MODE", "value": "LIVE"}
{"command": "SET_STATUS", "status": "WARNING"}
{"command": "AI_UPDATE", "ai_crowd_count": 150, "ai_density": 0.75, "combined_risk": "HIGH"}
```

## Tech Stack

- **Hardware**: ESP32 microcontroller, SSD1306 OLED display, MPU6050 accelerometer, BMP085 pressure sensor, APDS9960 proximity sensor
- **Firmware**: Arduino C++ with sensor libraries (Adafruit MPU6050, BMP085, APDS9960)
- **Communication**: WiFi (802.11 b/g/n), MQTT protocol for IoT messaging
- **Data Format**: JSON for structured data exchange
- **AI Integration**: External AI service integration via MQTT for crowd analysis
- **Display**: 128x64 OLED for real-time status visualization

## Requirements / Installation

**Hardware Requirements:**
- ESP32 development board
- SSD1306 128x64 OLED display
- MPU6050 6-axis accelerometer/gyroscope
- BMP085 barometric pressure sensor
- APDS9960 proximity/gesture sensor
- Passive buzzer
- Breadboard and jumper wires

**Software Requirements:**
- Arduino IDE 1.8.19 or later
- ESP32 board package
- Required libraries: Adafruit_MPU6050, Adafruit_BMP085, SparkFun_APDS9960, PubSubClient, ArduinoJson

**Installation Steps:**
```bash
# Install Arduino IDE and ESP32 board package
# Install required libraries via Library Manager
# Configure WiFi credentials in firmware
# Upload firmware to ESP32
# Set up MQTT broker for data collection
```

## File Structure (Optional)

```
MYOSA/
├── firmware_fixed.ino          # Main firmware code
├── project-drishtit.md  # This documentation
├── cover.jpg                   # Device photo
├── drishti-sensor-layout.jpeg    # Hardware layout
├── drishti-dashboard.jpeg      # Dashboard screenshot
└── drishti-demo.mp4             # Demonstration video
```

## License (Optional)

This project is released under the MIT License for educational and research purposes. Commercial deployment requires additional safety certifications and compliance with local regulations.

## Contribution Notes (Optional)

Future enhancements could include additional sensor types (sound level monitoring, air quality), machine learning model optimization, and integration with building management systems. Contributors are welcome to submit improvements for sensor accuracy, power optimization, and AI algorithm refinement.

**Final Submission Checklist**
- [x] Markdown format followed
- [x] Images added correctly
- [x] MP4 video uploaded
- [x] Proper overview written
- [x] Tech stack added
- [x] Commands & code formatted correctly
