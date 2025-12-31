#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>

/* ========== Sensors ========== */
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <SparkFun_APDS9960.h>

/* ========== Display ========== */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ========== WIFI ========== */
const char* WIFI_SSID = "Prashant.4G";
const char* WIFI_PASS = "9722424360";

/* ========== MQTT ========== */
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* CLIENT_ID = "CMU_01_ESP32_LIVE_DEMO";

#define MQTT_STATUS_TOPIC  "drishti/device/CMU_01/status"
#define MQTT_CONTROL_TOPIC "drishti/device/CMU_01/control"

/* ========== HARDWARE ========== */
#define BUZZER_PIN 12
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/* ========== NOISE FLOORS ========== */
const float VIBRATION_NOISE_FLOOR = 0.05f;
const float PRESSURE_NOISE_FLOOR  = 0.1f;
const int   PROX_NOISE_FLOOR      = 5;

/* ========== MODE ========== */
enum SystemMode { MODE_LIVE, MODE_DEMO };
SystemMode currentMode = MODE_LIVE;

/* Demo inputs */
float demoVibration = 0.0;
float demoPressureDelta = 0.0;
int   demoProximity = 0;
float demoTemperature = 0.0;

/* ========== OBJECTS ========== */
WiFiClient espClient;
PubSubClient mqtt(espClient);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
SparkFun_APDS9960 apds;

/* ========== STATE ========== */
String currentStatus = "BOOTING";
float cri = 0.0;

bool manualOverride = false;
unsigned long overrideTime = 0;
const unsigned long overrideDuration = 10000;

/* ========== SENSOR VALUES ========== */
float vibration = 0.0;
float pressureDelta = 0.0;
float temperature = 0.0;
int proximity = 0;

/* ========== BASELINES ========== */
float vibBaseline = 0.0;
double pressBaseline = 0.0;
int proxBaseline = 0;

/* ========== SENSOR STATUS ========== */
bool mpuReady = false;
bool bmpReady = false;
bool apdsReady = false;

/* ========== AI INTEGRATION ========== */
float aiCrowdCount = 0;
float aiDensity = 0.0;
float aiCri = 0.0;
String aiRiskLevel = "LOW";
bool aiDataAvailable = false;

/* ========== AI CROWD ANALYSIS ========== */
float calculateAICrowdDensity() {
  float sensorDensity = (vibration * 0.4f) + (pressureDelta * 0.3f) + (proximity * 0.003f);
  if (aiDataAvailable) {
    float combinedDensity = (sensorDensity * 0.6f) + (aiDensity * 0.4f);
    return (combinedDensity > 1.0f) ? 1.0f : combinedDensity;
  }
  return (sensorDensity > 1.0f) ? 1.0f : sensorDensity;
}

float calculateAICrowdCount() {
  if (aiDataAvailable && aiCrowdCount > 0) {
    float sensorFactor = 1.0f + (vibration * 0.5f) + (proximity * 0.01f);
    return aiCrowdCount * sensorFactor;
  }
  float estimatedCount = (proximity * 0.2f) + (vibration * 10.0f) + (pressureDelta * 2.0f);
  return (estimatedCount > 0.0f) ? estimatedCount : 0.0f;
}

String getAIRiskAssessment() {
  if (aiDataAvailable) return aiRiskLevel;
  float totalRisk = vibration + (pressureDelta * 0.5f) + (proximity * 0.01f);
  if (totalRisk > 0.8f) return "CRITICAL";
  if (totalRisk > 0.5f) return "HIGH";
  if (totalRisk > 0.3f) return "MEDIUM";
  return "LOW";
}

float calculateEnhancedCRI() {
  float sensorCri = (vibration * 0.4f) + (pressureDelta * 0.3f) + (proximity * 0.002f);
  float tempFactor = (temperature > 30.0f) ? 1.2f : 1.0f;
  sensorCri *= tempFactor;
  
  if (aiDataAvailable) {
    float enhancedCri = (sensorCri * 0.4f) + (aiCri * 0.6f);
    if (aiRiskLevel == "CRITICAL") enhancedCri *= 1.5f;
    else if (aiRiskLevel == "HIGH") enhancedCri *= 1.3f;
    else if (aiRiskLevel == "MEDIUM") enhancedCri *= 1.1f;
    return (enhancedCri > 1.0f) ? 1.0f : enhancedCri;
  }
  return (sensorCri > 1.0f) ? 1.0f : sensorCri;
}

/* ========== DISPLAY ========== */
void displayStatus() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("MYOSA ");
  display.print(currentMode == MODE_DEMO ? "DEMO" : "LIVE");
  
  display.setCursor(85, 0);
  if (currentStatus == "SAFE") {
    display.print("[OK]");
  } else if (currentStatus == "WARNING") {
    display.print("[!]");
  } else if (currentStatus == "CRITICAL") {
    display.print("[X]");
  } else {
    display.print("[?]");
  }
  
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  
  display.setCursor(0, 12);
  display.print("CRI: ");
  display.print(cri, 2);
  
  int barWidth = (int)(cri * 60);
  display.drawRect(40, 12, 62, 6, SSD1306_WHITE);
  if (barWidth > 0) {
    display.fillRect(41, 13, barWidth, 4, SSD1306_WHITE);
  }
  
  display.setCursor(0, 22);
  display.print("VIB:");
  display.print(vibration, 2);
  
  display.setCursor(64, 22);
  display.print("PROX:");
  display.print(proximity);
  
  display.setCursor(0, 32);
  display.print("PRES:");
  display.print(pressureDelta, 1);
  
  display.setCursor(64, 32);
  display.print("TEMP:");
  display.print(temperature, 1);
  display.print("C");
  
  display.setCursor(0, 42);
  display.print("Sensors: ");
  display.print(mpuReady ? "M" : "m");
  display.print(bmpReady ? "B" : "b");
  display.print(apdsReady ? "A" : "a");
  
  display.setCursor(64, 42);
  display.print("Net: ");
  display.print(WiFi.status() == WL_CONNECTED ? "W" : "w");
  display.print(mqtt.connected() ? "M" : "m");
  
  if (manualOverride) {
    display.setCursor(0, 52);
    display.print("[OVERRIDE]");
  }
  
  display.display();
}

/* ========== BUZZER ========== */
void handleBuzzer() {
  digitalWrite(BUZZER_PIN, currentStatus == "CRITICAL" ? HIGH : LOW);
}

/* ========== WIFI + MQTT ========== */
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

void connectMQTT() {
  if (!mqtt.connected()) {
    if (mqtt.connect(CLIENT_ID)) {
      mqtt.subscribe(MQTT_CONTROL_TOPIC);
      Serial.println("MQTT connected!");
    }
  }
}

/* ========== MQTT CALLBACK ========== */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  String command = doc["command"] | "";
  command.toUpperCase();

  if (command == "MODE") {
    String value = doc["value"] | "";
    value.toUpperCase();
    if (value == "LIVE") currentMode = MODE_LIVE;
    if (value == "DEMO") currentMode = MODE_DEMO;
    manualOverride = false;
  }

  if (command == "DEMO_INPUT" && currentMode == MODE_DEMO) {
    demoVibration      = doc["vibration"]      | demoVibration;
    demoPressureDelta  = doc["pressure_delta"] | demoPressureDelta;
    demoProximity      = doc["proximity"]      | demoProximity;
    demoTemperature    = doc["temperature"]    | demoTemperature;
  }

  if (command == "SET_STATUS") {
    currentStatus = doc["status"].as<String>();
    manualOverride = true;
    overrideTime = millis();
  }

  if (command == "AI_UPDATE") {
    aiCrowdCount = doc["ai_crowd_count"] | aiCrowdCount;
    aiDensity = doc["ai_density"] | aiDensity;
    aiCri = doc["ai_cri"] | aiCri;
    String newRiskLevel = doc["combined_risk"];
    if (newRiskLevel != "") aiRiskLevel = newRiskLevel;
    aiDataAvailable = true;
    Serial.println("AI data received and integrated");
  }

  if (command == "AUTO") {
    manualOverride = false;
  }
}

/* ========== CALIBRATION ========== */
void calibrateSensors() {
  display.clearDisplay();
  display.println("CALIBRATING...");
  display.println("KEEP STILL");
  display.display();

  float vibSum = 0;
  double pressSum = 0;
  long proxSum = 0;
  int validReadings = 0;

  for (int i = 0; i < 50; i++) {
    if (mpuReady) {
      sensors_event_t a, g, t;
      if (mpu.getEvent(&a, &g, &t)) {
        float accel = sqrt(
          a.acceleration.x * a.acceleration.x +
          a.acceleration.y * a.acceleration.y +
          a.acceleration.z * a.acceleration.z
        );
        vibSum += abs(accel - 9.8f);
        validReadings++;
      }
    }

    if (bmpReady) {
      float p = bmp.readPressure();
      if (p > 0) {
        pressSum += p;
      }
    }

    if (apdsReady) {
      uint8_t pr = 0;
      if (apds.readProximity(pr)) {
        proxSum += pr;
      }
    }

    delay(50);
  }

  if (validReadings > 0) {
    vibBaseline = vibSum / validReadings;
  }
  pressBaseline = pressSum / 50.0;
  proxBaseline = proxSum / 50;

  if (pressBaseline < 50000) {
    pressBaseline = 101325.0;
  }
  
  Serial.println("Calibration complete");
}

/* ========== SENSOR READ ========== */
void readSensors() {
  if (currentMode == MODE_DEMO) {
    vibration     = demoVibration;
    pressureDelta = demoPressureDelta;
    proximity     = demoProximity;
    temperature   = demoTemperature;
    return;
  }

  if (mpuReady) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      float totalAccel = sqrt(
        a.acceleration.x * a.acceleration.x +
        a.acceleration.y * a.acceleration.y +
        a.acceleration.z * a.acceleration.z
      );

      float dynamicAccel = abs(totalAccel - 9.8f) - vibBaseline;
      if (dynamicAccel < 0.0f) dynamicAccel = 0.0f;

      vibration = dynamicAccel / 3.0f;
      if (vibration < VIBRATION_NOISE_FLOOR) vibration = 0.0f;
    } else {
      vibration = 0.0f;
    }
  } else {
    vibration = 0.1f;
  }

  if (bmpReady) {
    temperature = bmp.readTemperature();
    
    float currentPressure = bmp.readPressure();
    if (currentPressure > 0) {
      pressureDelta = abs(currentPressure - pressBaseline) / 100.0f;
      if (pressureDelta < PRESSURE_NOISE_FLOOR) pressureDelta = 0.0f;
    } else {
      pressureDelta = 0.0f;
    }
  } else {
    temperature = 25.0f;
    pressureDelta = 0.2f;
  }

  if (apdsReady) {
    uint8_t pr = 0;
    if (apds.readProximity(pr)) {
      int proxDelta = abs((int)pr - proxBaseline);
      proximity = proxDelta > PROX_NOISE_FLOOR ? proxDelta : 0;
    } else {
      proximity = 0;
    }
  } else {
    proximity = 15;
  }
}

/* ========== STATUS UPDATE ========== */
void updateStatus() {
  if (manualOverride && (millis() - overrideTime < overrideDuration)) return;
  manualOverride = false;
  
  cri = calculateEnhancedCRI();
  String aiRisk = getAIRiskAssessment();
  
  if (cri >= 0.8f || aiRisk == "CRITICAL") {
    currentStatus = "CRITICAL";
  } else if (cri >= 0.5f || aiRisk == "HIGH") {
    currentStatus = "WARNING";
  } else if (cri >= 0.2f || aiRisk == "MEDIUM") {
    currentStatus = "CAUTION";
  } else {
    currentStatus = "SAFE";
  }
}

/* ========== MQTT PUBLISH ========== */
void publishStatus() {
  if (!mqtt.connected()) return;

  StaticJsonDocument<512> doc;
  doc["device_id"] = "CMU_01";
  doc["timestamp"] = millis();
  doc["mode"] = (currentMode == MODE_DEMO) ? "DEMO" : "LIVE";
  doc["status"] = currentStatus;
  doc["cri"] = cri;
  
  doc["sensors"]["vibration"] = vibration;
  doc["sensors"]["pressure_delta"] = pressureDelta;
  doc["sensors"]["proximity"] = proximity;
  doc["sensors"]["temperature"] = temperature;
  
  if (aiDataAvailable) {
    doc["ai_analysis"]["crowd_count"] = calculateAICrowdCount();
    doc["ai_analysis"]["density"] = calculateAICrowdDensity();
    doc["ai_analysis"]["risk_level"] = getAIRiskAssessment();
    doc["ai_analysis"]["ai_cri"] = aiCri;
  }
  
  doc["system"]["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  doc["system"]["mqtt_connected"] = mqtt.connected();
  doc["system"]["sensors_ready"] = mpuReady && bmpReady && apdsReady;
  
  String payload;
  serializeJson(doc, payload);
  mqtt.publish(MQTT_STATUS_TOPIC, payload.c_str());
}

/* ========== SENSOR INIT ========== */
void initSensors() {
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuReady = true;
    Serial.println("MPU6050 OK");
  }

  if (bmp.begin()) {
    bmpReady = true;
    Serial.println("BMP085 OK");
  }

  if (apds.init() && apds.enableProximitySensor(false)) {
    apdsReady = true;
    Serial.println("APDS9960 OK");
  }
}

/* ========== SETUP ========== */
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display failed");
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MYOSA Initializing...");
  display.display();
  
  initSensors();
  connectWiFi();
  
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  connectMQTT();
  
  calibrateSensors();
  currentStatus = "SAFE";
  Serial.println("MYOSA Ready with AI Integration");
}

/* ========== MAIN LOOP ========== */
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();
  
  readSensors();
  updateStatus();
  displayStatus();
  handleBuzzer();
  
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish > 2000) {
    publishStatus();
    lastPublish = millis();
  }
  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    Serial.printf("Status: %s, CRI: %.3f, AI: %s\n", 
                  currentStatus.c_str(), cri, 
                  aiDataAvailable ? "Available" : "Unavailable");
    lastDebug = millis();
  }
  
  delay(100);
}