#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "max6675.h"
#include <Preferences.h>

// ==================== 1. INTERNET SETTINGS ====================
const char* ssid = "Ali";
const char* password = "00000000";

// ==================== 2. ADAFRUIT IO SETTINGS ====================
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Jana8887"
#define AIO_KEY         "aio_DLNU63NouH5jigkMArBmlAq5ynDR"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// ==================== FEED NAMES ====================
Adafruit_MQTT_Publish actualTempPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/actual-temp");   
Adafruit_MQTT_Publish setpointPub   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/target-temp");   
Adafruit_MQTT_Publish errorPub      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/error");         
Adafruit_MQTT_Publish rampRatePub   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/ramp-rate");     

Adafruit_MQTT_Subscribe targetSub   = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/setpoint");          
Adafruit_MQTT_Subscribe rampRateSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/change-rate");  

// ==================== 3. HARDWARE & CONTROL SETTINGS ====================
int thermoSO = 21;
int thermoSCK = 19;
int thermoCS_Top = 5;
int thermoCS_Bot = 18;
int ssrPin = 25;

MAX6675 thermoTop(thermoSCK, thermoCS_Top, thermoSO);
MAX6675 thermoBot(thermoSCK, thermoCS_Bot, thermoSO);
Preferences preferences; 

double setpoint;
double rampRatePerSec; 
double maxSetpoint;
const double maxTempAbort = 110.0;
const double maxTempDiff = 10.0;

double Kp = 400.0;
double Ki = 2.0;
double integralSum = 0;
const double integralMax = 1000.0;

unsigned long windowStartTime;
int windowSize = 2000;
double output = 0;
unsigned long lastRampTime = 0;
unsigned long lastMQTTPublish = 0; 

double currentTempTop = 0;
double currentTempBot = 0;
bool fault = false;
bool isStratified = false;

String errorMessage = "OK";

unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
const unsigned long reconnectInterval = 5000; 

double calibrateTemperature(double raw) {
  if (raw < 50) return raw - 1.5;
  if (raw < 70) return raw - 2.0;
  if (raw < 90) return raw - 3.0;
  return raw - 4.0;
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastWifiAttempt < reconnectInterval) return;
  lastWifiAttempt = now;
  WiFi.disconnect();
  WiFi.begin(ssid, password);
}

bool maintainMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false; 

  unsigned long now = millis();
  if (now - lastMqttAttempt < reconnectInterval) return false;
  lastMqttAttempt = now;

  int8_t ret = mqtt.connect();
  if (ret != 0) {
    mqtt.disconnect();
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);

  // Load Saved Values
  preferences.begin("furnace", false);
  maxSetpoint = preferences.getDouble("target", 100.0); 
  double savedRateHr = preferences.getDouble("rateHr", 5.0); 
  rampRatePerSec = savedRateHr / 3600.0;

  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
  }

  mqtt.subscribe(&targetSub);   
  mqtt.subscribe(&rampRateSub); 

  delay(500);
  double rawTop = thermoTop.readCelsius();
  double rawBot = thermoBot.readCelsius();
  if (isnan(rawTop) || isnan(rawBot)) {
    fault = true;
    errorMessage = "Sensor read failed at boot";
  } else {
    currentTempTop = calibrateTemperature(rawTop);
    currentTempBot = calibrateTemperature(rawBot);
    setpoint = currentTempTop;
  }

  windowStartTime = millis();
  lastRampTime = millis();
}

void loop() {
  unsigned long now = millis();

  maintainWiFi();
  bool mqttUp = maintainMQTT();

  // 1. Read Adafruit IO 
  if (mqttUp) {
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(10))) {
      if (subscription == &targetSub) {
        double newTarget = atof((char *)targetSub.lastread);
        if (newTarget < 0) newTarget = 0;
        if (newTarget > 300) newTarget = 300;
        maxSetpoint = newTarget;
        preferences.putDouble("target", maxSetpoint);
      }
      if (subscription == &rampRateSub) {
        double newRateHr = atof((char *)rampRateSub.lastread);
        if (newRateHr < 0) newRateHr = 0;
        if (newRateHr > 300) newRateHr = 300;
        rampRatePerSec = newRateHr / 3600.0;
        preferences.putDouble("rateHr", newRateHr);
      }
    }
  }

  // 2. SAFETY-CRITICAL SECTION 
  if (fault) {
    digitalWrite(ssrPin, LOW);
    if (mqttUp && now - lastMQTTPublish >= 12000) {
      lastMQTTPublish = now;
      errorPub.publish(errorMessage.c_str());
    }
    return;
  }

  // 3. The PID & Ramp Math (Runs every 1 second)
  if (now - lastRampTime >= 1000) {
    lastRampTime = now;

    double rawTop = thermoTop.readCelsius();
    double rawBot = thermoBot.readCelsius();

    if (isnan(rawTop) || isnan(rawBot)) {
      fault = true;
      errorMessage = "Sensor read failed";
      digitalWrite(ssrPin, LOW);
      return;
    }

    double calibTop = calibrateTemperature(rawTop);
    double calibBot = calibrateTemperature(rawBot);

    if (abs(calibTop - currentTempTop) < 10.0) {
      currentTempTop = (0.8 * currentTempTop) + (0.2 * calibTop);
    }
    
    if (abs(calibBot - currentTempBot) < 10.0) {
      currentTempBot = (0.8 * currentTempBot) + (0.2 * calibBot);
    }
    // ==========================================

    if (currentTempTop >= maxTempAbort || currentTempBot >= maxTempAbort) {
      fault = true;
      errorMessage = "Over-temp abort";
      digitalWrite(ssrPin, LOW);
      return;
    }

    // Traffic Cop
    if ((currentTempBot - currentTempTop) > maxTempDiff) isStratified = true;
    else isStratified = false;

    if (!isStratified && setpoint < maxSetpoint) {
      setpoint += rampRatePerSec;
      if (setpoint > maxSetpoint) setpoint = maxSetpoint;
    }

    double currentError = setpoint - currentTempTop;

    if (currentError <= 0) {
      output = 0;
      integralSum = 0;
    } else {
      integralSum += currentError;
      if (integralSum > integralMax) integralSum = integralMax;
      output = (Kp * currentError) + (Ki * integralSum);
    }

    if (output > windowSize) output = windowSize;
    if (output < 0) output = 0;
  }

  // 4. Send Data to Adafruit Cloud 
  if (mqttUp && now - lastMQTTPublish >= 12000) {
    lastMQTTPublish = now;
    
    actualTempPub.publish(currentTempTop);          
    delay(100); 
    setpointPub.publish(setpoint);                  
    delay(100);
    rampRatePub.publish(rampRatePerSec * 3600.0);    
    delay(100);
    errorMessage = isStratified ? "Stratified - ramp paused" : "OK";
    errorPub.publish(errorMessage.c_str());          
  }

  // 5. The PWM Muscle
  if (now - windowStartTime >= (unsigned long)windowSize) {
    windowStartTime += windowSize;
  }

  if (output > (now - windowStartTime)) {
    digitalWrite(ssrPin, HIGH);
  } else {
    digitalWrite(ssrPin, LOW);
  }
}