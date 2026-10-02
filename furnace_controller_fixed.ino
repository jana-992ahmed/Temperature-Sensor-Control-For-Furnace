#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "max6675.h"

// ==================== AP Mode Credentials ====================
const char* ssid     = "Furnace_Test"; // Network name you will see on your phone
const char* password = "password123";  // Password to connect

// AP IP address will be 192.168.4.1
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// ==================== Hardware Pins ====================
int thermoSO = 21;
int thermoSCK = 19;
int thermoCS_Top = 5;
int thermoCS_Bot = 18; 
int ssrPin = 25;

MAX6675 thermoTop(thermoSCK, thermoCS_Top, thermoSO);
MAX6675 thermoBot(thermoSCK, thermoCS_Bot, thermoSO);
Preferences preferences;

// ==================== Control Variables ====================
double setpoint;
double rampRatePerSec;
double maxSetpoint;
const double maxTempAbort = 110.0;
const double maxTempDiff = 10.0; // Traffic Cop limit

double Kp = 400.0;
double Ki = 2.0;
double integralSum = 0;
const double integralMax = 1000.0;

unsigned long windowStartTime;
int windowSize = 2000;
double output = 0;
unsigned long lastRampTime = 0;

double currentTempTop = 0;
double currentTempBot = 0;
double currentError = 0;

bool relayState = false;
bool fault = false;
bool isStratified = false; 

WebServer server(80);

double calibrateTemperature(double raw) {
  if (raw < 50) return raw - 1.5;
  if (raw < 70) return raw - 2.0;
  if (raw < 90) return raw - 3.0;
  return raw - 4.0;
}

// ==================== HTML Dashboard ====================
String buildHTML() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Furnace Controller</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{font-family:'Segoe UI',Arial,sans-serif;background:linear-gradient(135deg,#0f2027,#203a43,#2c5364);color:#eee;text-align:center;padding-top:30px;margin:0;min-height:100vh;}";
  html += "h1{color:#4fd1c5;font-size:1.5em;text-shadow:0 0 10px rgba(79,209,197,0.5);margin-bottom:25px;}";
  html += ".card{background:rgba(30,30,40,0.85);display:inline-block;padding:30px 45px;border-radius:16px;box-shadow:0 0 25px rgba(0,0,0,0.5);border:1px solid rgba(255,255,255,0.08);backdrop-filter:blur(6px);}";
  html += ".row{margin:16px 0;padding:12px;border-radius:10px;background:rgba(255,255,255,0.03);}";
  html += ".value{font-size:2em;font-weight:600;}";
  html += ".label{color:#9aa5b1;font-size:0.8em;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:4px;}";
  html += "#setpoint{color:#f6ad55;}";
  html += "#actualTop{color:#63b3ed;}";
  html += "#actualBot{color:#fc8181;}";
  html += "#status{color:#f6e05e;}";
  html += ".on{color:#48bb78;text-shadow:0 0 10px rgba(72,187,120,0.7);font-weight:bold;}";
  html += ".off{color:#718096;font-weight:bold;}";
  html += ".setform{margin-top:26px;padding-top:20px;border-top:1px solid rgba(255,255,255,0.1);}";
  html += ".setform input[type=number]{width:120px;padding:10px;font-size:1.1em;border-radius:8px;border:none;text-align:center;background:#f0f4f8;color:#1a202c;}";
  html += ".setform select{padding:10px;font-size:1.05em;border-radius:8px;border:none;background:#f0f4f8;color:#1a202c;margin-left:5px;}";
  html += ".setform button{padding:10px 22px;font-size:1.05em;margin-left:10px;border:none;border-radius:8px;background:linear-gradient(135deg,#f6ad55,#ed8936);color:#1a202c;font-weight:bold;cursor:pointer;transition:transform 0.15s;}";
  html += ".setform button:hover{transform:scale(1.05);}";
  html += "#msg{margin-top:10px;font-size:0.9em;color:#68d391;min-height:1.2em;}";
  html += "</style></head><body>";
  html += "<h1>ESP32 PI Temperature Controller</h1>";
  html += "<div class='card'>";
  html += "<div class='row'><div class='label'>Ramping Setpoint</div><div class='value' id='setpoint'>--</div></div>";
  html += "<div class='row'><div class='label'>Top Sensor</div><div class='value' id='actualTop'>--</div></div>";
  html += "<div class='row'><div class='label'>Bottom Sensor</div><div class='value' id='actualBot'>--</div></div>";
  html += "<div class='row'><div class='label'>Ramp Status</div><div class='value' id='status' style='font-size:1.2em;'>--</div></div>";
  html += "<div class='row'><div class='label'>Relay State</div><div class='value' id='relay'>--</div></div>";

  html += "<div class='setform'>";
  html += "<div class='label'>Target Temperature (C)</div>";
  html += "<input type='number' id='targetInput' step='0.5' min='0' max='300' placeholder='e.g. 80'>";
  html += "<button onclick='sendTarget()'>Set Target</button>";

  html += "<div class='label' style='margin-top:15px;'>Ramp Rate</div>";
  html += "<input type='number' id='rateInput' step='0.1' min='0' max='300' placeholder='e.g. 5'>";
  html += "<select id='rateUnit'><option value='hour'>Deg/Hour</option><option value='min'>Deg/Min</option></select>";
  html += "<button onclick='sendRate()'>Set Rate</button>";

  html += "<div id='msg'></div>";
  html += "</div>";
  html += "</div>";

  html += "<script>";
  html += "async function update(){";
  html += "  try{";
  html += "    const res = await fetch('/data');";
  html += "    const d = await res.json();";
  html += "    document.getElementById('setpoint').innerText = d.setpoint.toFixed(2) + ' C';";
  html += "    document.getElementById('actualTop').innerText = d.top.toFixed(2) + ' C';";
  html += "    document.getElementById('actualBot').innerText = d.bot.toFixed(2) + ' C';";
  html += "    document.getElementById('status').innerText = d.stratified ? 'PAUSED: Waiting for heat to rise' : 'RAMPING NORMALLY';";
  html += "    const relayEl = document.getElementById('relay');";
  html += "    relayEl.innerText = d.relay ? 'ON' : 'OFF';";
  html += "    relayEl.className = 'value ' + (d.relay ? 'on' : 'off');";
  html += "  } catch(e) { console.log(e); }";
  html += "}";
  html += "async function sendTarget(){";
  html += "  const val = parseFloat(document.getElementById('targetInput').value);";
  html += "  const msg = document.getElementById('msg');";
  html += "  if (isNaN(val)) { msg.style.color='#fc8181'; msg.innerText='Enter a valid number'; return; }";
  html += "  try{";
  html += "    const res = await fetch('/setpoint?value=' + val);";
  html += "    if (res.ok) { msg.style.color='#68d391'; msg.innerText='Target updated'; document.getElementById('targetInput').value=''; }";
  html += "  } catch(e) { msg.style.color='#fc8181'; msg.innerText='Connection error'; }";
  html += "}";
  html += "async function sendRate(){";
  html += "  const val = parseFloat(document.getElementById('rateInput').value);";
  html += "  const unit = document.getElementById('rateUnit').value;";
  html += "  const msg = document.getElementById('msg');";
  html += "  if (isNaN(val)) { msg.style.color='#fc8181'; msg.innerText='Enter a valid number'; return; }";
  html += "  try{";
  html += "    const res = await fetch('/setrate?value=' + val + '&unit=' + unit);";
  html += "    if (res.ok) { msg.style.color='#68d391'; msg.innerText='Rate updated'; document.getElementById('rateInput').value=''; }";
  html += "  } catch(e) { msg.style.color='#fc8181'; msg.innerText='Connection error'; }";
  html += "}";
  html += "setInterval(update, 1000);";
  html += "update();";
  html += "</script>";
  html += "</body></html>";
  return html;
}

// ==================== API Routes ====================
void handleRoot() {
  server.send(200, "text/html", buildHTML());
}

void handleData() {
  String json = "{";
  json += "\"setpoint\":" + String(setpoint, 2) + ",";
  json += "\"top\":" + String(currentTempTop, 2) + ",";
  json += "\"bot\":" + String(currentTempBot, 2) + ",";
  json += "\"stratified\":" + String(isStratified ? "true" : "false") + ",";
  json += "\"relay\":" + String(relayState ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetpoint() {
  if (server.hasArg("value")) {
    double newTarget = server.arg("value").toDouble();
    if (newTarget < 0) newTarget = 0;
    if (newTarget > 300) newTarget = 300;
    maxSetpoint = newTarget;
    preferences.putDouble("target", maxSetpoint);
  }
  server.send(200, "text/plain", "OK");
}

void handleRate() {
  if (server.hasArg("value")) {
    double newRate = server.arg("value").toDouble();
    String unit = server.hasArg("unit") ? server.arg("unit") : "hour";
    if (unit == "hour") {
      rampRatePerSec = newRate / 3600.0;
    } else {
      rampRatePerSec = newRate / 60.0;
    }
    preferences.putDouble("rate", rampRatePerSec);
  }
  server.send(200, "text/plain", "OK");
}

// ==================== Main Setup ====================
void setup() {
  Serial.begin(115200);
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);

  preferences.begin("furnace", false);
  maxSetpoint = preferences.getDouble("target", 100.0);
  rampRatePerSec = preferences.getDouble("rate", 2.0 / 60.0);

  windowStartTime = millis();
  lastRampTime = millis();

  delay(500);
  double rawTop = thermoTop.readCelsius();
  double rawBot = thermoBot.readCelsius();
  
  if (isnan(rawTop) || isnan(rawBot)) {
    fault = true;
    Serial.println("Fault: Sensor missing at boot.");
  } else {
    currentTempTop = calibrateTemperature(rawTop);
    currentTempBot = calibrateTemperature(rawBot);
    setpoint = currentTempTop; 
  }

  // CREATE ACCESS POINT
  Serial.println("Creating Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  Serial.print("Connect to WiFi: ");
  Serial.println(ssid);
  Serial.println("Go to URL: http://192.168.4.1");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setpoint", handleSetpoint);
  server.on("/setrate", handleRate);
  server.begin();
}

// ==================== Main Loop ====================
void loop() {
  server.handleClient();
  unsigned long now = millis();

  if (fault) {
    digitalWrite(ssrPin, LOW);
    relayState = false;
    return;
  }

  if (now - lastRampTime >= 1000) {
    lastRampTime = now;

    double rawTop = thermoTop.readCelsius();
    double rawBot = thermoBot.readCelsius();
    
    if (isnan(rawTop) || isnan(rawBot)) {
      fault = true;
      digitalWrite(ssrPin, LOW);
      relayState = false;
      return;
    }

    double calibTop = calibrateTemperature(rawTop);
    double calibBot = calibrateTemperature(rawBot);
    
    currentTempTop = (0.8 * currentTempTop) + (0.2 * calibTop);
    currentTempBot = (0.8 * currentTempBot) + (0.2 * calibBot);

    if (currentTempTop >= maxTempAbort || currentTempBot >= maxTempAbort) {
      fault = true;
      digitalWrite(ssrPin, LOW);
      relayState = false;
      return;
    }

    // THE TRAFFIC COP LOGIC
    double diff = currentTempBot - currentTempTop;
    if (diff > maxTempDiff) {
      isStratified = true;
    } else {
      isStratified = false;
    }

    // ONLY INCREASE TARGET IF NOT STRATIFIED
    if (!isStratified && setpoint < maxSetpoint) {
      setpoint += rampRatePerSec;
      if (setpoint > maxSetpoint) setpoint = maxSetpoint;
    }

    currentError = setpoint - currentTempTop;

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

  if (now - windowStartTime >= (unsigned long)windowSize) {
    windowStartTime += windowSize;
  }

  if (output > (now - windowStartTime)) {
    digitalWrite(ssrPin, HIGH);
    relayState = true;
  } else {
    digitalWrite(ssrPin, LOW);
    relayState = false;
  }
}