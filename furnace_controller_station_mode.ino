#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include "max6675.h"


const char* ssid     = "COLLEGE_WIFI_SSID";
const char* password = "COLLEGE_WIFI_PASSWORD";

//fixed local IP 
#define USE_STATIC_IP
#ifdef USE_STATIC_IP
IPAddress local_IP(192, 168, 1, 50);      // <-- pick a free IP on that network's subnet--> ESP32 own address  (to reach the web page)
IPAddress gateway(192, 168, 1, 1);        // <-- that network's router/gateway IP
IPAddress subnet(255, 255, 255, 0);       //range --> it just tells the esp32 "anything starting with 192.168.1.__" is your local neighborhood
IPAddress primaryDNS(8, 8, 8, 8);         // phonebook servers
IPAddress secondaryDNS(8, 8, 4, 4);
#endif


const char* mdnsHostname = "TempControl"; //instead of using IP


int thermoSO = 21;
int thermoCS = 18;
int thermoSCK = 19;
int ssrPin = 25;

MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
Preferences preferences;

double setpoint;
double rampRatePerSec;
double maxSetpoint;
const double maxTempAbort = 110.0;

double Kp = 400.0;
double Ki = 2.0;
double integralSum = 0;
const double integralMax = 1000.0;

unsigned long windowStartTime;
int windowSize = 2000;
double output = 0;
unsigned long lastRampTime = 0;

double currentTemp  = 0;
double currentError = 0;
bool relayState = false;
bool fault = false;

// WiFi reconnect bookkeeping — needed now that we depend on an
// external network instead of hosting our own.
unsigned long lastWifiCheck = 0;
const unsigned long wifiCheckInterval = 5000; // check every 5s

WebServer server(80);

double calibrateTemperature(double raw) {
  if (raw < 50) return raw - 1.5;
  if (raw < 70) return raw - 2.0;
  if (raw < 90) return raw - 3.0;
  return raw - 4.0;
}

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
  html += "#actual{color:#63b3ed;}";
  html += "#error{color:#e879f9;}";
  html += "#pwm{color:#68d391;}";
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
  html += "<div class='row'><div class='label'>Actual Temperature</div><div class='value' id='actual'>--</div></div>";
  html += "<div class='row'><div class='label'>Current Rate</div><div class='value' id='ramp_val'>--</div></div>";
  html += "<div class='row'><div class='label'>Error</div><div class='value' id='error'>--</div></div>";
  html += "<div class='row'><div class='label'>PWM Output (2000 ms window)</div><div class='value' id='pwm'>--</div></div>";
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
  html += "    document.getElementById('actual').innerText = d.actual.toFixed(2) + ' C';";
  html += "    document.getElementById('ramp_val').innerText = d.ramp.toFixed(2) + ' C/hr';";
  html += "    document.getElementById('error').innerText = d.error.toFixed(2) + ' C';";
  html += "    document.getElementById('pwm').innerText = d.pwm.toFixed(0) + ' ms';";
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
  html += "    if (res.ok) { msg.style.color='#68d391'; msg.innerText='Target updated to ' + val.toFixed(1) + ' C'; document.getElementById('targetInput').value=''; }";
  html += "  } catch(e) { msg.style.color='#fc8181'; msg.innerText='Connection error'; }";
  html += "}";
  html += "async function sendRate(){";
  html += "  const val = parseFloat(document.getElementById('rateInput').value);";
  html += "  const unit = document.getElementById('rateUnit').value;";
  html += "  const msg = document.getElementById('msg');";
  html += "  if (isNaN(val)) { msg.style.color='#fc8181'; msg.innerText='Enter a valid number'; return; }";
  html += "  try{";
  html += "    const res = await fetch('/setrate?value=' + val + '&unit=' + unit);";
  html += "    if (res.ok) { msg.style.color='#68d391'; msg.innerText='Rate updated to ' + val + (unit==='hour'?' C/hr':' C/min'); document.getElementById('rateInput').value=''; }";
  html += "  } catch(e) { msg.style.color='#fc8181'; msg.innerText='Connection error'; }";
  html += "}";
  html += "setInterval(update, 1000);";
  html += "update();";
  html += "</script>";
  html += "</body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", buildHTML());
}

void handleData() {
  String json = "{";
  json += "\"setpoint\":" + String(setpoint, 2) + ",";
  json += "\"actual\":" + String(currentTemp, 2) + ",";
  json += "\"error\":" + String(currentError, 2) + ",";
  json += "\"pwm\":" + String(output, 0) + ",";
  json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
  json += "\"ramp\":" + String(rampRatePerSec * 3600.0, 3) + ",";
  json += "\"target\":" + String(maxSetpoint, 2);
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

// ============================================================
// NEW: connect to the real WiFi network (Station mode) and set
// up mDNS. Retries with feedback over Serial so you can debug
// on-site before relying on it remotely.
// ============================================================
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

#ifdef USE_STATIC_IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Static IP config failed, falling back to DHCP.");
  }
#endif

  WiFi.begin(ssid, password);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(mdnsHostname)) {
      Serial.print("mDNS started: http://");
      Serial.print(mdnsHostname);
      Serial.println(".local");
    } else {
      Serial.println("mDNS setup failed.");
    }
  } else {
    Serial.println();
    Serial.println("WiFi connect failed — will keep retrying in loop().");
  }
}

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
  double raw = thermocouple.readCelsius();
  if (isnan(raw)) {
    fault = true;
  } else {
    currentTemp = calibrateTemperature(raw);
    setpoint = currentTemp;
  }

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setpoint", handleSetpoint);
  server.on("/setrate", handleRate);
  server.begin();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // Non-blocking WiFi watchdog: if the link drops (router reboot,
  // signal issue, etc.), try to rejoin every few seconds instead
  // of hanging. This does NOT touch the burner logic at all —
  // heating control below runs independently of WiFi state.
  if (now - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  }

  // ------------------------------------------------------------
  // SAFETY: this fault/abort logic is 100% local and does not
  // depend on WiFi, the webpage, Tailscale, or anything remote.
  // That's intentional — a dropped connection must never be able
  // to leave the SSR stuck on.
  // ------------------------------------------------------------
  if (fault) {
    digitalWrite(ssrPin, LOW);
    relayState = false;
    return;
  }

  if (now - lastRampTime >= 1000) {
    if (setpoint < maxSetpoint) {
      setpoint += rampRatePerSec;
      if (setpoint > maxSetpoint) setpoint = maxSetpoint;
    }
    lastRampTime = now;

    double raw = thermocouple.readCelsius();
    if (isnan(raw)) {
      fault = true;
      digitalWrite(ssrPin, LOW);
      relayState = false;
      return;
    }

    double calib = calibrateTemperature(raw);
    currentTemp = (0.8 * currentTemp) + (0.2 * calib);

    if (currentTemp >= maxTempAbort) {
      fault = true;
      digitalWrite(ssrPin, LOW);
      relayState = false;
      return;
    }

    currentError = setpoint - currentTemp;

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
