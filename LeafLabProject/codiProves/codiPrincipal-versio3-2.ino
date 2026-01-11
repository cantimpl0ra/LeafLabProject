/***********************************************************************
  LEAF LAB – v5.1 FINAL MODIFICADA
  Umbrales: Suelo < 450 (Bomba), F3/F4 T > 26°C (IoT Relay)
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_seesaw.h>

#define DEBUG_SERIAL 1

MKRIoTCarrier carrier;

/* ================= WIFI + MQTT CONFIG ================= */
char WIFI_SSID[32];
char WIFI_PASS[32];
char MQTT_HOST[32];
const int MQTT_PORT = 1883;
WiFiClient wifi;
MqttClient mqtt(wifi);

/* ================= MQTT TOPICS ================= */
const char* TOPIC_CMD_PHASE  = "leaflab/cmd/phase";
const char* TOPIC_CMD_PUMP   = "leaflab/cmd/pump";
const char* TOPIC_CMD_HUMID  = "leaflab/cmd/humid";
const char* TOPIC_CMD_HEATER = "leaflab/cmd/heater";
const char* TOPIC_CMD_IOT    = "leaflab/cmd/iot";

const char* TOPIC_STATE_PHASE     = "leaflab/state/phase";
const char* TOPIC_STATE_ACTUATORS = "leaflab/state/actuators";
const char* TOPIC_STATE_SENSORS   = "leaflab/state/sensors";
const char* TOPIC_STATE_HEALTH    = "leaflab/state/health";

/* ================= I2C / PAHUB / PCF ================= */
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define CH_PCF     5
#define CH_SOIL    3
#define SOIL_ADDR  0x36

bool pahubPresent = false;
uint8_t pahubCurrentChannel = 255;
uint8_t carrierChannel = 255;

/* ================= IOT RELAY ================= */
const int PIN_IOT_RELAY = 2;

/* ================= I2C RELAY (PCF8574) mapping ================= */
const uint8_t RELAYNUM_PUMP   = 2;
const uint8_t RELAYNUM_HEATER = 7;
const uint8_t RELAYNUM_HUMID  = 8;

uint8_t pcfState = 0x00;

/* ================= PHASE + ACTUATORS ================= */
int phase = 1;

struct Actuator { bool manual; bool state; };
Actuator pump   = {false, false};
Actuator humid  = {false, false};
Actuator heater = {false, false};
Actuator iot    = {false, false};

/* ================= HYSTERESIS ================= */
const float TEMP_HYS = 0.4;
const float HUM_HYS  = 2.5;
const float SOIL_HYS = 30.0; 

/* ================= TIMING ================= */
const unsigned long BTN_DEBOUNCE_MS     = 450;
const unsigned long MQTT_RECONNECT_MS   = 3000;
const unsigned long PUBLISH_SENSORS_MS  = 1000;
const unsigned long PUBLISH_STATUS_MS   = 1200;
const unsigned long PUBLISH_HEALTH_MS   = 5000;
const unsigned long SCREEN_REFRESH_MS   = 500;

/* ================= STATE VARS ================= */
bool lastTouch[4] = {false, false, false, false};
unsigned long lastToggleMs[4] = {0, 0, 0, 0};
unsigned long lastMqttAttemptMs = 0;
unsigned long lastPubSensorsMs  = 0;
unsigned long lastPubStatusMs   = 0;
unsigned long lastPubHealthMs   = 0;
unsigned long lastScreenMs      = 0;

bool pendingPhase = false;
int  pendingPhaseValue = 1;
bool pendingTogglePump   = false;
bool pendingToggleHumid  = false;
bool pendingToggleHeater = false;
bool pendingToggleIot    = false;

Adafruit_seesaw ss;
bool soilPresent = false;
uint16_t lastSoil = 0;

/* ================= SERIAL INPUT HELPERS ================= */
static inline bool isLineEnd(char c) { return (c == '\n' || c == '\r'); }

void readLine(char* buffer, size_t maxLen) {
  size_t idx = 0;
  while (!Serial.available()) {}
  while (idx < maxLen - 1) {
    while (!Serial.available()) {}
    char c = Serial.read();
    if (isLineEnd(c)) break;
    buffer[idx++] = c;
  }
  buffer[idx] = '\0';
  while (Serial.available()) {
    char c = Serial.peek();
    if (isLineEnd(c)) Serial.read();
    else break;
  }
}

void requestNetworkConfigFromSerial() {
  Serial.println("=== LEAF LAB SETUP ===");
  Serial.println("Enter WIFI SSID:");
  readLine(WIFI_SSID, sizeof(WIFI_SSID));
  Serial.println("Enter WIFI PASSWORD:");
  readLine(WIFI_PASS, sizeof(WIFI_PASS));
  Serial.println("Enter MQTT HOST:");
  readLine(MQTT_HOST, sizeof(MQTT_HOST));
}

/* ================= I2C HELPERS ================= */
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void pahubSelect(uint8_t ch) {
  if (!pahubPresent) return;
  if (pahubCurrentChannel == ch) return;
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  pahubCurrentChannel = ch;
  delay(2);
}

inline void useCarrierBus() { if (pahubPresent && carrierChannel != 255) pahubSelect(carrierChannel); }
inline void usePcfBus()     { if (pahubPresent) pahubSelect(CH_PCF); }
inline void useSoilBus()    { if (pahubPresent) pahubSelect(CH_SOIL); }

int scanCountExcluding2(uint8_t excludeA, uint8_t excludeB) {
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == excludeA || addr == excludeB) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) count++;
  }
  return count;
}

void autodetectI2CLayout() {
  pahubPresent = i2cPing(PAHUB_ADDR);
  if (!pahubPresent) { carrierChannel = 255; return; }

  int counts[8];
  for (uint8_t ch = 0; ch < 8; ch++) {
    pahubSelect(ch);
    counts[ch] = scanCountExcluding2(PCF_ADDR, SOIL_ADDR);
  }

  bool allEqual = true;
  for (uint8_t ch = 1; ch < 8; ch++) { if (counts[ch] != counts[0]) { allEqual = false; break; } }

  if (allEqual && counts[0] > 0) {
    carrierChannel = 255;
  } else {
    int best = -1; uint8_t bestCh = 0;
    for (uint8_t ch = 0; ch < 8; ch++) { if (counts[ch] > best) { best = counts[ch]; bestCh = ch; } }
    carrierChannel = bestCh;
  }
  useCarrierBus();
}

/* ================= OUTPUT MANAGEMENT ================= */
void writePCF() {
  usePcfBus();
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();
  useCarrierBus();
}

void setI2CRelay(uint8_t relayNum, bool on) {
  if (relayNum < 1 || relayNum > 8) return;
  uint8_t bitIndex = relayNum - 1;
  if (on)  pcfState |=  (1 << bitIndex);
  else     pcfState &= ~(1 << bitIndex);
  writePCF();
}

void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);
  digitalWrite(PIN_IOT_RELAY, iot.state ? HIGH : LOW);
}

/* ================= AUTOMATION LOGIC ================= */
bool belowHys(bool s, float v, float lim, float h) {
  if (!s && v < lim) return true;
  if ( s && v > lim + h) return false;
  return s;
}
bool aboveHys(bool s, float v, float lim, float h) {
  if (!s && v > lim) return true;
  if ( s && v < lim - h) return false;
  return s;
}

void applyAutomation(float t, float h, uint16_t soil) {
  // Bomba: Umbral 450
  if (!pump.manual) {
    pump.state = belowHys(pump.state, (float)soil, 450.0, SOIL_HYS);
  }

  // Calentador
  if (!heater.manual) {
    float lim = (phase <= 2) ? 20.0 : 18.0;
    heater.state = belowHys(heater.state, t, lim, TEMP_HYS);
  }

  // Humidificador
  if (!humid.manual) {
    float lim = (phase == 1) ? 65.0 : (phase == 2) ? 55.0 : (phase == 3) ? 50.0 : 40.0;
    humid.state = belowHys(humid.state, h, lim, HUM_HYS);
  }

  // IoT Relay (F1/F2 Humedad, F3/F4 Temperatura > 26)
  if (!iot.manual) {
    if (phase == 1)      iot.state = aboveHys(iot.state, h, 75.0, HUM_HYS);
    else if (phase == 2) iot.state = aboveHys(iot.state, h, 70.0, HUM_HYS);
    else                 iot.state = aboveHys(iot.state, t, 26.0, TEMP_HYS);
  }
}

/* ================= INTERFACE & BUTTONS ================= */
void toggleManual(Actuator &a) {
  a.manual = !a.manual;
  if (!a.manual) a.state = false; 
  else a.state = true;
}

void handleButtons() {
  useCarrierBus();
  carrier.Buttons.update();
  bool t[4] = { carrier.Buttons.getTouch(TOUCH0), carrier.Buttons.getTouch(TOUCH1), 
                carrier.Buttons.getTouch(TOUCH2), carrier.Buttons.getTouch(TOUCH3) };
  unsigned long now = millis();
  for(int i=0; i<4; i++) {
    if (t[i] && !lastTouch[i] && (now - lastToggleMs[i] > BTN_DEBOUNCE_MS)) {
      if(i==0) toggleManual(pump); if(i==1) toggleManual(humid);
      if(i==2) toggleManual(heater); if(i==3) toggleManual(iot);
      lastToggleMs[i] = now;
    }
    lastTouch[i] = t[i];
  }
}

/* ================= SCREEN ================= */
void drawStaticUI() {
  carrier.display.fillScreen(ST77XX_BLACK);
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);
  carrier.display.setCursor(0, 0); carrier.display.println("LEAF LAB");
  carrier.display.setCursor(0, 30);  carrier.display.print("FASE:");
  carrier.display.setCursor(0, 55);  carrier.display.print("T:");
  carrier.display.setCursor(0, 80);  carrier.display.print("H:");
  carrier.display.setCursor(0, 105); carrier.display.print("SOIL:");
  carrier.display.setCursor(0, 135); carrier.display.print("PUMP:");
  carrier.display.setCursor(0, 155); carrier.display.print("HUMI:");
  carrier.display.setCursor(0, 175); carrier.display.print("HEAT:");
  carrier.display.setCursor(0, 195); carrier.display.print("IOT:");
}

void printValueAt(int x, int y, const String &val) {
  carrier.display.fillRect(x, y, 140, 18, ST77XX_BLACK);
  carrier.display.setCursor(x, y); carrier.display.print(val);
}

void updateScreen(float t, float h, uint16_t soil) {
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);
  printValueAt(70, 30,  String(phase));
  printValueAt(40, 55,  String(t, 1) + "C");
  printValueAt(40, 80,  String(h, 1) + "%");
  printValueAt(80, 105, String(soil));
  auto m = [](const Actuator &a) { return String(a.manual ? "MAN" : "AUT") + " " + (a.state ? "ON" : "OFF"); };
  printValueAt(70, 135, m(pump)); printValueAt(70, 155, m(humid));
  printValueAt(70, 175, m(heater)); printValueAt(70, 195, m(iot));
}

/* ================= MQTT PUBLISH ================= */
void publishPhase() {
  if (!mqtt.connected()) return;
  mqtt.beginMessage(TOPIC_STATE_PHASE); mqtt.print(phase); mqtt.endMessage();
}

void publishActuators() {
  if (!mqtt.connected()) return;
  mqtt.beginMessage(TOPIC_STATE_ACTUATORS);
  mqtt.print("{");
  auto p = [](const char* n, const Actuator &a, bool last=false) {
    mqtt.print("\""); mqtt.print(n); mqtt.print("\":{\"manual\":");
    mqtt.print(a.manual ? "true":"false"); mqtt.print(",\"on\":");
    mqtt.print(a.state ? "true":"false"); mqtt.print(last ? "}" : "},");
  };
  p("pump", pump); p("humid", humid); p("heater", heater); p("iot", iot, true);
  mqtt.print("}"); mqtt.endMessage();
}

void publishSensors(float temp, float hum, uint16_t soil) {
  if (!mqtt.connected()) return;
  mqtt.beginMessage(TOPIC_STATE_SENSORS);
  mqtt.print("{\"phase\":"); mqtt.print(phase);
  mqtt.print(",\"temp\":");  mqtt.print(temp, 1);
  mqtt.print(",\"hum\":");   mqtt.print(hum, 1);
  mqtt.print(",\"soil\":");  mqtt.print(soil); mqtt.print("}");
  mqtt.endMessage();
}

void onMqttMessage(int size) {
  String topic = mqtt.messageTopic();
  String payload;
  while (mqtt.available()) payload += (char)mqtt.read();
  payload.trim();
  if (topic == TOPIC_CMD_PHASE) {
    int p = payload.toInt(); if (p >= 1 && p <= 4) { pendingPhaseValue = p; pendingPhase = true; }
  } else if (payload == "toggle") {
    if (topic == TOPIC_CMD_PUMP)   pendingTogglePump = true;
    if (topic == TOPIC_CMD_HUMID)  pendingToggleHumid = true;
    if (topic == TOPIC_CMD_HEATER) pendingToggleHeater = true;
    if (topic == TOPIC_CMD_IOT)    pendingToggleIot = true;
  }
}

/* ================= SETUP & LOOP ================= */
void setup() {
  Serial.begin(115200);
  requestNetworkConfigFromSerial();
  Wire.begin();
  autodetectI2CLayout();
  pinMode(PIN_IOT_RELAY, OUTPUT);
  useCarrierBus();
  carrier.begin();
  
  // Init Soil
  useSoilBus();
  if (i2cPing(SOIL_ADDR) && ss.begin(SOIL_ADDR)) soilPresent = true;
  
  useCarrierBus();
  drawStaticUI();
}

void loop() {
  handleButtons();
  if (WiFi.status() != WL_CONNECTED) WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (mqtt.connected()) mqtt.poll();
  else {
    if (millis() - lastMqttAttemptMs > MQTT_RECONNECT_MS) {
      lastMqttAttemptMs = millis();
      if (mqtt.connect(MQTT_HOST, MQTT_PORT)) {
        mqtt.onMessage(onMqttMessage);
        mqtt.subscribe(TOPIC_CMD_PHASE); mqtt.subscribe(TOPIC_CMD_PUMP);
        mqtt.subscribe(TOPIC_CMD_HUMID); mqtt.subscribe(TOPIC_CMD_HEATER);
        mqtt.subscribe(TOPIC_CMD_IOT);
        publishPhase(); publishActuators();
      }
    }
  }

  if (pendingPhase) { phase = pendingPhaseValue; pendingPhase = false; publishPhase(); }
  if (pendingTogglePump)   { toggleManual(pump);   pendingTogglePump = false;   publishActuators(); }
  if (pendingToggleHumid)  { toggleManual(humid);  pendingToggleHumid = false;  publishActuators(); }
  if (pendingToggleHeater) { toggleManual(heater); pendingToggleHeater = false; publishActuators(); }
  if (pendingToggleIot)    { toggleManual(iot);    pendingToggleIot = false;    publishActuators(); }

  useCarrierBus();
  float t = carrier.Env.readTemperature();
  float h = carrier.Env.readHumidity();
  useSoilBus();
  uint16_t s = soilPresent ? ss.touchRead(0) : lastSoil;
  lastSoil = s;
  useCarrierBus();

  applyAutomation(t, h, s);
  applyOutputs();

  unsigned long now = millis();
  if (now - lastPubSensorsMs > PUBLISH_SENSORS_MS) { publishSensors(t, h, s); lastPubSensorsMs = now; }
  if (now - lastPubStatusMs > PUBLISH_STATUS_MS)   { publishActuators(); lastPubStatusMs = now; }
  if (now - lastScreenMs > SCREEN_REFRESH_MS)      { updateScreen(t, h, s); lastScreenMs = now; }
  
  delay(20);
}