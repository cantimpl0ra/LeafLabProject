/***********************************************************************
  LEAF LAB – v5.1 FINAL
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// Soil sensor (Adafruit seesaw / STEMMA soil sensor)
#include <Adafruit_seesaw.h>

// SD (NO SPI.begin / NO SD.begin)
#include <SD.h>

#define DEBUG_SERIAL 1

MKRIoTCarrier carrier;

/* ================= WIFI + MQTT (CONFIG VIA SERIAL) ================= */
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

// Ajusta SOLO si tu PCF no está en este canal del PaHub:
#define CH_PCF     5

// Soil sensor on PaHub channel 3 (as you said)
#define CH_SOIL    3

// Default I2C addr for Adafruit seesaw soil sensor
#define SOIL_ADDR  0x36

bool pahubPresent = false;
uint8_t pahubCurrentChannel = 255;

// carrierChannel:
// 255 = upstream (no channel switching needed for carrier)
// 0..7 = downstream channel where the carrier devices are visible
uint8_t carrierChannel = 255;

/*
  If your dehumidifier and "pinguino” share the same output and cannot be separated:
  - true  : F3/F4 keep IoT relay ON always (dehum always on)
  - false : F3/F4 IoT relay only used for cooling when T>22
*/
#define IOT_SHARED_DEHUM_AND_COOL true

/* ================= I2C RELAY (PCF8574) mapping ================= */
const uint8_t RELAYNUM_PUMP   = 2;
const uint8_t RELAYNUM_IOT    = 3;   // ✅ IoT relay ahora es el #3 (PCF)
const uint8_t RELAYNUM_HEATER = 7;
const uint8_t RELAYNUM_HUMID  = 8;

// PCF direct logic: 1=ON, 0=OFF
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

/* ================= TIMING ================= */
const unsigned long BTN_DEBOUNCE_MS     = 450;
const unsigned long MQTT_RECONNECT_MS   = 3000;
const unsigned long PUBLISH_SENSORS_MS  = 1000;
const unsigned long PUBLISH_STATUS_MS   = 1200;
const unsigned long PUBLISH_HEALTH_MS   = 5000;
const unsigned long SCREEN_REFRESH_MS   = 500;

/* ================= SD LOGGING ================= */
const unsigned long SD_LOG_MS = 120000; // 2 minutos
unsigned long lastSdLogMs = 0;

const char* SD_LOG_FILE = "leaflab.csv";
bool sdOk = false;

/* ================= BUTTONS ================= */
bool lastTouch[4] = {false, false, false, false};
unsigned long lastToggleMs[4] = {0, 0, 0, 0};

/* ================= TIMERS ================= */
unsigned long lastMqttAttemptMs = 0;
unsigned long lastPubSensorsMs  = 0;
unsigned long lastPubStatusMs   = 0;
unsigned long lastPubHealthMs   = 0;
unsigned long lastScreenMs      = 0;

/* ================= MQTT COMMAND FLAGS ================= */
bool pendingPhase = false;
int  pendingPhaseValue = 1;
bool pendingTogglePump   = false;
bool pendingToggleHumid  = false;
bool pendingToggleHeater = false;
bool pendingToggleIot    = false;

/* ================= SOIL SENSOR (I2C) ================= */
Adafruit_seesaw ss;
bool soilPresent = false;
uint16_t lastSoil = 0;

/* ================= PROTOTYPES ================= */
void onMqttMessage(int size);
void updateScreen(float t, float h, uint16_t soil);
void applyAutomation(float t, float h, uint16_t soil);

void publishPhase();
void publishActuators();
void publishSensors(float temp, float hum, uint16_t soil);
void publishHealth();

uint16_t readSoil();

// SD
void sdInit();                 // (solo SD.exists/open)
void sdLogRow(float t, float h, uint16_t soil);

/* ================= SERIAL INPUT ================= */
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

  Serial.println("Enter MQTT HOST (IP or hostname):");
  readLine(MQTT_HOST, sizeof(MQTT_HOST));

  Serial.println("Configuration received.");
}

/* ================= LOW-LEVEL I2C HELPERS ================= */
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

inline void useCarrierBus() {
  if (!pahubPresent) return;
  if (carrierChannel == 255) return; // upstream
  pahubSelect(carrierChannel);
}
inline void usePcfBus() {
  if (!pahubPresent) return;
  pahubSelect(CH_PCF);
}
inline void useSoilBus() {
  if (!pahubPresent) return;
  pahubSelect(CH_SOIL);
}

/* ================= I2C SCAN (current bus) ================= */
int scanCountExcluding2(uint8_t excludeA, uint8_t excludeB) {
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == excludeA || addr == excludeB) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) count++;
  }
  return count;
}

/* ================= AUTODETECT PAHUB + CARRIER LOCATION ================= */
void autodetectI2CLayout() {
  pahubPresent = i2cPing(PAHUB_ADDR);

#if DEBUG_SERIAL
  Serial.print("[I2C] PaHub present: ");
  Serial.println(pahubPresent ? "YES" : "NO");
#endif

  if (!pahubPresent) {
    carrierChannel = 255; // direct bus
    return;
  }

  int counts[8];
  for (uint8_t ch = 0; ch < 8; ch++) {
    pahubSelect(ch);
    counts[ch] = scanCountExcluding2(PCF_ADDR, SOIL_ADDR);
#if DEBUG_SERIAL
    Serial.print("[I2C] ch"); Serial.print(ch);
    Serial.print(" devices(excl 0x27,0x36): ");
    Serial.println(counts[ch]);
#endif
  }

  bool allEqual = true;
  for (uint8_t ch = 1; ch < 8; ch++) {
    if (counts[ch] != counts[0]) { allEqual = false; break; }
  }

  if (allEqual && counts[0] > 0) {
    carrierChannel = 255;
#if DEBUG_SERIAL
    Serial.println("[I2C] Carrier seems UPSTREAM (no channel needed).");
#endif
  } else {
    int best = -1;
    uint8_t bestCh = 0;
    for (uint8_t ch = 0; ch < 8; ch++) {
      if (counts[ch] > best) { best = counts[ch]; bestCh = ch; }
    }
    carrierChannel = bestCh;
#if DEBUG_SERIAL
    Serial.print("[I2C] Carrier seems DOWNSTREAM on channel: ");
    Serial.println(carrierChannel);
#endif
  }

  useCarrierBus();
}

/* ================= PCF8574 RELAYS ================= */
void writePCF() {
  if (pahubPresent) usePcfBus();

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

/* ================= OUTPUTS ================= */
void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);
  setI2CRelay(RELAYNUM_IOT,    iot.state); // ✅ IoT via I2C relay #3
}

/* ================= MANUAL TOGGLE ================= */
void toggleManual(Actuator &a) {
  if (!a.manual) { a.manual = true;  a.state = true;  }
  else           { a.manual = false; a.state = false; }
}

/* ================= BUTTONS (EDGE + DEBOUNCE) ================= */
void handleButtons() {
  useCarrierBus();
  carrier.Buttons.update();

  bool t[4];
  t[0] = carrier.Buttons.getTouch(TOUCH0);
  t[1] = carrier.Buttons.getTouch(TOUCH1);
  t[2] = carrier.Buttons.getTouch(TOUCH2);
  t[3] = carrier.Buttons.getTouch(TOUCH3);

  unsigned long now = millis();

  if (t[0] && !lastTouch[0] && (now - lastToggleMs[0] > BTN_DEBOUNCE_MS)) { toggleManual(pump);   lastToggleMs[0] = now; }
  if (t[1] && !lastTouch[1] && (now - lastToggleMs[1] > BTN_DEBOUNCE_MS)) { toggleManual(humid);  lastToggleMs[1] = now; }
  if (t[2] && !lastTouch[2] && (now - lastToggleMs[2] > BTN_DEBOUNCE_MS)) { toggleManual(heater); lastToggleMs[2] = now; }
  if (t[3] && !lastTouch[3] && (now - lastToggleMs[3] > BTN_DEBOUNCE_MS)) { toggleManual(iot);    lastToggleMs[3] = now; }

  for (int i = 0; i < 4; i++) lastTouch[i] = t[i];
}

/* ================= HYSTERESIS HELPERS ================= */
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

/* ================= AUTOMATION BY PHASE ================= */
void applyAutomation(float t, float h, uint16_t soil) {
  if (!pump.manual) {
    uint16_t thDry = (phase == 1) ? 550 :
                     (phase == 2) ? 600 :
                     (phase == 3) ? 650 : 700;
    pump.state = belowHys(pump.state, (float)soil, (float)thDry, 30.0);
  }

  if (!heater.manual) {
    float lim = (phase <= 2) ? 20.0 : 18.0;
    heater.state = belowHys(heater.state, t, lim, TEMP_HYS);
  }

  if (!humid.manual) {
    float lim = (phase == 1) ? 65.0 :
                (phase == 2) ? 55.0 :
                (phase == 3) ? 50.0 : 40.0;
    humid.state = belowHys(humid.state, h, lim, HUM_HYS);
  }

  if (!iot.manual) {
    if (phase == 1) {
      iot.state = aboveHys(iot.state, h, 75.0, HUM_HYS);
    } else if (phase == 2) {
      iot.state = aboveHys(iot.state, h, 70.0, HUM_HYS);
    } else {
      bool needCool = aboveHys(false, t, 22.0, TEMP_HYS);
#if IOT_SHARED_DEHUM_AND_COOL
      (void)needCool;
      iot.state = true;
#else
      iot.state = needCool;
#endif
    }
  }
}

/* ================= WIFI / MQTT ================= */
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void ensureMQTT() {
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttAttemptMs < MQTT_RECONNECT_MS) return;
  lastMqttAttemptMs = now;

  mqtt.setId("leaflab-mkr");
  if (mqtt.connect(MQTT_HOST, MQTT_PORT)) {
    mqtt.onMessage(onMqttMessage);

    mqtt.subscribe(TOPIC_CMD_PHASE);
    mqtt.subscribe(TOPIC_CMD_PUMP);
    mqtt.subscribe(TOPIC_CMD_HUMID);
    mqtt.subscribe(TOPIC_CMD_HEATER);
    mqtt.subscribe(TOPIC_CMD_IOT);

#if DEBUG_SERIAL
    Serial.println("[MQTT] Connected + subscribed.");
#endif

    publishPhase();
    publishActuators();
    publishHealth();
  }
}

/* ================= MQTT CALLBACK ================= */
void onMqttMessage(int) {
  String topic = mqtt.messageTopic();
  String payload;
  while (mqtt.available()) payload += (char)mqtt.read();
  payload.trim();

#if DEBUG_SERIAL
  Serial.print("[MQTT] "); Serial.print(topic); Serial.print(" -> "); Serial.println(payload);
#endif

  if (topic == TOPIC_CMD_PHASE) {
    int p = payload.toInt();
    if (p >= 1 && p <= 4) { pendingPhaseValue = p; pendingPhase = true; }
    return;
  }

  if (payload == "toggle") {
    if (topic == TOPIC_CMD_PUMP)   pendingTogglePump = true;
    if (topic == TOPIC_CMD_HUMID)  pendingToggleHumid = true;
    if (topic == TOPIC_CMD_HEATER) pendingToggleHeater = true;
    if (topic == TOPIC_CMD_IOT)    pendingToggleIot = true;
  }
}

/* ================= MQTT PUBLISH ================= */
bool mqttReady() { return mqtt.connected(); }

void publishPhase() {
  if (!mqttReady()) return;
  mqtt.beginMessage(TOPIC_STATE_PHASE);
  mqtt.print(phase);
  mqtt.endMessage();
}

void publishActuators() {
  if (!mqttReady()) return;
  mqtt.beginMessage(TOPIC_STATE_ACTUATORS);
  mqtt.print("{");
  mqtt.print("\"pump\":{\"manual\":");   mqtt.print(pump.manual ? "true":"false");
  mqtt.print(",\"on\":");                mqtt.print(pump.state ? "true":"false"); mqtt.print("},");
  mqtt.print("\"humid\":{\"manual\":");  mqtt.print(humid.manual ? "true":"false");
  mqtt.print(",\"on\":");                mqtt.print(humid.state ? "true":"false"); mqtt.print("},");
  mqtt.print("\"heater\":{\"manual\":"); mqtt.print(heater.manual ? "true":"false");
  mqtt.print(",\"on\":");                mqtt.print(heater.state ? "true":"false"); mqtt.print("},");
  mqtt.print("\"iot\":{\"manual\":");    mqtt.print(iot.manual ? "true":"false");
  mqtt.print(",\"on\":");                mqtt.print(iot.state ? "true":"false"); mqtt.print("}");
  mqtt.print("}");
  mqtt.endMessage();
}

void publishSensors(float temp, float hum, uint16_t soil) {
  if (!mqttReady()) return;
  mqtt.beginMessage(TOPIC_STATE_SENSORS);
  mqtt.print("{");
  mqtt.print("\"phase\":"); mqtt.print(phase); mqtt.print(",");
  mqtt.print("\"temp\":");  mqtt.print(temp, 1); mqtt.print(",");
  mqtt.print("\"hum\":");   mqtt.print(hum, 1); mqtt.print(",");
  mqtt.print("\"soil\":");  mqtt.print(soil);
  mqtt.print("}");
  mqtt.endMessage();
}

void publishHealth() {
  if (!mqttReady()) return;
  mqtt.beginMessage(TOPIC_STATE_HEALTH);
  mqtt.print("{");
  mqtt.print("\"uptime_s\":"); mqtt.print(millis() / 1000UL); mqtt.print(",");
  mqtt.print("\"wifi_rssi\":"); mqtt.print(WiFi.RSSI()); mqtt.print(",");
  mqtt.print("\"mqtt\":true,");
  mqtt.print("\"pahub\":"); mqtt.print(pahubPresent ? "true":"false"); mqtt.print(",");
  mqtt.print("\"soil_i2c\":"); mqtt.print(soilPresent ? "true":"false");
  mqtt.print("}");
  mqtt.endMessage();
}

/* ================= SOIL READ ================= */
uint16_t readSoil() {
  if (!soilPresent) return lastSoil;

  if (pahubPresent) useSoilBus();
  uint16_t v = ss.touchRead(0);
  useCarrierBus();

  lastSoil = v;
  return v;
}

/* ================= SD FUNCTIONS (OFICIAL: sin SD.begin) ================= */
void sdInit() {
  // La SD ya está inicializada en carrier.begin()
  // Aquí SOLO comprobamos si podemos abrir/crear el archivo.
  sdOk = false;

  // Si no existe, lo creamos con cabecera
  if (!SD.exists(SD_LOG_FILE)) {
    File f = SD.open(SD_LOG_FILE, FILE_WRITE);
    if (!f) {
#if DEBUG_SERIAL
      Serial.println("[SD] ERROR: cannot create/open log file (SD not detected or wrong SD library).");
#endif
      return;
    }
    f.println("uptime_s,phase,temp_c,hum_pct,soil,pump_on,pump_manual,humid_on,humid_manual,heater_on,heater_manual,iot_on,iot_manual");
    f.close();

#if DEBUG_SERIAL
    Serial.println("[SD] Log file created + header written.");
#endif
    sdOk = true;
    return;
  }

  // Si existe, probamos abrirlo en append
  File f = SD.open(SD_LOG_FILE, FILE_WRITE);
  if (!f) {
#if DEBUG_SERIAL
    Serial.println("[SD] ERROR: cannot open existing log file (SD not detected or wrong SD library).");
#endif
    return;
  }
  f.close();

#if DEBUG_SERIAL
  Serial.println("[SD] Log file exists. SD ready.");
#endif
  sdOk = true;
}

void sdLogRow(float t, float h, uint16_t soil) {
  if (!sdOk) return;

  File f = SD.open(SD_LOG_FILE, FILE_WRITE);
  if (!f) {
#if DEBUG_SERIAL
    Serial.println("[SD] ERROR: open() failed while logging.");
#endif
    sdOk = false;
    return;
  }

  unsigned long uptime_s = millis() / 1000UL;

  f.print(uptime_s); f.print(",");
  f.print(phase);    f.print(",");
  f.print(t, 1);     f.print(",");
  f.print(h, 1);     f.print(",");
  f.print(soil);     f.print(",");

  f.print(pump.state ? 1 : 0);    f.print(",");
  f.print(pump.manual ? 1 : 0);  f.print(",");

  f.print(humid.state ? 1 : 0);  f.print(",");
  f.print(humid.manual ? 1 : 0); f.print(",");

  f.print(heater.state ? 1 : 0); f.print(",");
  f.print(heater.manual ? 1 : 0);f.print(",");

  f.print(iot.state ? 1 : 0);    f.print(",");
  f.print(iot.manual ? 1 : 0);

  f.println();
  f.close();

#if DEBUG_SERIAL
  Serial.println("[SD] Logged row.");
#endif
}

/* ================= SCREEN (NO FLICKER) ================= */
void drawStaticUI() {
  carrier.display.fillScreen(ST77XX_BLACK);
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);

  carrier.display.setCursor(0, 0);
  carrier.display.println("LEAF LAB");

  carrier.display.setTextSize(2);

  carrier.display.setCursor(0, 30);  carrier.display.print("FASE:");
  carrier.display.setCursor(0, 55);  carrier.display.print("T:");
  carrier.display.setCursor(0, 80);  carrier.display.print("H:");
  carrier.display.setCursor(0, 105); carrier.display.print("SOIL:");

  carrier.display.setCursor(0, 135); carrier.display.print("PUMP:");
  carrier.display.setCursor(0, 155); carrier.display.print("HUMI:");
  carrier.display.setCursor(0, 175); carrier.display.print("HEAT:");
  carrier.display.setCursor(0, 195); carrier.display.print("IOT:");
}

void printValueAt(int x, int y, const String &val, int w = 120, int h = 18) {
  carrier.display.fillRect(x, y, w, h, ST77XX_BLACK);
  carrier.display.setCursor(x, y);
  carrier.display.print(val);
}

void updateScreen(float t, float h, uint16_t soil) {
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);

  printValueAt(70, 30,  String(phase));
  printValueAt(40, 55,  String(t, 1) + "C");
  printValueAt(40, 80,  String(h, 1) + "%");
  printValueAt(80, 105, String(soil));

  auto modeTxt = [](const Actuator &a) -> String {
    return String(a.manual ? "MAN" : "AUT") + " " + String(a.state ? "ON" : "OFF");
  };

  printValueAt(70, 135, modeTxt(pump));
  printValueAt(70, 155, modeTxt(humid));
  printValueAt(70, 175, modeTxt(heater));
  printValueAt(70, 195, modeTxt(iot));
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  requestNetworkConfigFromSerial();

  Wire.begin();
  autodetectI2CLayout();

  // ✅ IMPORTANTE: carrier.begin() inicializa SD internamente
  useCarrierBus();
  carrier.begin();

  // ✅ SD: NO SD.begin(), solo abrir/crear archivo
  sdInit();

  // Init PCF (all OFF)
  if (pahubPresent) usePcfBus();
  pcfState = 0x00;
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();
  useCarrierBus();

  // Init soil sensor on CH3
  soilPresent = false;
  if (pahubPresent) useSoilBus();

  if (i2cPing(SOIL_ADDR)) {
    if (ss.begin(SOIL_ADDR)) {
      soilPresent = true;
#if DEBUG_SERIAL
      Serial.println("[I2C] Soil sensor detected + initialized (seesaw).");
#endif
    }
  }

  useCarrierBus();

  drawStaticUI();
  lastScreenMs = 0;
  lastSdLogMs = millis();

#if DEBUG_SERIAL
  Serial.println("[BOOT] Setup done.");
#endif
}

/* ================= LOOP ================= */
void loop() {
  // Buttons
  handleButtons();

  // Network
  ensureWiFi();
  ensureMQTT();
  mqtt.poll();

  // Apply pending MQTT commands safely
  bool changed = false;

  if (pendingPhase) {
    pendingPhase = false;
    phase = pendingPhaseValue;
    changed = true;
  }
  if (pendingTogglePump)   { pendingTogglePump = false;   toggleManual(pump);   changed = true; }
  if (pendingToggleHumid)  { pendingToggleHumid = false;  toggleManual(humid);  changed = true; }
  if (pendingToggleHeater) { pendingToggleHeater = false; toggleManual(heater); changed = true; }
  if (pendingToggleIot)    { pendingToggleIot = false;    toggleManual(iot);    changed = true; }

  // Read sensors (carrier bus)
  useCarrierBus();
  float temp = carrier.Env.readTemperature();
  float hum  = carrier.Env.readHumidity();
  uint16_t soil = readSoil();

  // Automation + Outputs
  applyAutomation(temp, hum, soil);
  applyOutputs();

  // Publish + screen + SD
  unsigned long now = millis();

  if (changed) {
    publishPhase();
    publishActuators();
    lastScreenMs = 0;
  }

  if (now - lastPubSensorsMs > PUBLISH_SENSORS_MS) {
    lastPubSensorsMs = now;
    publishSensors(temp, hum, soil);
  }

  if (now - lastPubStatusMs > PUBLISH_STATUS_MS) {
    lastPubStatusMs = now;
    publishActuators();
  }

  if (now - lastPubHealthMs > PUBLISH_HEALTH_MS) {
    lastPubHealthMs = now;
    publishHealth();
  }

  if (now - lastScreenMs > SCREEN_REFRESH_MS) {
    lastScreenMs = now;
    updateScreen(temp, hum, soil);
  }

  // SD log cada 2 minutos
  if (now - lastSdLogMs >= SD_LOG_MS) {
    lastSdLogMs = now;
    sdLogRow(temp, hum, soil);
  }

  delay(20);
}
