/***********************************************************************
  LEAF LAB – v5.4 (IoT relay NO flicker while OFF + Soil calibration + Pump rule)

  CHANGES v5.4 (per your request):
  1) IoT relay MUST NOT flicker when OFF:
     - Added "output latch": we only write to the IoT relay pin when the desired
       state CHANGES (no repeated digitalWrite every loop).
     - Added minimum hold time BOTH directions (min ON time and min OFF time)
       to avoid chatter around thresholds.
     - Kept active-low option.

  2) Soil % calibration (RAW -> %):
     - RAW 300 = DRY (0%)
     - RAW 650 = WET (100%)

  3) Pump rule (ALL phases same):
     - If soilRaw < 450 => pump ON
     - If soilRaw >= 450 => pump OFF
     (No hysteresis; exactly as you asked.)
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_seesaw.h>

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

// carrierChannel: 255 = upstream, 0..7 = downstream
uint8_t carrierChannel = 255;
bool carrierOk = false;

/* ================= IOT RELAY ================= */
const int PIN_IOT_RELAY = 2;

// Many IoT relay boards are active-low
#define IOT_ACTIVE_LOW true

// If dehum + pinguino share SAME relay output:
#define IOT_SHARED_DEHUM_AND_COOL true

// Hard anti-chatter: minimum time we keep a state before allowing change
const unsigned long IOT_MIN_HOLD_MS = 30000;

/* ================= I2C RELAY (PCF8574) mapping ================= */
const uint8_t RELAYNUM_PUMP   = 2;
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
const float TEMP_HYS = 0.5;
const float HUM_HYS  = 3.5;

/* ================= TIMING ================= */
const unsigned long BTN_DEBOUNCE_MS     = 450;
const unsigned long MQTT_RECONNECT_MS   = 3000;
const unsigned long PUBLISH_SENSORS_MS  = 1000;
const unsigned long PUBLISH_STATUS_MS   = 1200;
const unsigned long PUBLISH_HEALTH_MS   = 5000;
const unsigned long SCREEN_REFRESH_MS   = 500;

/* ================= BUTTONS ================= */
bool lastTouch[4] = {false, false, false, false};
unsigned long lastToggleMs[4] = {0, 0, 0, 0};

/* ================= TIMERS ================= */
unsigned long lastMqttAttemptMs = 0;
unsigned long lastPubSensorsMs  = 0;
unsigned long lastPubStatusMs   = 0;
unsigned long lastPubHealthMs   = 0;
unsigned long lastScreenMs      = 0;

// IoT relay state-hold + output latch
unsigned long lastIotChangeMs = 0;
bool lastIotPinState = false;   // last state written to PIN (logical state: ON/OFF)

/* ================= MQTT COMMAND FLAGS ================= */
bool pendingPhase = false;
int  pendingPhaseValue = 1;

/* ================= SOIL SENSOR (I2C) ================= */
Adafruit_seesaw ss;
bool soilPresent = false;
uint16_t lastSoilRaw = 0;

// Your calibration:
const uint16_t SOIL_DRY_RAW = 300; // 0%
const uint16_t SOIL_WET_RAW = 650; // 100%

// Your pump threshold (raw):
const uint16_t PUMP_ON_BELOW_RAW = 450;

/* ================= Last valid Env readings ================= */
float lastTemp = 0.0f;
float lastHum  = 0.0f;
bool  envOk    = false;

/* ================= PROTOTYPES ================= */
void onMqttMessage(int size);

void publishPhase();
void publishActuators();
void publishSensors(float temp, float hum, uint16_t soilRaw, uint8_t soilPct);
void publishHealth();

void applyAutomation(float t, float h, uint16_t soilRaw);
void applyOutputs();

void drawStaticUI();
void updateScreen(float t, float h, uint16_t soilRaw, uint8_t soilPct);

uint16_t readSoilRaw();
uint8_t  soilPercent(uint16_t raw);

bool i2cPing(uint8_t addr);
void pahubSelect(uint8_t ch);
void useCarrierBus();
void usePcfBus();
void useSoilBus();

bool initCarrierOnCurrentBusAndValidate();
void detectCarrierBusRobust();

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

void useCarrierBus() {
  if (!pahubPresent) return;
  if (carrierChannel == 255) return;
  pahubSelect(carrierChannel);
}

void usePcfBus() {
  if (!pahubPresent) return;
  pahubSelect(CH_PCF);
}

void useSoilBus() {
  if (!pahubPresent) return;
  pahubSelect(CH_SOIL);
}

/* ================= Robust carrier init + validate ================= */
bool initCarrierOnCurrentBusAndValidate() {
  carrier.begin();

  float t1 = carrier.Env.readTemperature();
  float h1 = carrier.Env.readHumidity();
  delay(20);
  float t2 = carrier.Env.readTemperature();
  float h2 = carrier.Env.readHumidity();

  auto valid = [](float t, float h) -> bool {
    if (isnan(t) || isnan(h)) return false;
    if (t < -5.0 || t > 60.0) return false;
    if (h < 0.0  || h > 100.0) return false;
    if (t == 0.0f && h == 0.0f) return false;
    return true;
  };

  if (valid(t1, h1) || valid(t2, h2)) {
    lastTemp = valid(t2, h2) ? t2 : t1;
    lastHum  = valid(t2, h2) ? h2 : h1;
    envOk = true;
    return true;
  }

  envOk = false;
  return false;
}

void detectCarrierBusRobust() {
  pahubPresent = i2cPing(PAHUB_ADDR);

#if DEBUG_SERIAL
  Serial.print("[I2C] PaHub present: ");
  Serial.println(pahubPresent ? "YES" : "NO");
#endif

  if (!pahubPresent) {
    carrierChannel = 255;
    carrierOk = initCarrierOnCurrentBusAndValidate();
#if DEBUG_SERIAL
    Serial.print("[I2C] Carrier direct bus ok: ");
    Serial.println(carrierOk ? "YES" : "NO");
#endif
    return;
  }

  // 1) Try UPSTREAM first
  carrierChannel = 255;
  carrierOk = initCarrierOnCurrentBusAndValidate();
#if DEBUG_SERIAL
  Serial.print("[I2C] Try carrier UPSTREAM -> ");
  Serial.println(carrierOk ? "OK" : "FAIL");
#endif
  if (carrierOk) return;

  // 2) Try each channel
  for (uint8_t ch = 0; ch < 8; ch++) {
    carrierChannel = ch;
    pahubSelect(ch);

    bool ok = initCarrierOnCurrentBusAndValidate();
#if DEBUG_SERIAL
    Serial.print("[I2C] Try carrier ch"); Serial.print(ch);
    Serial.print(" -> "); Serial.println(ok ? "OK" : "FAIL");
#endif
    if (ok) {
      carrierOk = true;
      return;
    }
  }

  carrierOk = false;
  carrierChannel = 255;
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

/* ================= IOT RELAY (NO FLICKER) ================= */
void writeIoTPinIfChanged(bool logicalOn) {
  // This is the key: DO NOT write every loop. Only if state changes.
  if (logicalOn == lastIotPinState) return;

#if IOT_ACTIVE_LOW
  digitalWrite(PIN_IOT_RELAY, logicalOn ? LOW : HIGH);
#else
  digitalWrite(PIN_IOT_RELAY, logicalOn ? HIGH : LOW);
#endif

  lastIotPinState = logicalOn;
}

bool applyHoldTime(bool current, bool desired) {
  if (current == desired) return current;
  unsigned long now = millis();
  if (now - lastIotChangeMs < IOT_MIN_HOLD_MS) {
    // Still within hold time: keep current state
    return current;
  }
  lastIotChangeMs = now;
  return desired;
}

/* ================= OUTPUTS ================= */
void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);

  // IoT relay: write only when changed
  writeIoTPinIfChanged(iot.state);
}

/* ================= MANUAL TOGGLE ================= */
void toggleManual(Actuator &a) {
  if (!a.manual) { a.manual = true;  a.state = true;  }
  else           { a.manual = false; a.state = false; }
}

/* ================= BUTTONS ================= */
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

/* ================= HYSTERESIS ================= */
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

/* ================= AUTOMATION ================= */
void applyAutomation(float t, float h, uint16_t soilRaw) {
  // Pump (same for all phases, RAW threshold)
  if (!pump.manual) {
    pump.state = (soilRaw < PUMP_ON_BELOW_RAW);
  }

  // Heater
  if (!heater.manual) {
    float lim = (phase <= 2) ? 20.0 : 18.0;
    heater.state = belowHys(heater.state, t, lim, TEMP_HYS);
  }

  // Humidifier
  if (!humid.manual) {
    float lim = (phase == 1) ? 65.0 :
                (phase == 2) ? 55.0 :
                (phase == 3) ? 50.0 : 40.0;
    humid.state = belowHys(humid.state, h, lim, HUM_HYS);
  }

  // IoT relay
  if (!iot.manual) {
    bool desired = false;

    if (phase == 1) {
      desired = aboveHys(iot.state, h, 75.0, HUM_HYS);
    } else if (phase == 2) {
      desired = aboveHys(iot.state, h, 70.0, HUM_HYS);
    } else {
#if IOT_SHARED_DEHUM_AND_COOL
      desired = true;
#else
      desired = aboveHys(iot.state, t, 22.0, TEMP_HYS);
#endif
    }

    // IMPORTANT: hold time prevents chattering near threshold
    iot.state = applyHoldTime(iot.state, desired);
  }
}

/* ================= MQTT ================= */
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

    // Only phase control
    mqtt.subscribe(TOPIC_CMD_PHASE);

#if DEBUG_SERIAL
    Serial.println("[MQTT] Connected + subscribed (PHASE ONLY).");
#endif

    publishPhase();
    publishActuators();
    publishHealth();
  }
}

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
  }
}

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

void publishSensors(float temp, float hum, uint16_t soilRaw, uint8_t soilPct) {
  if (!mqttReady()) return;
  mqtt.beginMessage(TOPIC_STATE_SENSORS);
  mqtt.print("{");
  mqtt.print("\"phase\":"); mqtt.print(phase); mqtt.print(",");
  mqtt.print("\"temp\":");  mqtt.print(temp, 1); mqtt.print(",");
  mqtt.print("\"hum\":");   mqtt.print(hum, 1); mqtt.print(",");
  mqtt.print("\"soil_raw\":"); mqtt.print(soilRaw); mqtt.print(",");
  mqtt.print("\"soil_pct\":"); mqtt.print(soilPct);
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
  mqtt.print("\"carrier_ok\":"); mqtt.print(carrierOk ? "true":"false"); mqtt.print(",");
  mqtt.print("\"env_ok\":"); mqtt.print(envOk ? "true":"false"); mqtt.print(",");
  mqtt.print("\"soil_i2c\":"); mqtt.print(soilPresent ? "true":"false");
  mqtt.print("}");
  mqtt.endMessage();
}

/* ================= SOIL ================= */
uint16_t readSoilRaw() {
  if (!soilPresent) return lastSoilRaw;
  if (pahubPresent) useSoilBus();
  uint16_t v = ss.touchRead(0);
  useCarrierBus();
  lastSoilRaw = v;
  return v;
}

uint8_t soilPercent(uint16_t raw) {
  // RAW 300 -> 0%, RAW 650 -> 100%
  if (SOIL_WET_RAW == SOIL_DRY_RAW) return 0;

  if (raw <= SOIL_DRY_RAW) return 0;
  if (raw >= SOIL_WET_RAW) return 100;

  long pct = (long)(raw - SOIL_DRY_RAW) * 100L / (long)(SOIL_WET_RAW - SOIL_DRY_RAW);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

/* ================= SCREEN (no flicker) ================= */
void drawStaticUI() {
  carrier.display.fillScreen(ST77XX_BLACK);
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);

  carrier.display.setCursor(0, 0);
  carrier.display.println("LEAF LAB");

  carrier.display.setCursor(0, 30);  carrier.display.print("FASE:");
  carrier.display.setCursor(0, 55);  carrier.display.print("T:");
  carrier.display.setCursor(0, 80);  carrier.display.print("H:");
  carrier.display.setCursor(0, 105); carrier.display.print("SOIL:");

  // Swap display order: HUMI then PUMP
  carrier.display.setCursor(0, 135); carrier.display.print("HUMI:");
  carrier.display.setCursor(0, 155); carrier.display.print("PUMP:");
  carrier.display.setCursor(0, 175); carrier.display.print("HEAT:");
  carrier.display.setCursor(0, 195); carrier.display.print("IOT:");
}

void printValueAt(int x, int y, const String &val, int w = 150, int h = 18) {
  carrier.display.fillRect(x, y, w, h, ST77XX_BLACK);
  carrier.display.setCursor(x, y);
  carrier.display.print(val);
}

void updateScreen(float t, float h, uint16_t soilRaw, uint8_t soilPct) {
  carrier.display.setTextSize(2);
  carrier.display.setTextColor(ST77XX_WHITE);

  printValueAt(70, 30,  String(phase));
  printValueAt(40, 55,  String(t, 1) + "C");
  printValueAt(40, 80,  String(h, 1) + "%");
  printValueAt(80, 105, String(soilPct) + "% (" + String(soilRaw) + ")");

  auto modeTxt = [](const Actuator &a) -> String {
    return String(a.manual ? "MAN" : "AUT") + " " + String(a.state ? "ON" : "OFF");
  };

  printValueAt(70, 135, modeTxt(humid));
  printValueAt(70, 155, modeTxt(pump));
  printValueAt(70, 175, modeTxt(heater));
  printValueAt(70, 195, modeTxt(iot));
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  requestNetworkConfigFromSerial();

  Wire.begin();

  // Detect and init carrier
  detectCarrierBusRobust();

  pinMode(PIN_IOT_RELAY, OUTPUT);

  // Force a clean OFF and store the latch state (prevents "first loop flicker")
  bool offLogical = false;
#if IOT_ACTIVE_LOW
  digitalWrite(PIN_IOT_RELAY, HIGH); // OFF
#else
  digitalWrite(PIN_IOT_RELAY, LOW);  // OFF
#endif
  lastIotPinState = offLogical;
  lastIotChangeMs = millis(); // start hold timer from boot

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
  if (i2cPing(SOIL_ADDR) && ss.begin(SOIL_ADDR)) {
    soilPresent = true;
#if DEBUG_SERIAL
    Serial.println("[I2C] Soil sensor detected + initialized (CH3, 0x36).");
#endif
  } else {
#if DEBUG_SERIAL
    Serial.println("[I2C] Soil sensor NOT detected on CH3 (0x36).");
#endif
  }
  useCarrierBus();

  drawStaticUI();
  lastScreenMs = 0;

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

  // Apply pending phase
  bool changed = false;
  if (pendingPhase) { pendingPhase = false; phase = pendingPhaseValue; changed = true; }

  // Env readings (guard)
  useCarrierBus();
  float t = carrier.Env.readTemperature();
  float h = carrier.Env.readHumidity();

  if (!isnan(t) && !isnan(h) && !(t == 0.0f && h == 0.0f)) {
    lastTemp = t;
    lastHum  = h;
    envOk = true;
  } else {
    envOk = false;
  }

  // Soil
  uint16_t soilRaw = readSoilRaw();
  uint8_t  soilPct = soilPercent(soilRaw);

  // Automation
  applyAutomation(lastTemp, lastHum, soilRaw);

  // Outputs (IoT pin is latched here)
  applyOutputs();

  // Publish + screen
  unsigned long now = millis();

  if (changed) {
    publishPhase();
    publishActuators();
    lastScreenMs = 0;
  }

  if (now - lastPubSensorsMs > PUBLISH_SENSORS_MS) {
    lastPubSensorsMs = now;
    publishSensors(lastTemp, lastHum, soilRaw, soilPct);
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
    updateScreen(lastTemp, lastHum, soilRaw, soilPct);
  }

  delay(20);
}
