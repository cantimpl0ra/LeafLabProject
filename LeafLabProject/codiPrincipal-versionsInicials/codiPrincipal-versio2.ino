/***********************************************************************
  LEAF LAB – Automation v3.0 STABLE (Serial Network Config)
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

MKRIoTCarrier carrier;

/* ================= WIFI + MQTT (CONFIG VIA SERIAL) ================= */
char WIFI_SSID[32];
char WIFI_PASS[32];
char MQTT_HOST[32];
const int MQTT_PORT = 1883;

WiFiClient wifi;
MqttClient mqtt(wifi);

/* ================= MQTT TOPICS ================= */
const char* TOPIC_PHASE = "leaflab/phase";
const char* TOPIC_MANUAL_PUMP   = "leaflab/manual/pump";
const char* TOPIC_MANUAL_HUMID  = "leaflab/manual/humid";
const char* TOPIC_MANUAL_HEATER = "leaflab/manual/heater";
const char* TOPIC_MANUAL_IOT    = "leaflab/manual/iot";

/* ================= CONSTANTES ================= */
const float TEMP_HYS = 0.5;
const float HUM_HYS  = 3.0;
const int   SOIL_HYS = 30;

const unsigned long BTN_DEBOUNCE_MS = 500;
const unsigned long MQTT_RECONNECT_MS = 3000;
const unsigned long SCREEN_REFRESH_MS = 250;

/* ================= IOT RELAY ================= */
const int PIN_IOT_RELAY = 2;

/* ================= I2C RELAY ================= */
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define CH_PCF     5

const uint8_t RELAYNUM_PUMP   = 2;
const uint8_t RELAYNUM_HUMID  = 8;
const uint8_t RELAYNUM_HEATER = 7;

uint8_t pcfState = 0x00;

/* ================= FASE ================= */
int phase = 1;

/* ================= ACTUATOR ================= */
struct Actuator {
  bool manual;
  bool state;
};

Actuator pump   = {false, false};
Actuator humid  = {false, false};
Actuator heater = {false, false};
Actuator iot    = {false, false};

/* ================= BOTONES ================= */
bool lastTouch[4] = {false, false, false, false};
unsigned long lastToggleMs[4] = {0, 0, 0, 0};

/* ================= TIMERS ================= */
unsigned long lastMqttAttemptMs = 0;
unsigned long lastScreenMs = 0;

/* ================= SERIAL CONFIG ================= */
void readLine(char* buffer, size_t maxLen) {
  size_t idx = 0;
  while (idx < maxLen - 1) {
    while (!Serial.available());
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
    buffer[idx++] = c;
  }
  buffer[idx] = '\0';
}

void requestNetworkConfigFromSerial() {
  Serial.println("=== LEAF LAB SETUP ===");

  Serial.print("Enter WIFI SSID: ");
  readLine(WIFI_SSID, sizeof(WIFI_SSID));

  Serial.print("Enter WIFI PASSWORD: ");
  readLine(WIFI_PASS, sizeof(WIFI_PASS));

  Serial.print("Enter MQTT HOST (IP or hostname): ");
  readLine(MQTT_HOST, sizeof(MQTT_HOST));

  Serial.println("Configuration received.");
}

/* ================= PAHUB / PCF ================= */
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void writePCF() {
  selectChannel(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();
}

void setI2CRelay(uint8_t relayNum, bool on) {
  uint8_t bit = relayNum - 1;
  if (on)  pcfState |=  (1 << bit);
  else     pcfState &= ~(1 << bit);
  writePCF();
}

void setIoTRelay(bool on) {
  digitalWrite(PIN_IOT_RELAY, on ? HIGH : LOW);
}

void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);
  setIoTRelay(iot.state);
}

/* ================= MANUAL TOGGLE ================= */
void toggleManual(Actuator &a) {
  if (!a.manual) {
    a.manual = true;
    a.state  = true;
  } else {
    a.manual = false;
    a.state  = false;
  }
}

/* ================= BOTONES ================= */
void handleButtons() {
  bool t[4];
  t[0] = carrier.Buttons.getTouch(TOUCH0);
  t[1] = carrier.Buttons.getTouch(TOUCH1);
  t[2] = carrier.Buttons.getTouch(TOUCH2);
  t[3] = carrier.Buttons.getTouch(TOUCH3);

  unsigned long now = millis();

  if (t[0] && !lastTouch[0] && now - lastToggleMs[0] > BTN_DEBOUNCE_MS) { toggleManual(pump);   lastToggleMs[0] = now; applyOutputs(); }
  if (t[1] && !lastTouch[1] && now - lastToggleMs[1] > BTN_DEBOUNCE_MS) { toggleManual(humid);  lastToggleMs[1] = now; applyOutputs(); }
  if (t[2] && !lastTouch[2] && now - lastToggleMs[2] > BTN_DEBOUNCE_MS) { toggleManual(heater); lastToggleMs[2] = now; applyOutputs(); }
  if (t[3] && !lastTouch[3] && now - lastToggleMs[3] > BTN_DEBOUNCE_MS) { toggleManual(iot);    lastToggleMs[3] = now; applyOutputs(); }

  for (int i = 0; i < 4; i++) lastTouch[i] = t[i];
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
    mqtt.subscribe(TOPIC_PHASE);
    mqtt.subscribe(TOPIC_MANUAL_PUMP);
    mqtt.subscribe(TOPIC_MANUAL_HUMID);
    mqtt.subscribe(TOPIC_MANUAL_HEATER);
    mqtt.subscribe(TOPIC_MANUAL_IOT);
  }
}

/* ================= MQTT HANDLER ================= */
void onMqttMessage(int) {
  String payload;
  while (mqtt.available()) payload += (char)mqtt.read();
  payload.trim();

  String topic = mqtt.messageTopic();
  if (topic == TOPIC_PHASE) {
    int p = payload.toInt();
    if (p >= 1 && p <= 4) phase = p;
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  while (!Serial);

  requestNetworkConfigFromSerial();

  pinMode(PIN_IOT_RELAY, OUTPUT);
  digitalWrite(PIN_IOT_RELAY, LOW);

  Wire.begin();
  selectChannel(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  carrier.begin();
}

/* ================= LOOP ================= */
void loop() {
  carrier.Buttons.update();
  handleButtons();

  ensureWiFi();
  ensureMQTT();
  mqtt.poll();

  delay(20);
}
