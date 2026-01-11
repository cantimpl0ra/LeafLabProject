/***********************************************************************
  LEAF LAB – Automation v2.4 FINAL (TOUCH-compatible)

  ✔ PCF8574 direct logic (1 = ON, 0 = OFF)
  ✔ All I2C relays OFF at boot
  ✔ Buttons working (carrier.Buttons.update + TOUCH0..3)
  ✔ Manual / AUTO per actuator (buttons + MQTT toggle)
  ✔ Hysteresis (temp / hum / soil)
  ✔ Phase control from Node-RED
  ✔ SD logging every 10 minutes
  ✔ Real-time MQTT telemetry
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <SD.h>

MKRIoTCarrier carrier;

/* ================= WIFI + MQTT ================= */
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASS";

const char* MQTT_HOST = "192.168.1.50";
const int   MQTT_PORT = 1883;

WiFiClient wifi;
MqttClient mqtt(wifi);

const char* TOPIC_PHASE = "leaflab/phase";
const char* TOPIC_MANUAL_PUMP   = "leaflab/manual/pump";
const char* TOPIC_MANUAL_HEATER = "leaflab/manual/heater";
const char* TOPIC_MANUAL_HUMID  = "leaflab/manual/humid";
const char* TOPIC_MANUAL_IOT    = "leaflab/manual/iot";
const char* TOPIC_SENSORS       = "leaflab/sensors";

/* ================= HYSTERESIS ================= */
const float TEMP_HYS = 0.5;
const float HUM_HYS  = 3.0;
const int   SOIL_HYS = 30;

/* ================= IOT RELAY ================= */
const int PIN_IOT_RELAY = 2;

/* ================= I2C RELAY (PCF8574) ================= */
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define CH_PCF     5

const uint8_t RELAYNUM_PUMP   = 2;
const uint8_t RELAYNUM_HEATER = 7;
const uint8_t RELAYNUM_HUMID  = 8;

// Direct logic: 1 = ON, 0 = OFF
uint8_t pcfState = 0x00;

/* ================= PHASE ================= */
int phase = 1;

/* ================= ACTUATOR STRUCT ================= */
struct Actuator {
  bool manual;
  bool state;
};

Actuator pump   = {false, false};
Actuator heater = {false, false};
Actuator humid  = {false, false};
Actuator iot    = {false, false};

/* ================= SD LOG ================= */
unsigned long lastLogMs = 0;
const unsigned long LOG_INTERVAL_MS = 600000;

/* ================= PAHUB + PCF ================= */
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
  if (relayNum < 1 || relayNum > 8) return;
  uint8_t bitIndex = relayNum - 1;

  if (on)  pcfState |=  (1 << bitIndex);
  else     pcfState &= ~(1 << bitIndex);

  writePCF();
}

/* ================= IOT RELAY ================= */
void setIoTRelay(bool on) {
  digitalWrite(PIN_IOT_RELAY, on ? HIGH : LOW);
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

/* ================= APPLY OUTPUTS ================= */
void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setIoTRelay(iot.state);
}

/* ================= BUTTONS ================= */
void handleButtons() {

  if (carrier.Buttons.getTouch(TOUCH0)) {
    toggleManual(pump);
    applyOutputs();
    delay(250);
  }

  if (carrier.Buttons.getTouch(TOUCH1)) {
    toggleManual(heater);
    applyOutputs();
    delay(250);
  }

  if (carrier.Buttons.getTouch(TOUCH2)) {
    toggleManual(humid);
    applyOutputs();
    delay(250);
  }

  if (carrier.Buttons.getTouch(TOUCH3)) {
    toggleManual(iot);
    applyOutputs();
    delay(250);
  }
}

/* ================= MQTT ================= */
void onMqttMessage(int size) {
  String payload;
  while (mqtt.available()) payload += (char)mqtt.read();
  payload.trim();

  String topic = mqtt.messageTopic();

  if (topic == TOPIC_PHASE) {
    int p = payload.toInt();
    if (p >= 1 && p <= 4) phase = p;
    return;
  }

  if (payload == "toggle") {
    if (topic == TOPIC_MANUAL_PUMP)   toggleManual(pump);
    if (topic == TOPIC_MANUAL_HEATER) toggleManual(heater);
    if (topic == TOPIC_MANUAL_HUMID)  toggleManual(humid);
    if (topic == TOPIC_MANUAL_IOT)    toggleManual(iot);
    applyOutputs();
  }
}

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(250);
  }

  if (!mqtt.connected()) {
    mqtt.setId("leaflab-mkr");
    mqtt.connect(MQTT_HOST, MQTT_PORT);
    mqtt.onMessage(onMqttMessage);

    mqtt.subscribe(TOPIC_PHASE);
    mqtt.subscribe(TOPIC_MANUAL_PUMP);
    mqtt.subscribe(TOPIC_MANUAL_HEATER);
    mqtt.subscribe(TOPIC_MANUAL_HUMID);
    mqtt.subscribe(TOPIC_MANUAL_IOT);
  }
}

/* ================= HYSTERESIS ================= */
bool belowHys(bool state, float val, float limit, float hys) {
  if (!state && val < limit) return true;
  if ( state && val > limit + hys) return false;
  return state;
}

bool aboveHys(bool state, float val, float limit, float hys) {
  if (!state && val > limit) return true;
  if ( state && val < limit - hys) return false;
  return state;
}

bool soilHys(bool state, int soil, int threshold) {
  if (!state && soil > threshold) return true;
  if ( state && soil < threshold - SOIL_HYS) return false;
  return state;
}

/* ================= AUTOMATION ================= */
void applyAutomation(float t, float h, int soil) {

  if (!pump.manual) {
    int th = (phase == 1) ? 700 :
             (phase == 2) ? 650 :
             (phase == 3) ? 680 : 720;
    pump.state = soilHys(pump.state, soil, th);
  }

  if (!heater.manual) {
    float lim = (phase <= 2) ? 20.0 : 18.0;
    heater.state = belowHys(heater.state, t, lim, TEMP_HYS);
  }

  if (!humid.manual) {
    float lim = (phase == 1) ? 65 :
                (phase == 2) ? 55 :
                (phase == 3) ? 50 : 40;
    humid.state = belowHys(humid.state, h, lim, HUM_HYS);
  }

  if (!iot.manual) {
    if (phase <= 2) {
      float lim = (phase == 1) ? 75 : 70;
      iot.state = aboveHys(iot.state, h, lim, HUM_HYS);
    } else {
      iot.state = aboveHys(iot.state, t, 20.0, TEMP_HYS);
    }
  }
}

/* ================= TELEMETRY ================= */
void publishSensors(float t, float h, int soil, int tds, float ph, int air) {
  mqtt.beginMessage(TOPIC_SENSORS);
  mqtt.print("{");
  mqtt.print("\"phase\":"); mqtt.print(phase); mqtt.print(",");
  mqtt.print("\"temp\":");  mqtt.print(t,1); mqtt.print(",");
  mqtt.print("\"hum\":");   mqtt.print(h,1); mqtt.print(",");
  mqtt.print("\"soil\":");  mqtt.print(soil); mqtt.print(",");
  mqtt.print("\"tds\":");   mqtt.print(tds);  mqtt.print(",");
  mqtt.print("\"ph\":");    mqtt.print(ph,1); mqtt.print(",");
  mqtt.print("\"air\":");   mqtt.print(air);
  mqtt.print("}");
  mqtt.endMessage();
}

/* ================= SETUP ================= */
void setup() {
  pinMode(PIN_IOT_RELAY, OUTPUT);
  digitalWrite(PIN_IOT_RELAY, LOW);

  Wire.begin();

  // Force ALL I2C relays OFF at boot
  selectChannel(CH_PCF);
  pcfState = 0x00;
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();

  carrier.begin();

  pump.manual = heater.manual = humid.manual = iot.manual = false;
  pump.state  = heater.state  = humid.state  = iot.state  = false;
  applyOutputs();

  SD.begin(4);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

/* ================= LOOP ================= */
void loop() {
  ensureConnections();
  mqtt.poll();

  carrier.Buttons.update();   // 🔴 CRITICAL
  handleButtons();

  float temp = carrier.Env.readTemperature();
  float hum  = carrier.Env.readHumidity();

  int soil = analogRead(A0);
  int tds  = analogRead(A1);
  float ph = analogRead(A2);
  int air  = carrier.AirQuality.readVOC();

  applyAutomation(temp, hum, soil);
  applyOutputs();

  publishSensors(temp, hum, soil, tds, ph, air);

  if (millis() - lastLogMs > LOG_INTERVAL_MS) {
    lastLogMs = millis();
    File f = SD.open("data.csv", FILE_WRITE);
    if (f) {
      f.print(millis()); f.print(",");
      f.print(phase);    f.print(",");
      f.print(soil);     f.print(",");
      f.print(hum);      f.print(",");
      f.print(temp);     f.print(",");
      f.print(tds);      f.print(",");
      f.print(ph);       f.print(",");
      f.println(air);
      f.close();
    }
  }

  delay(1000);
}
