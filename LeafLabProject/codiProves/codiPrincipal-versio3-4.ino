/***********************************************************************
  LEAF LAB – v5.5 FINAL (PRODUCCIÓN)
  - Umbral Suelo: < 450 (Bomba ON)
  - Umbral F3/F4: > 26°C (IoT ON)
  - Tiempos: Lectura 1s | Publicación 30s
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_seesaw.h>

MKRIoTCarrier carrier;
WiFiClient wifi;
MqttClient mqtt(wifi);

// Credenciales (Se piden por Serial en Setup)
char ssid[32], pass[32], broker[32];
const int  mqttPort = 1883;

// Configuración Hardware
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define PIN_IOT    2
const uint8_t CH_PCF  = 5;
const uint8_t CH_SOIL = 3;

// Estados
int phase = 1;
struct Actuator { bool manual; bool state; };
Actuator pump = {false, false}, humid = {false, false}, heater = {false, false}, iot = {false, false};

// Temporizadores
unsigned long lastRead = 0, lastPub = 0;
uint16_t currentSoil = 0;
Adafruit_seesaw ss;
bool soilReady = false;

/* --- SOPORTE BUS I2C --- */
void select(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/* ---------- SERIAL INPUT ROBUSTO ---------- */
String readLineBlocking() {
  // Lee hasta '\n' (o '\r\n'), con timeout suave por si el monitor serie no manda nada aún
  String s = "";
  unsigned long start = millis();
  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        s.trim();
        return s;
      }
      if (c != '\r') s += c;
    }
    // evita bloquear para siempre si la consola no está lista todavía
    if (millis() - start > 300000UL) { // 5 min (por seguridad)
      s.trim();
      return s;
    }
    delay(10);
  }
}

void promptToChar(const __FlashStringHelper* label, char* dst, size_t dstSize, bool allowEmpty) {
  while (true) {
    Serial.println(label);
    String s = readLineBlocking();
    s.trim();

    if (!allowEmpty && s.length() == 0) {
      Serial.println(F("[ERR] No puede estar vacío. Intenta de nuevo."));
      continue;
    }

    // Copia segura a char[]
    s.toCharArray(dst, dstSize);
    // Asegura terminación nula por si acaso
    dst[dstSize - 1] = '\0';
    break;
  }
}

void askUserConfig() {
  Serial.println(F("=== LEAF LAB SETUP ==="));
  Serial.println(F("Escribe cada valor y pulsa ENTER (Monitor Serie: 'Newline')."));

  promptToChar(F("Enter WIFI SSID:"), ssid, sizeof(ssid), false);
  promptToChar(F("Enter WIFI PASSWORD (vacío si no tiene):"), pass, sizeof(pass), true);
  promptToChar(F("Enter MQTT HOST (IP o hostname):"), broker, sizeof(broker), false);

  Serial.println(F("Configuration received:"));
  Serial.print(F("SSID: "));   Serial.println(ssid);
  Serial.print(F("BROKER: ")); Serial.println(broker);
}

/* ---------- WIFI + MQTT ---------- */
void connectWiFi() {
  // Espera a que el módulo WiFi responda
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("[WIFI] No se detecta módulo WiFiNINA."));
    // aquí podrías parar, pero seguimos para que lo veas en serie
  }

  Serial.print(F("[WIFI] Connecting to "));
  Serial.println(ssid);

  // Intentos hasta conectar
  while (WiFi.status() != WL_CONNECTED) {
    int status = WiFi.begin(ssid, pass); // pass puede ser ""
    (void)status;
    unsigned long t0 = millis();
    while (millis() - t0 < 12000UL) { // espera 12s por intento
      if (WiFi.status() == WL_CONNECTED) break;
      delay(250);
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[WIFI] Retry..."));
    }
  }

  Serial.print(F("[WIFI] Connected. IP: "));
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  mqtt.setId("leaflab-mkr");          // opcional
  mqtt.setKeepAliveInterval(30);      // opcional
  mqtt.setConnectionTimeout(5);       // opcional
  mqtt.setCleanSession(true);         // opcional

  Serial.print(F("[MQTT] Connecting to "));
  Serial.print(broker);
  Serial.print(F(":"));
  Serial.println(mqttPort);

  while (!mqtt.connect(broker, mqttPort)) {
    Serial.print(F("[MQTT] Failed, rc="));
    Serial.println(mqtt.connectError());
    delay(2000);
  }

  Serial.println(F("[MQTT] Connected + subscribed."));

  // Ejemplo de subs:
  mqtt.subscribe("leaflab/phase");
  mqtt.subscribe("leaflab/cmd/#");
}

/* --- LÓGICA DE CONTROL (TU CÓDIGO) --- */
void applyAutomation(float t, float h, uint16_t s) {
  if (!pump.manual) {
    if (s < 450) pump.state = true;
    else if (s > 480) pump.state = false;
  }

  if (!heater.manual) {
    float lim = (phase <= 2) ? 20.0 : 18.0;
    if (t < lim) heater.state = true;
    else if (t > lim + 0.5) heater.state = false;
  }

  if (!humid.manual) {
    float h_lim = (phase == 1 ? 65.0 : (phase == 2 ? 55.0 : (phase == 3 ? 50.0 : 40.0)));
    if (h < h_lim) humid.state = true;
    else if (h > h_lim + 2.0) humid.state = false;
  }

  if (!iot.manual) {
    if (phase == 1) {
      if (h > 75.0) iot.state = true; else if (h < 72.0) iot.state = false;
    } else if (phase == 2) {
      if (h > 70.0) iot.state = true; else if (h < 67.0) iot.state = false;
    } else {
      if (t > 26.0) iot.state = true;
      else if (t < 25.5) iot.state = false;
    }
  }
}

void applyOutputs() {
  uint8_t val = 0xFF;
  if (pump.state)   val &= ~(1 << 1); // Relay 2
  if (heater.state) val &= ~(1 << 6); // Relay 7
  if (humid.state)  val &= ~(1 << 7); // Relay 8

  select(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();

  digitalWrite(PIN_IOT, iot.state ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // importante en MKR para que el monitor serie esté listo

  Wire.begin();
  pinMode(PIN_IOT, OUTPUT);

  // Pide config primero
  askUserConfig();

  // Conecta WiFi/MQTT
  connectWiFi();
  connectMQTT();

  // Carrier
  if (!carrier.begin()) {
    Serial.println(F("[CARRIER] No responde carrier.begin()"));
  }

  // Sensor de suelo
  select(CH_SOIL);
  if (ss.begin(0x36)) {
    soilReady = true;
  } else {
    Serial.println(F("[SOIL] Seesaw no detectado en canal CH_SOIL."));
  }

  Serial.println(F("[BOOT] Setup done."));
}

void loop() {
  mqtt.poll();
  unsigned long now = millis();

  // reconexión simple
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[WIFI] Reconnecting..."));
    connectWiFi();
  }
  if (!mqtt.connected()) {
    Serial.println(F("[MQTT] Reconnecting..."));
    connectMQTT();
  }

  if (now - lastRead >= 1000) {
    lastRead = now;

    float t = carrier.Env.readTemperature();
    float h = carrier.Env.readHumidity();

    select(CH_SOIL);
    if (soilReady) currentSoil = ss.touchRead(0);

    applyAutomation(t, h, currentSoil);
    applyOutputs();

    Serial.print(F("[SENS] T=")); Serial.print(t, 2);
    Serial.print(F(" H="));       Serial.print(h, 2);
    Serial.print(F(" SOIL="));    Serial.println(currentSoil);
  }

  if (now - lastPub >= 30000) {
    lastPub = now;
    // aquí publicarías tu JSON
  }
}
