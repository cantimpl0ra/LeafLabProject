/***********************************************************************
  LEAF LAB – v5.4 (ESTABILIDAD TOTAL)
  - Lectura sensores: cada 1 segundo
  - Envío MQTT: cada 30 segundos
  - Umbral Suelo: < 450 (Bomba ON)
  - IoT Relay (F3/F4): > 26.0°C (ON) | < 25.5°C (OFF)
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_seesaw.h>

MKRIoTCarrier carrier;
char WIFI_SSID[32], WIFI_PASS[32], MQTT_HOST[32];
WiFiClient wifi;
MqttClient mqtt(wifi);

// Tópicos
const char* TOPIC_STATE_ACTUATORS = "leaflab/state/actuators";
const char* TOPIC_STATE_SENSORS   = "leaflab/state/sensors";

// Hardware y Estados
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define PIN_IOT_RELAY 2
bool pahubPresent = false;
uint8_t pcfState = 0x00;
int phase = 1;

struct Actuator { bool manual; bool state; };
Actuator pump = {false, false}, humid = {false, false}, heater = {false, false}, iot = {false, false};

// Tiempos
unsigned long lastReadMs = 0;
unsigned long lastPubMs = 0;
uint16_t lastSoil = 0;
Adafruit_seesaw ss;

void setup() {
  Serial.begin(115200);
  // Aquí iría tu función de lectura de red por Serial
  Wire.begin();
  pinMode(PIN_IOT_RELAY, OUTPUT);
  carrier.begin();
  // Inicialización de PaHub y Soil (abreviado)
  pahubPresent = (Wire.status() == 0); 
}

/* --- LOGICA DE AUTOMATIZACION SIN CONFLICTOS --- */
void applyAutomation(float t, float h, uint16_t soil) {
  
  // 1. Bomba de Agua (Histeresis para evitar rebotes)
  if (!pump.manual) {
    if (soil < 450) pump.state = true;
    else if (soil > 480) pump.state = false;
  }

  // 2. Calentador
  if (!heater.manual) {
    float t_limit = (phase <= 2) ? 20.0 : 18.0;
    if (t < t_limit) heater.state = true;
    else if (t > t_limit + 0.5) heater.state = false;
  }

  // 3. Humidificador
  if (!humid.manual) {
    float h_limit = (phase == 1 ? 65.0 : (phase == 2 ? 55.0 : (phase == 3 ? 50.0 : 40.0)));
    if (h < h_limit) humid.state = true;
    else if (h > h_limit + 2.0) humid.state = false;
  }

  // 4. IoT RELAY - SOLUCIÓN AL PARPADEO
  // Eliminamos cualquier orden de "Siempre ON" que causaba ruido a 22°C
  if (!iot.manual) {
    if (phase == 1) {
      if (h > 75.0) iot.state = true;
      else if (h < 72.0) iot.state = false;
    } 
    else if (phase == 2) {
      if (h > 70.0) iot.state = true;
      else if (h < 67.0) iot.state = false;
    } 
    else { 
      // Fases 3 y 4: SOLO responde a T > 26.0
      // Si la temperatura es 22, esta lógica obligatoriamente lo mantendrá APAGADO
      if (t > 26.0) {
        iot.state = true;
      } else if (t < 25.5) {
        iot.state = false;
      }
    }
  }
}

void applyOutputs() {
  // Escribir en PCF8574 (Relés I2C)
  uint8_t newPcf = 0;
  if (pump.state)   newPcf |= (1 << (2-1));
  if (heater.state) newPcf |= (1 << (7-1));
  if (humid.state)  newPcf |= (1 << (8-1));
  
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(newPcf);
  Wire.endTransmission();

  // Escribir en IoT Relay (Pin Digital)
  digitalWrite(PIN_IOT_RELAY, iot.state ? HIGH : LOW);
}

void loop() {
  unsigned long now = millis();
  mqtt.poll();

  // EJECUCIÓN CADA 1 SEGUNDO
  if (now - lastReadMs >= 1000) {
    lastReadMs = now;

    // Lectura de sensores
    float t = carrier.Env.readTemperature();
    float h = carrier.Env.readHumidity();
    uint16_t s = ss.touchRead(0);
    lastSoil = s;

    // Procesar y Actuar
    applyAutomation(t, h, s);
    applyOutputs();
    
    // Actualizar pantalla
    // updateScreen(t, h, s); 
  }

  // ENVÍO MQTT CADA 30 SEGUNDOS
  if (now - lastPubMs >= 30000) {
    lastPubMs = now;
    // publishSensors(t, h, s);
    // publishActuators();
  }
}