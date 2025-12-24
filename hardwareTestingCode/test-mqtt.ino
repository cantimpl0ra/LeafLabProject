#include <Arduino_MKRIoTCarrier.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

MKRIoTCarrier carrier;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Buffers
String wifiSSID = "";
String wifiPASS = "";
String brokerIP = "";

void pedirConfiguracion() {
  Serial.println("\n=== CONFIGURACION MQTT ===");
  Serial.println("Introduce:");
  Serial.println("SSID=tu_wifi");
  Serial.println("PASS=tu_password");
  Serial.println("IP=ip_broker");
  Serial.println("Escribe START para comenzar\n");

  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("SSID=")) {
        wifiSSID = line.substring(5);
        Serial.println("SSID guardado");
      }
      else if (line.startsWith("PASS=")) {
        wifiPASS = line.substring(5);
        Serial.println("Password guardado");
      }
      else if (line.startsWith("IP=")) {
        brokerIP = line.substring(3);
        Serial.println("IP broker guardada");
      }
      else if (line == "START") {
        if (wifiSSID != "" && wifiPASS != "" && brokerIP != "") {
          break;
        } else {
          Serial.println("Faltan datos!");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  carrier.begin();

  pedirConfiguracion();

  // --- WIFI ---
  Serial.print("Conectando a WiFi...");
  WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" OK");

  // --- MQTT ---
  Serial.print("Conectando al broker MQTT...");
  if (!mqttClient.connect(brokerIP.c_str(), 1883)) {
    Serial.println(" ERROR");
    while (1);
  }
  Serial.println(" OK");
}

void loop() {
  mqttClient.poll();

  mqttClient.beginMessage("invernadero/test");
  mqttClient.print("Hola desde MKR IoT Carrier");
  mqttClient.endMessage();

  Serial.println("Mensaje MQTT enviado");
  delay(5000);
}
