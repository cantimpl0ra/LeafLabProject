/***********************************************************************
   MQTT CONNECTION TEST - LEAF LAB PROJECT
   You have to input the mqtt setup configuration one by one.
 ***********************************************************************/

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

void askConfig() {
  Serial.println("\n=== MQTT CONFIGURATION ===");
  Serial.println("Enter:");
  Serial.println("SSID=your_wifi");
  Serial.println("PASS=your_password");
  Serial.println("IP=broker_ip");
  Serial.println("Type START to begin\n");

  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("SSID=")) {
        wifiSSID = line.substring(5);
        Serial.println("SSID saved");
      }
      else if (line.startsWith("PASS=")) {
        wifiPASS = line.substring(5);
        Serial.println("Password saved");
      }
      else if (line.startsWith("IP=")) {
        brokerIP = line.substring(3);
        Serial.println("Broker IP saved");
      }
      else if (line == "START") {
        if (wifiSSID != "" && wifiPASS != "" && brokerIP != "") {
          break;
        } else {
          Serial.println("Missing data!");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  carrier.begin();

  askConfig();

  // --- WIFI ---
  Serial.print("Connecting to WiFi...");
  WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" OK");

  // --- MQTT ---
  Serial.print("Connecting to MQTT broker...");
  if (!mqttClient.connect(brokerIP.c_str(), 1883)) {
    Serial.println(" ERROR");
    while (1);
  }
  Serial.println(" OK");
}

void loop() {
  mqttClient.poll();

  mqttClient.beginMessage("invernadero/test");
  mqttClient.print("Hello from MKR IoT Carrier");
  mqttClient.endMessage();

  Serial.println("MQTT message sent");
  delay(5000);
}
