/*
  MKR IoT Carrier Smart Garden Setup Project

  A setup that allows for remote/local control of
  a pump, as well as reading sensors (moisture,
  temperature, humidity).

  Built using the Arduino Cloud service.

  Components used:
  - MKR WiFi 1010
  - MKR IoT Carrier
  - Moisture Sensor
  - 5V water pump
  - USB Adapter with 2x slots
  - Micro USB Cable
  - Open ended USB Cable
  - Grove cable 

  Code by (c) Alessandro Ranelucci for Arduino.
*/

#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino_MKRIoTCarrier.h>
#include <Arduino_OplaUI.h>

const int moistPin = A6;

unsigned long startedWatering;

MKRIoTCarrier opla;
CycleWidgetsApp app;
Gauge2_Widget moistureWidget;
Bool_Widget wateringToggleWidget;

void setup() {
  Serial.begin(9600);
  delay(1500);

  // Make sure the pump is not running
  stopWatering();

  // Connect to Arduino Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

  // Configure widgets
  moistureWidget.attachValue(moisture);
  moistureWidget.setTitle("MOISTURE");
  moistureWidget.setRange(0, 100);
  moistureWidget.setDigits(0);
  moistureWidget.setSuffix(" %");
  moistureWidget.setReadOnly(true);

  wateringToggleWidget.attachValue(watering);
  wateringToggleWidget.setTitle("PUMP");
  wateringToggleWidget.onValueChange(onWateringChange);
  
  // Initialize Oplà
  CARRIER_CASE = true;
  opla.begin();

  // Initialize the widget application
  app.begin(opla);
  app.addWidget(moistureWidget);
  app.addWidget(wateringToggleWidget);
}

void loop() {
  ArduinoCloud.update();
  app.loop();
  
  // Read the sensor and convert its value to a percentage 
  // (0% = dry; 100% = wet)
  int raw_moisture = analogRead(moistPin);
  moisture = map(raw_moisture, 0, 1023, 0, 100);

  temperature = opla.Env.readTemperature();
  humidity = opla.Env.readHumidity();

  // Set the LED color according to the moisture percentage
  if (moisture > 40) {
    opla.leds.setPixelColor(1, 50, 0 , 0);  // green
  } else if (moisture > 10) {
    opla.leds.setPixelColor(1, 50, 50 , 0); // yellow
  } else {
    opla.leds.setPixelColor(1, 0, 50 , 0);  // red
  }
  opla.leds.show();
  
  // Stop watering after the configured duration
  if (watering && (millis() - startedWatering) >= waterTime*1000) {
    stopWatering();
  }
  
  delay(100);
}

// This function is triggered whenever the server sends a change event,
// which means that someone changed a value remotely and we need to do
// something.
void onWateringChange() {
  if (watering) {
    startWatering();
  } else {
    stopWatering();
  }
}

void startWatering () {
  watering = true;
  startedWatering = millis();
  opla.Relay2.open();
}

void stopWatering () {
  watering = false;
  opla.Relay2.close();
}

void onWaterTimeChange()  {
  // Add your code here to act upon WaterTime change
}