/*********************************************************************
   I2C SENSORS TEST – LEAF LAB PROJECT
   Components:
     - PaHub
     - Soil Moisture Sensor (I2C)
     - I2C Ultrasonic Sensor
     - ADS1115 (I2C) on PaHub channel A0
 *********************************************************************/

#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_ADS1X15.h>

// ===== PaHub =====
#define PAHUB_ADDR 0x70

// ===== PaHub Channels =====
#define CH_ADS        0   // A0 (normalmente canal 0)
#define CH_SOIL       3
#define CH_ULTRASONIC 4

// ===== I2C Addresses =====
#define SOIL_ADDR       0x36
#define ULTRASONIC_ADDR 0x57
#define ADS1115_ADDR    0x48  // Dirección por defecto del ADS1115 (ADDR a GND)

// ===== Object creation =====
Adafruit_seesaw soil;
Adafruit_ADS1115 ads;

// ===== FUNCTIONS =====

/************ Select PaHub channel ************/
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/************ I2C scan on a channel ************/
void scanChannel(uint8_t ch) {
  Serial.print("\nChannel ");
  Serial.print(ch);
  Serial.println(":");

  selectChannel(ch);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  I2C detected -> 0x");
      Serial.println(addr, HEX);
    }
  }
}

/************ Read ultrasonic sensor (simple I2C) ************/
uint16_t readUltrasonic() {
  selectChannel(CH_ULTRASONIC);

  Wire.beginTransmission(ULTRASONIC_ADDR);
  Wire.write(0x00);   // typical read register
  Wire.endTransmission();

  Wire.requestFrom(ULTRASONIC_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t dist = Wire.read() << 8;
    dist |= Wire.read();
    return dist;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  Serial.println("\n=== MINIMAL I2C TEST ===");

  // -------- Scan PaHub --------
  Serial.println("\n--- PAHUB SCAN ---");
  for (int ch = 0; ch < 6; ch++) {
    scanChannel(ch);
  }

  // -------- ADS1115 test --------
  Serial.println("\n--- ADS1115 TEST (PaHub CH A0) ---");
  selectChannel(CH_ADS);

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ERROR: ADS1115 NOT detected on channel A0");
  } else {
    Serial.println("OK: ADS1115 detected on channel A0");
    // lectura rápida del canal A0 del ADS1115
    int16_t adc0 = ads.readADC_SingleEnded(0);
    Serial.print("ADS1115 A0 raw: ");
    Serial.println(adc0);
  }

  // -------- Soil sensor test --------
  Serial.println("\n--- SOIL MOISTURE SENSOR TEST ---");
  selectChannel(CH_SOIL);
  if (!soil.begin(SOIL_ADDR)) {
    Serial.println("ERROR: Soil sensor NOT detected");
  } else {
    Serial.println("OK: Soil sensor detected");
  }

  // -------- Ultrasonic test --------
  Serial.println("\n--- ULTRASONIC SENSOR TEST ---");
  selectChannel(CH_ULTRASONIC);
  Wire.beginTransmission(ULTRASONIC_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("OK: Ultrasonic sensor detected");
  } else {
    Serial.println("ERROR: Ultrasonic sensor NOT detected");
  }
}

void loop() {

  // ===== ADS1115 =====
  selectChannel(CH_ADS);
  int16_t adsA0 = 0;
  // si no está conectado, normalmente seguirá devolviendo algo, pero lo importante es que NO cuelgue y responda
  adsA0 = ads.readADC_SingleEnded(0);

  // ===== Soil moisture =====
  selectChannel(CH_SOIL);
  uint16_t soilRaw = soil.touchRead(0);

  // ===== Ultrasonic =====
  uint16_t distance = readUltrasonic();

  Serial.print("ADS A0: ");
  Serial.print(adsA0);
  Serial.print(" | Soil RAW: ");
  Serial.print(soilRaw);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(1000);
}
