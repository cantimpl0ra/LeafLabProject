/*********************************************************************
   ANALOG SENSORS TEST – LEAF LAB PROJECT
   Components:
     - PaHub
     - ADS1115 ADC
     - pH Sensor (A0)
     - MQ135 Air Quality Sensor (A1)
     - Gravity TDS Sensor SEN0244 (A2)
*********************************************************************/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ===== PaHub =====
#define PAHUB_ADDR 0x70
#define CH_ADS     0      // ADS1115 connected to PaHub channel 0

// ===== ADS1115 =====
#define ADS_ADDR   0x48

Adafruit_ADS1115 ads;

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  Serial.println("\n=== ANALOG SENSORS TEST (ADS1115) ===");

  // -------- Scan PaHub --------
  Serial.println("\n--- PAHUB SCAN ---");
  for (int ch = 0; ch < 6; ch++) {
    scanChannel(ch);
  }

  // -------- ADS1115 init --------
  Serial.println("\n--- ADS1115 TEST ---");
  selectChannel(CH_ADS);

  ads.setGain(GAIN_ONE);  // ±4.096V (safe for 3.3V systems)

  if (!ads.begin(ADS_ADDR)) {
    Serial.println("ERROR: ADS1115 NOT detected");
    while (1);
  } else {
    Serial.println("OK: ADS1115 detected");
  }
}

void loop() {

  selectChannel(CH_ADS);

  // ===== Read analog sensors =====
  int16_t rawPH   = ads.readADC_SingleEnded(0); // A0
  int16_t rawMQ   = ads.readADC_SingleEnded(1); // A1
  int16_t rawTDS  = ads.readADC_SingleEnded(2); // A2

  float vPH  = ads.computeVolts(rawPH);
  float vMQ  = ads.computeVolts(rawMQ);
  float vTDS = ads.computeVolts(rawTDS);

  // ===== Serial output =====
  Serial.println("\n--- ADC READINGS ---");

  Serial.print("A0 pH  | RAW: ");
  Serial.print(rawPH);
  Serial.print(" | V: ");
  Serial.println(vPH, 3);

  Serial.print("A1 MQ  | RAW: ");
  Serial.print(rawMQ);
  Serial.print(" | V: ");
  Serial.println(vMQ, 3);

  Serial.print("A2 TDS | RAW: ");
  Serial.print(rawTDS);
  Serial.print(" | V: ");
  Serial.println(vTDS, 3);

  delay(1000);
}
