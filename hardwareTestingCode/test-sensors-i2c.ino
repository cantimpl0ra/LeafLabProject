/*********************************************************************
   TEST MÍNIMO I2C – MKR WiFi 1010 + PaHub
   Componentes:
     - PaHub
     - Sensor humedad suelo Adafruit 4026 (I2C)
     - Sensor ultrasónico I2C (M5 / compatible)
 *********************************************************************/

#include <Wire.h>
#include <Adafruit_seesaw.h>

// ===== PaHub =====
#define PAHUB_ADDR 0x70

// ===== Canales PaHub =====
#define CH_SOIL       3
#define CH_ULTRASONIC 4

// ===== Direcciones I2C =====
#define SOIL_ADDR       0x36
#define ULTRASONIC_ADDR 0x57

Adafruit_seesaw soil;

/************ Seleccionar canal PaHub ************/
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/************ Escaneo I2C en un canal ************/
void scanChannel(uint8_t ch) {
  Serial.print("\nCanal ");
  Serial.print(ch);
  Serial.println(":");

  selectChannel(ch);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  I2C detectado -> 0x");
      Serial.println(addr, HEX);
    }
  }
}

/************ Leer ultrasonidos (I2C simple) ************/
uint16_t readUltrasonic() {
  selectChannel(CH_ULTRASONIC);

  Wire.beginTransmission(ULTRASONIC_ADDR);
  Wire.write(0x00);   // registro típico de lectura
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

  Serial.println("\n=== TEST I2C MINIMO ===");

  // -------- Escanear PaHub --------
  Serial.println("\n--- ESCANEO PAHUB ---");
  for (int ch = 0; ch < 6; ch++) {
    scanChannel(ch);
  }

  // -------- Test sensor suelo --------
  Serial.println("\n--- TEST SENSOR HUMEDAD SUELO ---");
  selectChannel(CH_SOIL);
  if (!soil.begin(SOIL_ADDR)) {
    Serial.println("ERROR: Sensor de suelo NO detectado");
  } else {
    Serial.println("OK: Sensor de suelo detectado");
  }

  // -------- Test ultrasonidos --------
  Serial.println("\n--- TEST ULTRASONIDOS ---");
  selectChannel(CH_ULTRASONIC);
  Wire.beginTransmission(ULTRASONIC_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("OK: Sensor ultrasonidos detectado");
  } else {
    Serial.println("ERROR: Ultrasonidos NO detectado");
  }
}

void loop() {

  // ===== Humedad suelo =====
  selectChannel(CH_SOIL);
  uint16_t soilRaw = soil.touchRead(0);

  // ===== Ultrasonidos =====
  uint16_t distance = readUltrasonic();

  Serial.print("Suelo RAW: ");
  Serial.print(soilRaw);
  Serial.print(" | Distancia: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(1000);
}
