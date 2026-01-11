/*********************************************************************
  MKR WiFi 1010 + MKR IoT Carrier + PaHub TEST
  - PaHub (TCA9548A) @ 0x70
  - Soil I2C (seesaw) @ 0x36  -> PaHub channel CH_SOIL   (pon tu X)
  - ADS1115 @ 0x48            -> PaHub channel 0

  ADS1115 Analog mapping:
  - ADS A0: Air Quality (analog)
  - ADS A1: pH water (analog)
  - ADS A2: TDS water (analog)
*********************************************************************/

#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_ADS1X15.h>

// ===== PaHub =====
#define PAHUB_ADDR 0x70

// ===== PaHub Channels =====
#define CH_ADS   0        // ADS1115 SIEMPRE en canal 0 (como has dicho)
#define CH_SOIL  3        // <-- CAMBIA ESTO por tu canal X (0..7)

// ===== I2C Addresses =====
#define ADS1115_ADDR 0x48
#define SOIL_ADDR    0x36

Adafruit_ADS1115 ads;
Adafruit_seesaw soil;

// ---- Utilidades ----
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool selectPahubChannel(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  return (Wire.endTransmission() == 0);
}

// V/bit según ganancia (para convertir raw->V)
float voltsPerBit(adsGain_t gain) {
  switch (gain) {
    case GAIN_TWOTHIRDS: return 0.1875e-3;
    case GAIN_ONE:       return 0.1250e-3;
    case GAIN_TWO:       return 0.0625e-3;
    case GAIN_FOUR:      return 0.03125e-3;
    case GAIN_EIGHT:     return 0.015625e-3;
    case GAIN_SIXTEEN:   return 0.0078125e-3;
    default:             return 0.1250e-3;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(100000);  // recomendado en MKR + Carrier + I2C cargado

  Serial.println("\n=== PAHUB + SOIL + ADS1115 (A0/A1/A2) TEST ===");

  // ---- Comprobar PaHub ----
  Serial.print("PaHub @0x70: ");
  if (i2cPing(PAHUB_ADDR)) Serial.println("OK");
  else {
    Serial.println("NOT FOUND (revisa I2C del carrier -> PaHub)");
    while (1) delay(100);
  }

  // ---- ADS1115 (canal 0) ----
  Serial.println("\n--- ADS1115 TEST (PaHub channel 0) ---");
  if (!selectPahubChannel(CH_ADS)) {
    Serial.println("ERROR: No puedo seleccionar canal ADS en PaHub");
    while (1) delay(100);
  }
  delay(2);

  // Probar varias direcciones típicas por si ADDR no está en GND
  uint8_t foundAddr = 0;
  bool found = false;
  for (uint8_t addr = 0x48; addr <= 0x4B; addr++) {
    if (i2cPing(addr)) { foundAddr = addr; found = true; break; }
  }
  if (!found) {
    Serial.println("ERROR: ADS1115 no responde en 0x48..0x4B (en canal 0).");
    Serial.println("Revisa VDD/GND del ADS y el pin ADDR.");
    while (1) delay(100);
  }

  Serial.print("ADS1115 detectada en 0x");
  Serial.println(foundAddr, HEX);

  if (!ads.begin(foundAddr)) {
    Serial.println("ERROR: ads.begin() falló (bus inestable o módulo).");
    while (1) delay(100);
  }

  // Ganancia recomendada si tu ADS está alimentada a 3.3V:
  // GAIN_ONE te da buena resolución; OJO: la señal NO debe superar ~3.3V.
  ads.setGain(GAIN_ONE);

  Serial.print("ADS gain: GAIN_ONE | V/bit=");
  Serial.println(voltsPerBit(GAIN_ONE), 9);

  // ---- Soil (canal X) ----
  Serial.println("\n--- SOIL I2C TEST (PaHub CH_SOIL) ---");
  Serial.print("CH_SOIL definido como: ");
  Serial.println(CH_SOIL);

  if (!selectPahubChannel(CH_SOIL)) {
    Serial.println("ERROR: No puedo seleccionar canal SOIL en PaHub");
  } else {
    delay(2);
    if (!soil.begin(SOIL_ADDR)) {
      Serial.println("ERROR: Soil sensor NO detectado (0x36)");
    } else {
      Serial.println("OK: Soil sensor detectado (0x36)");
    }
  }

  Serial.println("\nLecturas cada 1s:");
  Serial.println("ADS A0=AirQuality | A1=pH | A2=TDS  + Soil RAW");
  Serial.println("IMPORTANTE: Si pH/TDS/AirQuality dan salida >3.3V, usa divisor antes del ADS.");
}

void loop() {
  // ===== ADS1115 =====
  // Aseguramos canal 0 antes de leer el ADS
  if (!selectPahubChannel(CH_ADS)) {
    Serial.println("ERROR: no puedo seleccionar canal ADS");
    delay(1000);
    return;
  }
  delay(2);

  int16_t airRaw = ads.readADC_SingleEnded(0); // ADS A0
  int16_t phRaw  = ads.readADC_SingleEnded(1); // ADS A1
  int16_t tdsRaw = ads.readADC_SingleEnded(2); // ADS A2

  float vpb = voltsPerBit(GAIN_ONE);
  float airV = airRaw * vpb;
  float phV  = phRaw  * vpb;
  float tdsV = tdsRaw * vpb;

  // ===== SOIL =====
  uint16_t soilRaw = 0;
  if (selectPahubChannel(CH_SOIL)) {
    delay(2);
    // si soil no está inicializado, devolverá 0/valor raro; por eso en setup avisamos
    soilRaw = soil.touchRead(0);
  }

  // ===== Print =====
  Serial.print("AIR A0: raw=");
  Serial.print(airRaw);
  Serial.print(" V=");
  Serial.print(airV, 4);

  Serial.print(" | pH A1: raw=");
  Serial.print(phRaw);
  Serial.print(" V=");
  Serial.print(phV, 4);

  Serial.print(" | TDS A2: raw=");
  Serial.print(tdsRaw);
  Serial.print(" V=");
  Serial.print(tdsV, 4);

  Serial.print(" | SOIL raw=");
  Serial.print(soilRaw);

  // Saturación aproximada
  if (airRaw > 32000 || phRaw > 32000 || tdsRaw > 32000) {
    Serial.print("  [SATURANDO: voltaje alto]");
  }

  Serial.println();
  delay(1000);
}
