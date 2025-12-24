/***********************************************************************
   TEST MÍNIMO HARDWARE – Arduino Cloud Compatible
   Funciones:
     ✔ Escaneo PaHub
     ✔ Prueba relés PCF8574 (1–8), cada uno 4 ciclos con ON = 10s
     ✔ Lectura temperatura y humedad del IoT Carrier
     ✔ Mostrar en Serial + Pantalla
 ***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>

MKRIoTCarrier carrier;

// Dirección PaHub
#define PAHUB_ADDR 0x70

// Dirección PCF8574 y canal donde está conectado
#define PCF_ADDR   0x27
#define CH_PCF     5

uint8_t pcfState = 0xFF;   // lógica invertida en este módulo

/*********** Seleccionar canal del PaHub ***********/
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/*********** Escanear un canal ***********/
void scanChannel(uint8_t ch) {
  Serial.print("\n Canal ");
  Serial.println(ch);

  selectChannel(ch);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" → I2C detectado: 0x");
      Serial.println(addr, HEX);
    }
  }
}

/*********** Alternar un relé ***********/
void toggleRelay(uint8_t relay) {
  uint8_t bit = relay - 1;
  pcfState ^= (1 << bit);

  selectChannel(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(~pcfState);   // PCF usa lógica invertida
  Wire.endTransmission();
}

/***********************************************************************/

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  carrier.begin();

  Serial.println("\n=== TEST MÍNIMO HARDWARE ===");

  carrier.display.fillScreen(0x0000);
  carrier.display.setTextColor(0xFFFF);
  carrier.display.setTextSize(2);
  carrier.display.setCursor(0, 0);
  carrier.display.println("Test Minimo");

  /******** Escaneo PaHub ********/
  Serial.println("\n--- ESCANEANDO PAHUB ---");
  for (int ch = 0; ch < 6; ch++) {
    scanChannel(ch);
  }

  /******** Probar relés ********/
  Serial.println("\n--- PROBANDO RELÉS (1–8), ON=10s (4 ciclos) ---");

  for (int r = 1; r <= 8; r++) {
    Serial.print("\nRelé ");
    Serial.print(r);
    Serial.println(" — Ciclo de pruebas:");

    for (int ciclo = 1; ciclo <= 4; ciclo++) {
      
      Serial.print("  Ciclo ");
      Serial.print(ciclo);
      Serial.println(": ON (10s) → OFF");

      // Encender relé
      toggleRelay(r);   
      delay(10000);     // Mantener ON durante 10 segundos

      // Apagar relé
      toggleRelay(r);   
      delay(500);       // Pausa visual entre ciclos
    }

    Serial.print("Relé ");
    Serial.p
