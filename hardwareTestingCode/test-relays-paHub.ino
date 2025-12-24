/***********************************************************************
   I2C RELAY TEST – LEAF LAB PROJECT
     ✔ PCF8574 relay test (1–8), each with 4 cycles, ON = 10s
     ✔ Temperature and humidity reading from IoT Carrier
     ✔ Display on Serial + Screen
 ***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>

MKRIoTCarrier carrier;

// PaHub address
#define PAHUB_ADDR 0x70

// PCF8574 address and channel where it is connected
#define PCF_ADDR   0x27
#define CH_PCF     5

uint8_t pcfState = 0xFF;   // inverted logic on this module

/*********** Select PaHub channel ***********/
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/*********** Scan a channel ***********/
void scanChannel(uint8_t ch) {
  Serial.print("\n Channel ");
  Serial.println(ch);

  selectChannel(ch);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" → I2C detected: 0x");
      Serial.println(addr, HEX);
    }
  }
}

/*********** Toggle a relay ***********/
void toggleRelay(uint8_t relay) {
  uint8_t bit = relay - 1;
  pcfState ^= (1 << bit);

  selectChannel(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(~pcfState);   // PCF uses inverted logic
  Wire.endTransmission();
}

/***********************************************************************/

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  carrier.begin();

  Serial.println("\n=== I2C RELAY TEST ===");

  carrier.display.fillScreen(0x0000);
  carrier.display.setTextColor(0xFFFF);
  carrier.display.setTextSize(2);
  carrier.display.setCursor(0, 0);
  carrier.display.println("I2C Relay test");

  /******** PaHub scan ********/
  Serial.println("\n--- SCANNING PAHUB ---");
  for (int ch = 0; ch < 6; ch++) {
    scanChannel(ch);
  }

  /******** Test relays ********/
  Serial.println("\n--- TESTING RELAYS (1-8), ON=10s (4 cycles) ---");

  for (int r = 1; r <= 8; r++) {
    Serial.print("\nRelay ");
    Serial.print(r);
    Serial.println(" — Test cycle:");

    for (int cycle = 1; cycle <= 4; cycle++) {
      
      Serial.print("  Cycle ");
      Serial.print(cycle);
      Serial.println(": ON (10s) → OFF");

      // Turn relay ON
      toggleRelay(r);   
      delay(10000);     // Keep ON for 10 seconds

      // Turn relay OFF
      toggleRelay(r);   
      delay(500);       // Visual pause between cycles
    }

    Serial.print("Relay ");
    Serial.p
