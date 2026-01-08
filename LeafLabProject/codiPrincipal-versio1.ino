/***********************************************************************
  LEAF LAB – Automation v2.5
  Buttons FIXED with edge detection (getTouch)

  ✔ Buttons WORKING (edge detection)
  ✔ Manual / AUTO per actuator
  ✔ I2C relays + IoT relay controlled
  ✔ Ready to keep automation, MQTT, etc.
***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <Wire.h>

MKRIoTCarrier carrier;

/* ================= IOT RELAY ================= */
const int PIN_IOT_RELAY = 2;

/* ================= I2C RELAY (PCF8574) ================= */
#define PAHUB_ADDR 0x70
#define PCF_ADDR   0x27
#define CH_PCF     5

const uint8_t RELAYNUM_PUMP   = 2;
const uint8_t RELAYNUM_HEATER = 7;
const uint8_t RELAYNUM_HUMID  = 8;

// Direct logic: 1 = ON, 0 = OFF
uint8_t pcfState = 0x00;

/* ================= ACTUATOR STRUCT ================= */
struct Actuator {
  bool manual;   // true = MANUAL
  bool state;    // true = ON
};

Actuator pump   = {false, false};
Actuator humid  = {false, false};
Actuator heater = {false, false};
Actuator iot    = {false, false};

/* ================= BUTTON EDGE STATE ================= */
bool lastTouch[4] = {false, false, false, false};

/* ================= PAHUB + PCF ================= */
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void writePCF() {
  selectChannel(CH_PCF);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();
}

void setI2CRelay(uint8_t relayNum, bool on) {
  uint8_t bitIndex = relayNum - 1;
  if (on)  pcfState |=  (1 << bitIndex);
  else     pcfState &= ~(1 << bitIndex);
  writePCF();
}

/* ================= IOT RELAY ================= */
void setIoTRelay(bool on) {
  digitalWrite(PIN_IOT_RELAY, on ? HIGH : LOW);
}

/* ================= MANUAL TOGGLE ================= */
void toggleManual(Actuator &a) {
  if (!a.manual) {
    a.manual = true;
    a.state  = true;   // MANUAL => ON
  } else {
    a.manual = false;
    a.state  = false;  // AUTO => OFF (automation decidirá)
  }
}

/* ================= APPLY OUTPUTS ================= */
void applyOutputs() {
  setI2CRelay(RELAYNUM_PUMP,   pump.state);
  setI2CRelay(RELAYNUM_HUMID,  humid.state);
  setI2CRelay(RELAYNUM_HEATER, heater.state);
  setIoTRelay(iot.state);
}

/* ================= BUTTONS (EDGE DETECTION) ================= */
void handleButtons() {

  bool touch[4];
  touch[0] = carrier.Buttons.getTouch(TOUCH0); // Bomba
  touch[1] = carrier.Buttons.getTouch(TOUCH1); // Humidificador
  touch[2] = carrier.Buttons.getTouch(TOUCH2); // Calefactor
  touch[3] = carrier.Buttons.getTouch(TOUCH3); // IoT Relay

  // Rising edge detection
  if (touch[0] && !lastTouch[0]) toggleManual(pump);
  if (touch[1] && !lastTouch[1]) toggleManual(humid);
  if (touch[2] && !lastTouch[2]) toggleManual(heater);
  if (touch[3] && !lastTouch[3]) toggleManual(iot);

  // Save state
  for (int i = 0; i < 4; i++) lastTouch[i] = touch[i];

  applyOutputs();
}

/* ================= SETUP ================= */
void setup() {
  pinMode(PIN_IOT_RELAY, OUTPUT);
  digitalWrite(PIN_IOT_RELAY, LOW);

  Wire.begin();

  // Force all I2C relays OFF
  selectChannel(CH_PCF);
  pcfState = 0x00;
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(pcfState);
  Wire.endTransmission();

  carrier.begin();

  pump.manual = humid.manual = heater.manual = iot.manual = false;
  pump.state  = humid.state  = heater.state  = iot.state  = false;

  applyOutputs();
}

/* ================= LOOP ================= */
void loop() {
  carrier.Buttons.update();   // OBLIGATORIO
  handleButtons();

  // Aquí volveremos a integrar:
  // - automatización
  // - MQTT
  // - sensores
  // cuando tú quieras

  delay(50);
}
