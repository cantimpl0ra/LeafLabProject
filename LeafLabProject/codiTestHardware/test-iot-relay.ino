/***********************************************************************
   IOT RELAY TEST - LEAF LAB PROJECT
 ***********************************************************************/

#define RELAY_PIN 2

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  digitalWrite(RELAY_PIN, LOW);
  delay(5000);
  digitalWrite(RELAY_PIN, HIGH);  // ON
  delay(5000);                    // 5 seconds
  digitalWrite(RELAY_PIN, LOW);   // OFF
  delay(20000);
}
