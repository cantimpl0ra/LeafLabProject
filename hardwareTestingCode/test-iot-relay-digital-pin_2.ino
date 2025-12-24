#define RELAY_PIN 2

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  digitalWrite(RELAY_PIN, LOW);
  delay(5000);
  digitalWrite(RELAY_PIN, HIGH);  // ACTIVA el relé
  delay(5000);                    // 5 segundos
  digitalWrite(RELAY_PIN, LOW);   // DESACTIVA el relé
  delay(20000);
}
