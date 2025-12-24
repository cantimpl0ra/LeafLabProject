/***********************************************************************
   MENU DE TESTS – MKR IOT CARRIER (DEFINITIVO Y FUNCIONAL)
   Opciones:
     T0 → Test pantalla
     T1 → Test botones
     T2 → Test SD
     T3 → Ejecutar TODOS los tests
     T4 → Volver al menú
 ***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <SD.h>

MKRIoTCarrier carrier;

/******************** UTILIDADES *************************/
void clearScreen(const char* title) {
  carrier.display.fillScreen(0x0000);
  carrier.display.setTextColor(0xFFFF);
  carrier.display.setTextSize(2);
  carrier.display.setCursor(10, 10);
  carrier.display.println(title);
}

void waitReturn() {
  carrier.display.println("\nPulsa T4 para volver");
  while (true) {
    carrier.Buttons.update();
    if (carrier.Buttons.getTouch(TOUCH4)) {
      delay(300);
      break;
    }
    delay(100);
  }
}

/******************** TEST PANTALLA *************************/
bool testPantalla() {
  clearScreen("Test Pantalla");

  carrier.display.fillScreen(0xF800); delay(1000); // Rojo
  carrier.display.fillScreen(0x07E0); delay(1000); // Verde
  carrier.display.fillScreen(0x001F); delay(1000); // Azul

  carrier.display.fillScreen(0x0000);
  carrier.display.setCursor(10, 40);
  carrier.display.println("Pantalla OK");

  waitReturn();
  return true;
}

/******************** TEST BOTONES *************************/
bool testBotones() {
  clearScreen("Test Botones");
  carrier.display.println("\nPulsa T0-T4");

  bool pressed[5] = {false, false, false, false, false};

  while (true) {
    carrier.Buttons.update();

    if (carrier.Buttons.getTouch(TOUCH0)) pressed[0] = true;
    if (carrier.Buttons.getTouch(TOUCH1)) pressed[1] = true;
    if (carrier.Buttons.getTouch(TOUCH2)) pressed[2] = true;
    if (carrier.Buttons.getTouch(TOUCH3)) pressed[3] = true;
    if (carrier.Buttons.getTouch(TOUCH4)) pressed[4] = true;

    carrier.display.fillRect(10, 60, 220, 120, 0x0000);
    carrier.display.setCursor(10, 60);

    for (int i = 0; i < 5; i++) {
      carrier.display.print("T");
      carrier.display.print(i);
      carrier.display.print(": ");
      carrier.display.println(pressed[i] ? "OK" : "--");
    }

    if (pressed[0] && pressed[1] && pressed[2] && pressed[3] && pressed[4]) {
      delay(500);
      break;
    }

    delay(200);
  }

  waitReturn();
  return true;
}

/******************** TEST SD *************************/
bool testSD() {
  clearScreen("Test SD");

  File f = SD.open("test_sd.txt", FILE_WRITE);
  if (!f) {
    carrier.display.println("\nERROR SD");
    waitReturn();
    return false;
  }

  f.println("TEST SD CARD OK");
  f.close();

  carrier.display.println("\nSD OK");
  waitReturn();
  return true;
}

/******************** TEST COMPLETO *************************/
void testTodos() {
  clearScreen("Test Completo");

  bool p = testPantalla();
  bool b = testBotones();
  bool s = testSD();

  File f = SD.open("resultados.txt", FILE_WRITE);
  if (f) {
    f.print("pantalla="); f.println(p ? "OK" : "FAIL");
    f.print("botones=");  f.println(b ? "OK" : "FAIL");
    f.print("sd=");       f.println(s ? "OK" : "FAIL");
    f.close();
  }

  clearScreen("Resultados");
  carrier.display.println(p ? "Pantalla OK" : "Pantalla FAIL");
  carrier.display.println(b ? "Botones OK"  : "Botones FAIL");
  carrier.display.println(s ? "SD OK"       : "SD FAIL");

  waitReturn();
}

/******************** MENU *************************/
void showMenu() {
  clearScreen("MENU TESTS");
  carrier.display.println("\nT0 Pantalla");
  carrier.display.println("T1 Botones");
  carrier.display.println("T2 SD");
  carrier.display.println("T3 Todos");
}

/******************** SETUP *************************/
void setup() {
  Serial.begin(115200);
  carrier.begin();   // 🔴 SD se inicializa AQUÍ

  carrier.display.setRotation(0);
  carrier.display.fillScreen(0x0000);
  carrier.display.setTextColor(0xFFFF);
  carrier.display.setTextSize(2);
  delay(300);

  showMenu();
}

/******************** LOOP *************************/
void loop() {
  carrier.Buttons.update();

  if (carrier.Buttons.getTouch(TOUCH0)) {
    delay(300);
    testPantalla();
    showMenu();
  }
  if (carrier.Buttons.getTouch(TOUCH1)) {
    delay(300);
    testBotones();
    showMenu();
  }
  if (carrier.Buttons.getTouch(TOUCH2)) {
    delay(300);
    testSD();
    showMenu();
  }
  if (carrier.Buttons.getTouch(TOUCH3)) {
    delay(300);
    testTodos();
    showMenu();
  }

  delay(200);
}
