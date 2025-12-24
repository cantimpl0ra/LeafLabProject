/***********************************************************************
   TEST MKR IOT CARRIER + SD - LEAF LAB PROJECT
   Options:
     T0 → Screen test
     T1 → Button test
     T2 → SD test
     T3 → Run ALL tests
     T4 → Return to menu
 ***********************************************************************/

#include <Arduino_MKRIoTCarrier.h>
#include <SD.h>

MKRIoTCarrier carrier;

/******************** UTILITIES *************************/
void clearScreen(const char* title) {
  carrier.display.fillScreen(0x0000);
  carrier.display.setTextColor(0xFFFF);
  carrier.display.setTextSize(2);
  carrier.display.setCursor(10, 10);
  carrier.display.println(title);
}

void waitReturn() {
  carrier.display.println("\nPress T4 to return");
  while (true) {
    carrier.Buttons.update();
    if (carrier.Buttons.getTouch(TOUCH4)) {
      delay(300);
      break;
    }
    delay(100);
  }
}

/******************** SCREEN TEST *************************/
bool testScreen() {
  clearScreen("Screen Test");

  carrier.display.fillScreen(0xF800); delay(1000); // Red
  carrier.display.fillScreen(0x07E0); delay(1000); // Green
  carrier.display.fillScreen(0x001F); delay(1000); // Blue

  carrier.display.fillScreen(0x0000);
  carrier.display.setCursor(10, 40);
  carrier.display.println("Screen OK");

  waitReturn();
  return true;
}

/******************** BUTTON TEST *************************/
bool testButtons() {
  clearScreen("Button Test");
  carrier.display.println("\nPress T0-T4");

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

/******************** SD TEST *************************/
bool testSD() {
  clearScreen("SD Test");

  File f = SD.open("test_sd.txt", FILE_WRITE);
  if (!f) {
    carrier.display.println("\nSD ERROR");
    waitReturn();
    return false;
  }

  f.println("SD CARD TEST OK");
  f.close();

  carrier.display.println("\nSD OK");
  waitReturn();
  return true;
}

/******************** FULL TEST *************************/
void testTodos() {
  clearScreen("Full Test");

  bool p = testScreen();
  bool b = testButtons();
  bool s = testSD();

  File f = SD.open("results.txt", FILE_WRITE);
  if (f) {
    f.print("screen=");  f.println(p ? "OK" : "FAIL");
    f.print("buttons="); f.println(b ? "OK" : "FAIL");
    f.print("sd=");      f.println(s ? "OK" : "FAIL");
    f.close();
  }

  clearScreen("Results");
  carrier.display.println(p ? "Screen OK"   : "Screen FAIL");
  carrier.display.println(b ? "Buttons OK"  : "Buttons FAIL");
  carrier.display.println(s ? "SD OK"       : "SD FAIL");

  waitReturn();
}

/******************** MENU *************************/
void showMenu() {
  clearScreen("TEST MENU");
  carrier.display.println("\nT0 Screen");
  carrier.display.println("T1 Buttons");
  carrier.display.println("T2 SD");
  carrier.display.println("T3 All");
}

/******************** SETUP *************************/
void setup() {
  Serial.begin(115200);
  carrier.begin();   // SD is initialized HERE

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
    testScreen();
    showMenu();
  }
  if (carrier.Buttons.getTouch(TOUCH1)) {
    delay(300);
    testButtons();
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
