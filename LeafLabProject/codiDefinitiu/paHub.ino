#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedHUB.h>
#include "Wire.h"

namespace {
m5::unit::UnitUnified Units;
m5::unit::UnitPaHub2 hub0{0x77};  // 0x70 as default, but we change to 0x77
}  // namespace

void setup()
{
    M5.begin();
    M5.Display.setFont(&fonts::FreeMonoBold12pt7b);
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);

    Wire.begin(pin_num_sda, pin_num_scl, 400000U);

    if (!Units.add(hub0, Wire) ||  // Connect hub0 to core
        !Units.begin()) {
        M5_LOGE("Failed to begin");
        M5.Display.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
}

void scan_ch(uint8_t ch)
{
    M5.Display.clear();
    int textColor = YELLOW;
    for (size_t i = 0; i < 2; i++) {
        M5.Display.setCursor(0, 0);
        M5.Display.print("scanning Address [HEX]\r\n");
        M5.Display.printf("Pahub Channel: %d\r\n", ch);
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            uint8_t error = Wire.endTransmission();
            if (error == 0) {
                M5.Display.print(addr, HEX);
                M5.Display.print(" ");
            } else {
                M5.Display.print(".");
            }
            delay(10);
        }

        if (textColor == YELLOW) {
            textColor = CYAN;
        } else {
            textColor = YELLOW;
        }
        M5.Display.setTextColor(textColor, BLACK);
    }
}

void loop()
{
    M5.update();
    for (uint8_t i = 0; i < 6; i++) {
        // Select & Scan Each Channel
        hub0.selectChannel(i);
        scan_ch(i);
    }
} 