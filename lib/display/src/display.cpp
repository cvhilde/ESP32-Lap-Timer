#include <display.h>
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <globals.h>
#include <gps.h>
#include <Ticker.h>
#include <ble.h>
#include <storage.h>
#include <BLEDevice.h>

SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
uint16_t speedSamples[20];
int speedSampleIndex = 0;
Ticker led;
bool blinkActive = false;

// initializes the display while alos drawing the basic sector times
void initDisplay() {
    VextON();
    delay(100);
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, LOW);
    analogReadResolution(12);
    display.init();
    display.setFont(Roboto_Light_14);
    display.drawString(34, 4, "Lap Timer");
    display.drawString(58, 30, "By");
    display.drawString(7, 48, "Carter Hildebrandt");
    display.display();
    delay(5000);
}

// give power to the display
void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

void permDraws() {
    display.drawXbm(120, 4, 8, 8, satelliteBitmap);
    display.display();
}

void drawCurrentMode(bool current) {
    char line[5];
    if (!current) {
        sprintf(line, "L");
    } else {
        sprintf(line, "R");
    }

    display.setColor(BLACK);
    display.fillRect(100, 16, 28, 16);
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 16, line);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.display();
}

void drawStatusScreen() {
    char line1[30];
    String gpsStatus = "";

    int fixType = gps.getFixType();
    if (fixType == 0 || fixType == 1) {
        gpsStatus = "No Fix";
    } else if (fixType == 2) {
        gpsStatus = "Poor";
    } else if (fixType == 3) {
        gpsStatus = "Good";
    } else if (fixType > 3) {
        gpsStatus = "Great";
    }
    sprintf(line1, "GPS: %s", gpsStatus.c_str());

    char subLine1[10];
    sprintf(subLine1, "%d", getSatCount());

    char line2[30];
    sprintf(line2, "Storage: %.2f%%", storageUsage());

    char line3[30];
    String bleStatus = "";

    if (isAdvertising()) {
        bleStatus = "Advertising";
    } else if (isConnected()) {
        bleStatus = "Connected";
    } else {
        bleStatus = "Idle";
    }
    sprintf(line3, "BLE: %s", bleStatus.c_str());

    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);

    if (isSending()) {
        int totalFiles = getFileCount();
        int currentFile = getCurrentFileNumber();

        char line4[30];
        sprintf(line4, "Transfer File %d/%d", currentFile, totalFiles);

        display.setColor(WHITE);
        display.drawString(0, 48, line4);
    }  

    display.setColor(BLACK);
    display.fillRect(0, 0, 80, 16);
    display.fillRect(90, 0, 30, 16);
    display.fillRect(0, 16, 100, 16);
    display.fillRect(0, 32, 100, 16);
    display.setColor(WHITE);
    display.drawString(0, 0, line1);
    display.drawString(0, 16, line2);
    display.drawString(0, 32, line3);

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(120, 0, subLine1);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    
    display.display();
}

void displayPurgingMessage() {
    char line[] = "Purging Flash";
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.setColor(WHITE);
    display.drawString(0, 48, line);
    display.display();
}

void clearPurgingMessage() {
    display.setColor(BLACK);
    display.fillRect(0, 48, 128, 16);
    display.display();
}

void IRAM_ATTR onBlink() {
    if (blinkActive) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void startBlink(unsigned long interval) {
    pinMode(LED_PIN, OUTPUT);
    blinkActive = true;

    led.attach_ms(interval, onBlink);
}

void stopBlink() {
    blinkActive = false;
    led.detach();
    digitalWrite(LED_PIN, LOW);
}

void turnLEDOn() {
    digitalWrite(LED_PIN, HIGH);
}

void turnLEDOff() {
    digitalWrite(LED_PIN, LOW);
}