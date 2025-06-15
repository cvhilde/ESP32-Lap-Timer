#include <display.h>
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <globals.h>
#include <gps.h>
#include <Ticker.h>

SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void drawSectorTime(int min, int sec, int tenths, int x, int y, int sector);
void drawLastLapTime(int min, int sec, int tenths);

uint16_t samples[20];
int avgIndex = 0;

unsigned long previousLapTime = 0;
unsigned long previousSect1 = 0;
unsigned long previousSect2 = 0;
unsigned long previousSect3 = 0;
unsigned long previousLastLap = 0;

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

void firstDraw() {
    display.clear();
    drawSectorTime(0, 0, 0, 0, 32, 1);
    drawSectorTime(0, 0, 0, 64, 32, 2);
    drawSectorTime(0, 0, 0, 0, 50, 3);
    drawLastLapTime(0, 0, 0);
    display.display();
}

// give power to the display
void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

// read the battery voltage and get a running average going to prevent voltage from visibly spiking
// due to inconsistent readings
float readBattVoltage() {
    digitalWrite(ADC_CTRL, HIGH);
    samples[avgIndex++] = analogRead(VBAT_Read);
    digitalWrite(ADC_CTRL, LOW);

    if (avgIndex >= 20) {
        avgIndex = 0;
    }
    uint32_t total = 0;
    for (int i = 0; i < 20; i++) {
        total += samples[i];
    }
    float avgRaw = total / 20.0;
    return avgRaw * (3.3 / 4095.0) * 5.215384 * 1.073655914; // Just 5.215384 for 2nd board
}

// basic status screen (no longer use)
void drawScreen(double lata, double longa, double distance) {
    display.clear();

    char str[30];
    char str2[30];
    char str3[30];

    int x = display.width()/2;
    int y = 0;
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(x, y, "Current Position");

    y = display.height() - 48;
    display.drawString(x, y, "   Latitude | Longitude");

    y = display.height() - 32;
    sprintf(str, "%.5lf | %.5lf", lata, longa);
    display.drawString(x, y, str);

    y = display.height() - 16;
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    float vcc = readBattVoltage();
    sprintf(str2, "%.2fV   Dist: %.2lf", vcc, distance);
    display.drawString(0, y, str2);

    display.display();
}

// draw only the distance
void drawDistance(double distance) {
    display.clear();
    char str[20];
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.width()/2, 0, "Distance to Waypoint " + String(currSector));
    sprintf(str, "%.2f Feet", distance);
    display.drawString(display.width()/2, 20, str);
    display.display();
}

// optimized lap time drawing logic
// will draw a black square where the time used to be, then draw the time
// allows screen updates without clearing and rewriting the entire screen
void drawLaptime(unsigned long lapTime, unsigned long sector1, unsigned long sector2, unsigned long sector3, unsigned long lastLap) {
    bool update = false;

    if (lapTime != previousLapTime) {
        previousLapTime = lapTime;
        int min = lapTime/60000;
        int sec = (lapTime % 60000) / 1000;
        int tenths = (lapTime % 1000) / 100;

        char curr[30];
        sprintf(curr, "%d:%02d.%d", min, sec, tenths);
        display.setColor(BLACK);
        display.fillRect(40, 5, 64, 14);
        display.setColor(WHITE);
        display.drawString(40, 5, curr);
        update = true;
    }

    if (sector1 != previousSect1) {
        previousSect1 = sector1;
        int min1 = sector1/60000;
        int sec1 = (sector1 % 60000) / 1000;
        int tenths1 = (sector1 % 1000) / 100;
        drawSectorTime(min1, sec1, tenths1, 0, 32, 1);
        update = true;
    }

    if (sector2 != previousSect2) {
        previousSect2 = sector2;
        int min2 = sector2/60000;
        int sec2 = (sector2 % 60000) / 1000;
        int tenths2 = (sector2 % 1000) / 100;
        drawSectorTime(min2, sec2, tenths2, 64, 32, 2);
        update = true;
    }

    if (sector3 != previousSect3) {
        previousSect3 = sector3;
        int min3 = sector3/60000;
        int sec3 = (sector3 % 60000) / 1000;
        int tenths3 = (sector3 % 1000) / 100;
        drawSectorTime(min3, sec3, tenths3, 0, 50, 3);
        update = true;
    }

    if (lastLap != previousLastLap) {
        previousLastLap = lastLap;
        int min4 = lastLap/60000;
        int sec4 = (lastLap % 60000) / 1000;
        int tenths4 = (lastLap % 1000) / 100;
        drawLastLapTime(min4, sec4, tenths4);
        update = true;
    }

    if (update) {
        display.display();
    }
}

// helper function to draw laptime in specifed slot
void drawSectorTime(int min, int sec, int tenths, int x, int y, int sector) {
    char str[30];
    sprintf(str, "S%d %d:%02d.%d", sector, min, sec, tenths);
    display.setColor(BLACK);
    display.fillRect(x, y, 64, 14);
    display.setColor(WHITE);
    display.drawString(x, y, str);
}

// helper for last lap time since it uses "L" instead of "S"
void drawLastLapTime(int min, int sec, int tenths) {
    char str[30];
    sprintf(str, "L    %d:%02d.%d", min, sec, tenths);
    display.setColor(BLACK);
    display.fillRect(64, 50, 64, 14);
    display.setColor(WHITE);
    display.drawString(64, 50, str);
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