#include <display.h>
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <globals.h>
#include <gps.h>

uint16_t samples[20];
int avgIndex = 0;

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
}

void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

void VextOFF() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, HIGH);
}

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
    uint32_t sats = getSatCount();
    sprintf(str2, "%.2fV   Dist: %.2lf", vcc, distance);
    display.drawString(0, y, str2);
    display.display();
}

void drawDistance(double distance) {
    display.clear();
    char str[20];
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.width()/2, 0, "Distance to Waypoint " + String(currSector));
    sprintf(str, "%.2f Feet", distance);
    display.drawString(display.width()/2, 20, str);
    display.display();
}

void drawLaptime(unsigned long lapTime, unsigned long sector1, unsigned long sector2, unsigned long sector3, unsigned long lastLap) {
    unsigned int min = lapTime/60000;
    unsigned int sec = (lapTime % 60000) / 1000;
    unsigned int tenths = (lapTime % 1000) / 100;

    unsigned int min1 = sector1/60000;
    unsigned int sec1 = (sector1 % 60000) / 1000;
    unsigned int tenths1 = (sector1 % 1000) / 100;

    unsigned int min2 = sector2/60000;
    unsigned int sec2 = (sector2 % 60000) / 1000;
    unsigned int tenths2 = (sector2 % 1000) / 100;

    unsigned int min3 = sector3/60000;
    unsigned int sec3 = (sector3 % 60000) / 1000;
    unsigned int tenths3 = (sector3 % 1000) / 100;

    unsigned int min4 = lastLap/60000;
    unsigned int sec4 = (lastLap % 60000) / 1000;
    unsigned int tenths4 = (lastLap % 1000) / 100;

    char curr[30];
    char sect1[30];
    char sect2[30];
    char sect3[30];
    char last[30];
    sprintf(curr, "%u:%02u.%u", min, sec, tenths);
    sprintf(sect1, "S1 %u:%02u.%u", min1, sec1, tenths1);
    sprintf(sect2, "S2 %u:%02u.%u", min2, sec2, tenths2);
    sprintf(sect3, "S3 %u:%02u.%u", min3, sec3, tenths3);
    sprintf(last, "L %u:%02u.%u", min4, sec4, tenths4);
    display.clear();
    display.drawString(40, 5, curr);
    display.drawString(0, 32, sect1);
    display.drawString(64, 32, sect2);
    display.drawString(0, 50, sect3);
    display.drawString(64, 50, last);
    display.display();
}

