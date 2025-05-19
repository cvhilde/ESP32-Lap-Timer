#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// #define gpsRX = 20
// #define gpsTX = 19

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
TinyGPSPlus gps;
HardwareSerial GPS(1);
TinyGPSCustom satsInView(gps, "GPGSV", 3); // GPGSV sentence, field 3

TinyGPSCustom satNumber[4];

#define Read_VBAT_Voltage 1
#define ADC_CTRL 37
#define ADC_READ_STABILIZE 10

float getVoltage();
uint32_t getSatCount();

void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

void VextOFF() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, HIGH);
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    VextON();
    delay(100);
    pinMode(1, INPUT);
    analogReadResolution(12);

    GPS.begin(9600, SERIAL_8N1, 19, 20);
    delay(1000);
    GPS.print("$PMTK220,100*2F\r\n"); // Enable 10 Hz
    GPS.print("$PMTK300,100,0,0,0,0*2C\r\n"); //  enabling NMEA output rate of 10hz
    //GPS.print("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");

    for (int i = 0; i < 4; ++i) {
        satNumber[i].begin(gps, "GPGSV", 4 + 4*i);
        // You can do elevation/azimuth/snr the same way
}


    display.init();
    display.setFont(ArialMT_Plain_10);
}

void drawScreen(double lata, double longa) {
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
    float vcc = getVoltage();
    uint32_t sats = getSatCount();
    sprintf(str2, "%.2fV   Sats: %d", vcc, sats);
    display.drawString(0, y, str2);

    //display.setTextAlignment(TEXT_ALIGN_RIGHT);
    // sprintf(str3, "Sats: %d", gps.satellites.value());
    // display.drawString(15, y, str3);

    display.display();
}

double getLatitude() {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
}

double getLongitude() {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
}

float getVoltage() {
    pinMode(ADC_CTRL,OUTPUT);
    digitalWrite(ADC_CTRL, LOW);
    delay(ADC_READ_STABILIZE); // let GPIO stabilize
    int analogValue = analogRead(Read_VBAT_Voltage);
    float voltage = analogValue;
    return voltage;
}

uint32_t getSatCount() {
    int visible = atoi(satsInView.value());
    return visible;
}


void loop() {
    while (GPS.available()) {
        gps.encode(GPS.read());
    }

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 500) {
        lastUpdate = millis();
        if (gps.location.isValid()) {
            double lat = getLatitude();
            double lng = getLongitude();
            uint32_t sats = getSatCount();
            float vcc = getVoltage();

            Serial.printf("lat: %.5lf | long: %.5lf | sats: %d | vcc %.2f\n", lat, lng, sats, vcc);
            display.clear();
            drawScreen(lat, lng);
        } else {
            uint32_t sats = getSatCount();
            Serial.printf("sats: %d\n", sats);
            char satInfo[30];
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(display.width()/2, display.height()/2 - 8, "Waiting for GPS...");
            sprintf(satInfo, "Sats: %d", getSatCount());
            display.drawString(display.width()/2, display.height()/2 - 16, satInfo);
            display.display();
        }
    }
}