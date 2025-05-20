#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// #define gpsRX = 20
// #define gpsTX = 19

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
TinyGPSPlus gps;
HardwareSerial GPS(1);
TinyGPSCustom satsInView(gps, "GPGSV", 3); // GPGSV sentence, field 3

TinyGPSCustom satNumber[4];

#define VBAT_Read 1
#define ADC_CTRL 37
uint16_t samples[8];
int avgIndex = 0;

typedef struct Point {
    double lat;
    double lng;
    int isSector;
    int sectorCount;
    int isActive;
} Point;

Point activeLocations[2]; // 0 is current, 1 is previous
Point trackWaypoints[6]; // first 2 are start/finish line, next 4 are the sector waypoints
int currSector = 0;
#define EARTH_RADIUS 6371000.0 // in meters

float readBattVoltage();
uint32_t getSatCount();
double getLongitude();
double getLatitude();
void drawScreen(double lata, double longa);
void VextOFF();
void VextON();
int orientation(Point p1, Point p2, Point p3);
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
void storeCurrLocation(double lat, double lng);


void VextON() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, LOW);
}

void VextOFF() {
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext, HIGH);
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

void drawLaptime(double laptime) {
    display.clear();
    char str[20];
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(display.width()/2, 0, "Passed waypoint " + String(currSector));
    sprintf(str, "%.2f seconds", laptime);
    display.drawString(display.width()/2, 20, str);
    display.display();
    delay(5000);
}

double getLatitude() {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
}

double getLongitude() {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
}

uint32_t getSatCount() {
    int visible = atoi(satsInView.value());
    return visible;
}

float readBattVoltage() {
    digitalWrite(ADC_CTRL, HIGH);
    samples[avgIndex++] = analogRead(VBAT_Read);
    digitalWrite(ADC_CTRL, LOW);

    if (avgIndex >= 8) {
        avgIndex = 0;
    }
    uint32_t total = 0;
    for (int i = 0; i < 8; i++) {
        total += samples[i];
    }
    float avgRaw = total / 8.0;
    return avgRaw * (3.3 / 4095.0) * 5.215384 * 1.073655914; // Just 5.215384 for 2nd board
}

int orientation(Point a, Point b, Point c) {
    double val = (b.lng - a.lng) * (c.lat - b.lat) - (b.lat - a.lat) * (c.lng - b.lng);
    if (val == 0.0) {
        // collinear
        return 0;
    } else if (val > 0) {
        // clockwise
        return 1;
    } else {
        // counter clockwise
        return 2;
    }
}

bool doIntersect(Point p1, Point q1, Point p2, Point q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    return (o1 != o2 && o3 != o4);
}

void storeCurrLocation(double lat, double lng) {
    activeLocations[1] = activeLocations[0];
    activeLocations[0].lat = lat;
    activeLocations[0].lng = lng;
    activeLocations[0].isActive = 1;
}

// note: extremely accurate but resource intensive. consider changing it later
double distanceCalc(Point p1, Point p2) {
    double dLat = radians(p1.lat - p2.lat);
    double dLng = radians(p1.lng - p2.lng);
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(p1.lat)) * cos(radians(p2.lat)) * sin(dLng / 2) * sin(dLng / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c * 3.28084;
}


void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    VextON();
    delay(100);
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, LOW);
    analogReadResolution(12);

    GPS.begin(9600, SERIAL_8N1, 19, 20);
    delay(1000);
    GPS.print("$PMTK220,100*2F\r\n"); // Enable 10 Hz
    GPS.print("$PMTK300,100,0,0,0,0*2C\r\n"); //  enabling NMEA output rate of 10hz

    for (int i = 0; i < 4; ++i) {
        satNumber[i].begin(gps, "GPGSV", 4 + 4*i);
    }

    display.init();
    display.setFont(ArialMT_Plain_10);

    // init activelocations
    for (int i = 0; i < 2; i++) {
        activeLocations[i].lat = 0;
        activeLocations[i].lng = 0;
        activeLocations[i].isSector = 0;
        activeLocations[i].sectorCount = 0;
        activeLocations[i].isActive = 0;
    }

    // // init tracked waypoints
    // for (int i = 0; i < 6; i++) {
    //     trackWaypoints[i].lat = 0;
    //     trackWaypoints[i].lng = 0;
    //     trackWaypoints[i].isSector = 0;
    //     trackWaypoints[i].sectorCount = 0;
    //     trackWaypoints[i].isActive = 0;
    // }

    trackWaypoints[0].lat = 28.61339545;
    trackWaypoints[0].lng = -81.17926727;
    trackWaypoints[0].isActive = 1;
    trackWaypoints[0].isSector = 0;
    trackWaypoints[0].sectorCount = 0;

    trackWaypoints[1].lat = 28.61346579;
    trackWaypoints[1].lng = -81.17926906;
    trackWaypoints[1].isActive = 1;
    trackWaypoints[1].isSector = 0;
    trackWaypoints[1].sectorCount = 0;

    trackWaypoints[2].lat = 28.61392242;
    trackWaypoints[2].lng = -81.17897079;
    trackWaypoints[2].isActive = 1;
    trackWaypoints[2].isSector = 1;
    trackWaypoints[2].sectorCount = 1;

    trackWaypoints[3].lat = 28.61392980;
    trackWaypoints[3].lng = -81.17906224;
    trackWaypoints[3].isActive = 1;
    trackWaypoints[3].isSector = 1;
    trackWaypoints[3].sectorCount = 1;

    trackWaypoints[4].lat = 28.61364765;
    trackWaypoints[4].lng = -81.17963668;
    trackWaypoints[4].isActive = 1;
    trackWaypoints[4].isSector = 1;
    trackWaypoints[4].sectorCount = 2;

    trackWaypoints[5].lat = 28.61365188;
    trackWaypoints[5].lng = -81.17954403;
    trackWaypoints[5].isActive = 1;
    trackWaypoints[5].isSector = 1;
    trackWaypoints[5].sectorCount = 2;
}

void loop() {
    while (GPS.available()) {
        gps.encode(GPS.read());
    }

    static unsigned long lastUpdate = 0;
    static unsigned long lastCrossTime = 0;
    if (millis() - lastUpdate >= 500) {
        lastUpdate = millis();
        if (gps.location.isValid()) {
            double lat = getLatitude();
            double lng = getLongitude();
            uint32_t sats = getSatCount();
            float vcc = readBattVoltage();
            storeCurrLocation(lat, lng);

            double distance = distanceCalc(activeLocations[0], trackWaypoints[currSector*2]);
            //drawDistance(distance);
            for (int i = 0; i < 6; i++) {
                if (trackWaypoints[i].isActive) {
                    if (doIntersect(trackWaypoints[i], trackWaypoints[(i+1)%6], activeLocations[1], activeLocations[0])) {
                        if (millis() - lastCrossTime > 15000) {
                            lastCrossTime = millis();
                            drawLaptime(50.69); // placeholder for now until i add lap/sector timing logic
                            currSector++;
                            if (currSector > 2) {
                                currSector = 0;
                            }
                        }
                    }
                }
            }

            Serial.printf("lat: %.5lf | long: %.5lf | sats: %d | vcc %.2f\n", lat, lng, sats, vcc);
            drawScreen(lat, lng, distance);
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