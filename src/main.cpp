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
TinyGPSCustom satsInView(gps, "GPGSV", 3);

TinyGPSCustom satNumber[4];

#define VBAT_Read 1
#define ADC_CTRL 37
uint16_t samples[500];
int avgIndex = 0;

typedef struct coord {
    double lat;
    double lng;
} coord;

typedef struct wayPoint {
    coord p1;
    coord p2;
    int isActive;
} wayPoint;

coord activeLocations[2]; // 0 is current, 1 is previous
wayPoint trackWaypoints[3]; // first is start/finish line, next 2 are the sector waypoints
int currSector = 0;

const float EARTH_RADIUS_FT = 20902230.0f;
const float DEG_TO_RADIANS = 0.017453292519943295f;

float readBattVoltage();
uint32_t getSatCount();
double getLongitude();
double getLatitude();
void drawScreen(double lata, double longa, double distance);
void VextOFF();
void VextON();
int orientation(coord p1, coord p2, coord p3);
bool doIntersect(coord prev, coord curr, wayPoint wp);
void storeCurrLocation(double lat, double lng);
coord midPoint(wayPoint wp);


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

    if (avgIndex >= 500) {
        avgIndex = 0;
    }
    uint32_t total = 0;
    for (int i = 0; i < 500; i++) {
        total += samples[i];
    }
    float avgRaw = total / 500.0;
    return avgRaw * (3.3 / 4095.0) * 5.215384 * 1.073655914; // Just 5.215384 for 2nd board
}

int orientation(coord p1, coord p2, coord p3) {
    double val = (p2.lng - p1.lng) * (p3.lat - p1.lat) - (p2.lat - p1.lat) * (p3.lng - p1.lng);
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

bool doIntersect(coord prev, coord curr, wayPoint wp) {
    int o1 = orientation(prev, curr, wp.p1);
    int o2 = orientation(prev, curr, wp.p2);
    int o3 = orientation(wp.p1, wp.p2, prev);
    int o4 = orientation(wp.p1, wp.p2, curr);

    if (o1 != o2 && o3 != o4) return true;

    if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0) {
        if (fmin(prev.lng, curr.lng) <= fmax(wp.p1.lng, wp.p2.lng) &&
            fmin(wp.p1.lng, wp.p2.lng) <= fmax(prev.lng, curr.lng) &&
            fmin(prev.lat, curr.lat) <= fmax(wp.p1.lat, wp.p2.lat) &&
            fmin(wp.p1.lat, wp.p2.lat) <= fmax(prev.lat, curr.lat)) {
            return true;
        }
    }
    return false;
}

void storeCurrLocation(double lat, double lng) {
    activeLocations[1] = activeLocations[0];
    activeLocations[0].lat = lat;
    activeLocations[0].lng = lng;
}

double distanceCalc(coord p, wayPoint wp) {
    coord mid = midPoint(wp);
    float dLat = (mid.lat - p.lat) * DEG_TO_RADIANS;
    float dLng = (mid.lng - p.lng) * DEG_TO_RADIANS;
    p.lat *= DEG_TO_RADIANS;
    mid.lat *= DEG_TO_RADIANS;

    float a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) + 
              sinf(dLng / 2.0f) * sinf(dLng / 2.0f) * cosf(p.lat) * cosf(mid.lat);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return EARTH_RADIUS_FT * c;
}

coord midPoint(wayPoint wp) {
    coord mid;
    mid.lat = (wp.p1.lat + wp.p2.lat) / 2.0;
    mid.lng = (wp.p1.lng + wp.p2.lng) / 2.0;
    Serial.printf("%.7f lat | %.7f lng\n", mid.lat, mid.lng);
    return mid;
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

    GPS.begin(115200, SERIAL_8N1, 19, 20);
    delay(1000);

    for (int i = 0; i < 4; ++i) {
        satNumber[i].begin(gps, "GPGSV", 4 + 4*i);
    }

    display.init();
    display.setFont(ArialMT_Plain_10);

    for (int i = 0; i < 2; i++) {
        activeLocations[i].lat = 0;
        activeLocations[i].lng = 0;
    }

    trackWaypoints[0].p1.lat = 28.613391;
    trackWaypoints[0].p1.lng = -81.179126;
    trackWaypoints[0].p2.lat = 28.613467;
    trackWaypoints[0].p2.lng = -81.179126;
    trackWaypoints[0].isActive = 1;

    trackWaypoints[1].p1.lat = 28.61392242;
    trackWaypoints[1].p1.lng = -81.17897079;
    trackWaypoints[1].p2.lat = 28.61392980;
    trackWaypoints[1].p2.lng = -81.17906224;
    trackWaypoints[1].isActive = 1;

    trackWaypoints[2].p1.lat = 28.61364765;
    trackWaypoints[2].p1.lng = -81.17963668;
    trackWaypoints[2].p2.lat = 28.61365188;
    trackWaypoints[2].p2.lng = -81.17954403;
    trackWaypoints[2].isActive = 1;
}

void loop() {
    while (GPS.available()) {
        gps.encode(GPS.read());
    }

    static unsigned long lastUpdate = 0;
    static unsigned long lastCrossTime = 0;
    static unsigned long lastSectorTime = 0;
    static unsigned long lastLapTime = 0;
    if (millis() - lastUpdate >= 100) {
        lastUpdate = millis();
        if (gps.location.isValid()) {
            double lat = getLatitude();
            double lng = getLongitude();
            uint32_t sats = getSatCount();
            float vcc = readBattVoltage();
            storeCurrLocation(lat, lng);

            double distance = distanceCalc(activeLocations[0], trackWaypoints[currSector]);
            if (trackWaypoints[currSector].isActive) {
                if (doIntersect(activeLocations[1], activeLocations[0], trackWaypoints[currSector])) {
                    if (millis() - lastCrossTime > 15000) {
                        lastCrossTime = millis();
                        if (currSector == 0) {
                            drawLaptime((millis() - lastLapTime) / 1000.0);
                            lastLapTime = millis();
                            lastSectorTime = millis();
                        } else {
                            drawLaptime((millis() - lastSectorTime) / 1000.0);
                            lastSectorTime = millis();
                        }
                        currSector++;
                        if (currSector > 2) {
                            currSector = 0;
                        }
                    }
                }
            }

            Serial.printf("lat: %.5lf | long: %.5lf | sats: %d | dist %.2f\n\n", lat, lng, sats, distance);
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