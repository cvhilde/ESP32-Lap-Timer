#include <gps.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <globals.h>

SFE_UBLOX_GNSS gps;
HardwareSerial GPS(1);

// gps module initialization
// using hardcoded waypoint markers for testing
void initGPS() {
    GPS.begin(115200, SERIAL_8N1, 19, 20);
    delay(1000);
    
    gps.begin(GPS);
    delay(1000);

    for (int i = 0; i < 2; i++) {
        activeLocations[i].lat = 0;
        activeLocations[i].lng = 0;
    }

    //bool successOutput = gps.setUART1Output(COM_TYPE_UBX);
    bool successSet = gps.setAutoESFRAW(true);

    Serial.printf("Set: %d\n", successSet);
}

double getLatitude() {
    return gps.getLatitude() / 10000000.0;
}

double getLongitude() {
    return gps.getLongitude() / 10000000.0;
}

int getSatCount() {
    return gps.getSIV();
}

double getSpeed() {
    return gps.getGroundSpeed() * 0.00223694;
}

double getAltitude() {
    double alt = gps.getAltitude();
    double sep = gps.getGeoidSeparation();
    return (alt - sep) / 304.8;
}