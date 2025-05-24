#include <gps.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <globals.h>

SFE_UBLOX_GNSS gps;
HardwareSerial GPS(1);

void initGPS() {
    GPS.begin(115200, SERIAL_8N1, 19, 20);
    delay(1000);
    if (!gps.begin(GPS)) {
        Serial.println("GNSS init failed");
        while (1);
    }


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

double getLatitude() {
    return gps.getLatitude() / 10000000.0;
}

double getLongitude() {
    return gps.getLongitude() / 10000000.0;
}

uint32_t getSatCount() {
    return gps.getSIV();
}

double getSpeed() {
    return gps.getGroundSpeed() * 0.00223694;
}