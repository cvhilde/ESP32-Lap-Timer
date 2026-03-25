#ifndef GPS_H
#define GPS_H

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>

extern SFE_UBLOX_GNSS gps;
extern HardwareSerial GPS;

struct AirborneData {
    bool valid;
    float ax_g;
    float ay_g;
    float az_g;
    float mag_g;
    bool airborne;

    AirborneData() :
        valid(false),
        ax_g(-1.0f),
        ay_g(-1.0f),
        az_g(-1.0f),
        mag_g(-1.0f),
        airborne(false)
    {}
};

void initGPS();
double getLatitude();
double getLongitude();
int getSatCount();
double getSpeed();
double getAltitude();
bool getAirborneState();

#endif