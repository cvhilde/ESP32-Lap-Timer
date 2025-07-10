#ifndef GPS_H
#define GPS_H

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>

extern SFE_UBLOX_GNSS gps;
extern HardwareSerial GPS;

void initGPS();
double getLatitude();
double getLongitude();
int getSatCount();
double getSpeed();

#endif