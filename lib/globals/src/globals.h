#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

#define VBAT_Read 1
#define ADC_CTRL 37
#define BUTTON_PIN 46
#define GPS_Rx 19
#define GPS_Tx 20
#define LED_PIN 26

typedef struct coord {
    double lat;
    double lng;
} coord;

typedef struct wayPoint {
    coord p1;
    coord p2;
    int isActive;
} wayPoint;

extern coord activeLocations[2]; // 0 is current, 1 is previous
extern wayPoint trackWaypoints[3]; // first is start/finish line, next 2 are the sector waypoints
extern uint8_t currSector;

const float EARTH_RADIUS_FT = 20902230.0f;
const float DEG_TO_RADIANS = 0.017453292519943295f;

extern String currentTimestamp;
extern bool sessionActive;

extern int lapNumber;


#endif