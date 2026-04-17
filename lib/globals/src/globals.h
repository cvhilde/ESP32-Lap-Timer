#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>


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

extern String currentTimestamp;
extern bool sessionActive;

extern int lapNumber;


#endif
