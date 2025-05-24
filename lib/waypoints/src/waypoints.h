#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <globals.h>


int orientation(coord p1, coord p2, coord p3);
bool doIntersect(coord prev, coord curr, wayPoint wp);
void storeCurrLocation(double lat, double lng);
coord midPoint(wayPoint wp);
double distanceCalc(coord p, wayPoint wp);


#endif