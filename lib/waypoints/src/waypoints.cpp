#include <waypoints.h>
#include <math.h>
#include <time.h>
#include <TimeLib.h>
#include <globals.h>

// calculates the physical orientation
// uses a formula from GeeksforGeeks
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

// if the orientation between the first two points are different,
// as well as the orientation between the second two points are different,
// then the lines intersect.
bool doIntersect(coord prev, coord curr, wayPoint wp) {
    int o1 = orientation(prev, curr, wp.p1);
    int o2 = orientation(prev, curr, wp.p2);
    int o3 = orientation(wp.p1, wp.p2, prev);
    int o4 = orientation(wp.p1, wp.p2, curr);

    if (o1 != o2 && o3 != o4) {
        return true;
    } else {
        return false;
    }
}

// helper function to update current location waypoint
void storeCurrLocation(double lat, double lng) {
    activeLocations[1] = activeLocations[0];
    activeLocations[0].lat = lat;
    activeLocations[0].lng = lng;
}

// distance calculation using the haversine formula
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

// for the distance calculation, so it can use the midpoint
coord midPoint(wayPoint wp) {
    coord mid;
    mid.lat = (wp.p1.lat + wp.p2.lat) / 2.0;
    mid.lng = (wp.p1.lng + wp.p2.lng) / 2.0;
    return mid;
}