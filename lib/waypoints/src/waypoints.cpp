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
