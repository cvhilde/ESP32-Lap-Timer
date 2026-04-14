#ifndef WAYPOINTS_H
#define WAYPOINTS_H

namespace WayPoints
{
    // Data structure for a coordinate point on a 2D plane.
    struct Coord
    {
        double lat;
        double lng;
    };

    // Data structure for a sector crossing waypoint. It uses two
    // coordinate points to create a line that acts as a crossing.
    struct WayPoint
    {
        Coord p1;
        Coord p2;
        bool isActive;

        WayPoint() :
            isActive(false)
        {}
    };

    // Earth radius in feet.
    constexpr double EARTH_RADIUS_FT = 20902230.0;

    // Constant for converting degrees to radians.
    constexpr double DEG_TO_RADIANS = 0.017453292519943295;

    // Function that will store the current gps location, and shift
    // back the previous location.
    void StoreCurrentLocation(double lat, double lng);

    // Calculates the distance between two coordinate points in feet.
    double DistanceFeet(WayPoints::Coord p1, WayPoints::Coord p2);

};

#endif
