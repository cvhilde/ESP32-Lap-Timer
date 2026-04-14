///===========================================================================
///
/// waypoints.cpp
///
/// This file contains waypoint storage and line-crossing logic used to track
/// sectors and lap progression from recent GPS coordinate updates.
///
///===========================================================================

#include <waypoints.h>
#include <array>
#include <math.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // Current Lap Number (1 index)
    uint8_t _lapNumber = 1;

    // Current active sector
    uint8_t _currentSector;

    // Array of all 3 sector waypoints to be used in lap timing
    std::array<WayPoints::WayPoint, 3> _trackWaypoints;

    // Array of the 2 most recent coordinate locations
    std::array<WayPoints::Coord, 2> _storedLocations;

    bool _firstLap = false;

    //------------------------------------------------------------------------
    // Calculates the physical orientation using a formula found on GeeksForGeeks
    int Orientation(WayPoints::Coord p1, WayPoints::Coord p2, WayPoints::Coord p3)
    {
        double val = (p2.lng - p1.lng) * (p3.lat - p1.lat) -
                     (p2.lat - p1.lat) * (p3.lng - p1.lng);

        if (val == 0.0)   // collinear
            return 0;
        else if (val > 0) // clockwise
            return 1;
        else              // counter clockwise
            return 2;

    }

    //------------------------------------------------------------------------
    // If the orientation between the first two points are different,
    // as well as the orientation between the second two points are different,
    // then the lines intersect.
    bool DoIntersect(WayPoints::Coord prev, WayPoints::Coord curr, WayPoints::WayPoint wp)
    {
        int o1 = Orientation(prev, curr, wp.p1);
        int o2 = Orientation(prev, curr, wp.p2);
        int o3 = Orientation(wp.p1, wp.p2, prev);
        int o4 = Orientation(wp.p1, wp.p2, curr);

        if (o1 != o2 && o3 != o4)
            return true;
        else
            return false;
        
    }
}

//----------------------------------------------------------------------------
// WayPoints Public namespace
//----------------------------------------------------------------------------
namespace WayPoints
{
    //------------------------------------------------------------------------
    void StoreCurrentLocation(double lat, double lng)
    {
        _storedLocations.back() = _storedLocations.front();
        _storedLocations.front().lat = lat;
        _storedLocations.front().lng = lng;
    }

    //------------------------------------------------------------------------
    double DistanceFeet(WayPoints::Coord p1, WayPoints::Coord p2)
    {
        double avgLatRadians = ((p1.lat + p2.lat) * 0.5) * DEG_TO_RADIANS;
        double x = (p2.lng - p1.lng) * DEG_TO_RADIANS * cos(avgLatRadians);
        double y = (p2.lat - p1.lat) * DEG_TO_RADIANS;
        return sqrt((x * x) + (y * y) * EARTH_RADIUS_FT);
    }
}


