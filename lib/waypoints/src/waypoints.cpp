///===========================================================================
///
/// waypoints.cpp
///
/// This file contains waypoint storage and line-crossing logic used to track
/// sectors and lap progression from recent GPS coordinate updates.
///
///===========================================================================

#include <waypoints.h>
#include <math.h>
#include <gps.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // Needed information for storing the total session distance
    struct SessionDistance
    {
        double           sessionDistanceFt;
        bool             haveLastDistancePoint;
        WayPoints::Coord lastCoord;

        SessionDistance() :
            sessionDistanceFt(0.0),
            haveLastDistancePoint(false)
        {}
    };

    // Current Lap Number (1 index)
    uint8_t _lapNumber = 1;

    // Current active sector
    uint8_t _currentSector;

    // Array of all 3 sector waypoints to be used in lap timing
    WayPoints::TrackedWaypoints _trackWaypoints;

    // Array of the 2 most recent coordinate locations
    WayPoints::RecentLocations _storedLocations;

    // Boolean for the first lap flag
    bool _firstLap = false;

    // Collection of session distance information
    SessionDistance _sessionDisInfo;

    // Minimum distance a segment must be for the distance to be added to
    // the total distance for the session
    constexpr double MIN_DISTANCE_FT = 1.0;

    // Minimum speed for the distance to be added to the total distance
    // for the session
    constexpr double MIN_SPEED_MPH = 0.5;

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

    //------------------------------------------------------------------------
    // Calculates the distance between two coordinate points in feet.
    double DistanceFeet(const WayPoints::Coord& p1, const WayPoints::Coord& p2)
    {
        double avgLatRadians = ((p1.lat + p2.lat) * 0.5) * WayPoints::DEG_TO_RADIANS;
        double x = (p2.lng - p1.lng) * WayPoints::DEG_TO_RADIANS * cos(avgLatRadians);
        double y = (p2.lat - p1.lat) * WayPoints::DEG_TO_RADIANS;
        return sqrt((x * x) + (y * y) * WayPoints::EARTH_RADIUS_FT);
    }
}

//----------------------------------------------------------------------------
// WayPoints Public namespace
//----------------------------------------------------------------------------
namespace WayPoints
{
    //------------------------------------------------------------------------
    void StoreCurrentLocation(WayPoints::Coord& point)
    {
        // back() is the previous location. front() is the current.
        // So the front becomes the back, and the current becomes the front.
        _storedLocations.back()  = _storedLocations.front();
        _storedLocations.front() = point;
    }

    //------------------------------------------------------------------------
    void ResetSessionDistance()
    {
        _sessionDisInfo.sessionDistanceFt     = 0.0;
        _sessionDisInfo.lastCoord.lat         = 0.0;
        _sessionDisInfo.lastCoord.lng         = 0.0;
        _sessionDisInfo.haveLastDistancePoint = false;
    }

    //------------------------------------------------------------------------
    void UpdateSessionDistance(const Coord& coord, const double speed)
    {
        if (!_sessionDisInfo.haveLastDistancePoint)
        {
            _sessionDisInfo.lastCoord = coord;
            _sessionDisInfo.haveLastDistancePoint = true;
            return;
        }

        double segmentFt = DistanceFeet(_sessionDisInfo.lastCoord, coord);
        _sessionDisInfo.lastCoord = coord;

        if (segmentFt  < MIN_DISTANCE_FT ||
            speed < MIN_SPEED_MPH)
        {
            return;
        }

        _sessionDisInfo.sessionDistanceFt += segmentFt;
    }

    bool WaypointCrossed(const unsigned currentSector)
    {
        const WayPoint& currentWaypoint(_trackWaypoints.at(currentSector));

        return(   currentWaypoint.isActive
               && DoIntersect(_storedLocations.back(), _storedLocations.front(), currentWaypoint));
    }

    const SessionDistance& GetSessionDistance()
    {
        SessionDistance distance;

        distance.distanceFeet = _sessionDisInfo.sessionDistanceFt;
        distance.distanceMile = _sessionDisInfo.sessionDistanceFt / FEET_PER_MILE;

        return distance;
    }

    const TrackedWaypoints& GetTrackWaypoints()
    {
        return _trackWaypoints;
    }

    void SetTrackWaypoints(const TrackedWaypoints& waypoints)
    {
        _trackWaypoints = waypoints;
    }
}


