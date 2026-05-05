#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <array>

namespace WayPoints
{
    // Data structure for a coordinate point on a 2D plane.
    struct Coord
    {
        double lat;
        double lng;

        Coord() :
            lat(0.0),
            lng(0.0)
        {}
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

    struct SessionDistance
    {
        double distanceFeet;
        double distanceMile;

        SessionDistance() :
            distanceFeet(0.0),
            distanceMile(0.0)
        {}
    };

    typedef std::array<WayPoints::WayPoint, 3> TrackedWaypoints;

    typedef std::array<WayPoints::Coord, 2> RecentLocations;

    // Earth radius in feet.
    constexpr double EARTH_RADIUS_FT = 20902230.0;

    // Constant for converting degrees to radians.
    constexpr double DEG_TO_RADIANS = 0.017453292519943295;

    // Consts for convering from/to miles/feet
    constexpr double FEET_PER_MILE = 5280.0;

    // Function that will store the current gps location, and shift
    // back the previous location.
    void StoreCurrentLocation(const WayPoints::Coord& point);

    // Resets the recent location history used for line crossing checks.
    void ResetRecentLocations();

    // Reset the session distance counter for a new session.
    void ResetSessionDistance();

    // Updates the session distance counter.
    void UpdateSessionDistance(const Coord& coord, const double speed);

    // Returns a boolean determining the passed in sector was crossed
    // within the past frame.
    bool WaypointCrossed(const unsigned currentSector);

    // Gets the session distance.
    const SessionDistance GetSessionDistance();

    // Gets a reference to the tracked waypoints.
    const TrackedWaypoints& GetTrackWaypoints();

    // Sets the tracked waypoints.
    void SetTrackWaypoints(const TrackedWaypoints& waypoints);
};

#endif
