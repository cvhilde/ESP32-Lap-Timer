#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <waypoints.h>

namespace GPS
{
    // Possible fix types that this gps can return
    enum class FixType
    {
        NO_FIX              = 0,
        DEAD_RECKONING      = 1,
        TWO_D               = 2,
        THREE_D             = 3,
        GNSS_DEAD_RECKONING = 4,
        TIME_ONLY           = 5
    };

    // Contains all data relating to the date and time
    struct GPSTimeData
    {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;   // uses a temporary hardcoded value to EDT
        uint8_t minute;
        uint8_t second;
        bool valid;

        GPSTimeData() :
            year(0),
            month(0),
            day(0),
            hour(0),
            minute(0),
            second(0),
            valid(false)
        {}
    };

    // Collection of all the data the gps returns
    struct FixData
    {
        WayPoints::Coord coord;
        double speed;     // mph
        double altitude;  // feet
        FixType fixType;
        uint8_t satelliteCount;
        GPSTimeData dateTime;

        // If false, fixType and satellite count are still valid,
        // but positional and speed/altitude data are not.
        bool valid;

        FixData() :
            speed(0.0),
            altitude(0.0),
            fixType(FixType::NO_FIX),
            satelliteCount(0),
            dateTime(),
            valid(false)
        {}
    };

    // Multiplication constant for converting from mm/s to mph.
    constexpr double MM_S_TO_MPH = 0.00223694;

    // Multiplication constant for converting from mm to feet.
    constexpr double MM_TO_FEET = 0.00328084;

    // Multiplication constant for converting from ublox lat/long
    // to a normalized value.
    constexpr double LAT_LONG_TO_DEGREES = 0.0000001;

    // Initialize the U-BLOX NEO M9N GPS module.
    bool InitializeUBLOX();

    // Grab the latest gps data from the module and cache it.
    // WARNING: This function performs a blocking action. It will pause until
    // new data is acquired. So if the frequency is set to 10Hz, and you try
    // to call this function at a rate of 20Hz, it will still result in a
    // 10Hz function call rate.
    void UpdateUBLOX();

    // Return the most recently acquired position/speed/altitude data.
    // This function does not request a new reading from the module.
    // Call UpdateUBLOX() first to refresh the cached GPS data.
    // fixType data:
    // 0 - No fix
    // 1 - Dead reckoning only (not available with NEO-M9N)
    // 2 - 2D
    // 3 - 3D
    // 4 - GNSS + Dead Reckoning combined
    // 5 - Time only fix
    // With the NEO-M9N, the best fix type will be 3.
    const FixData& GetFixData();
};

#endif
