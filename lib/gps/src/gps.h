#ifndef GPS_H
#define GPS_H

#include <stdint.h>

namespace GPS
{
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

    struct FixData
    {
        double latitude;  // degrees
        double longitude; // degrees
        double speed;     // mph
        double altitude;  // feet
        uint8_t fixType;
        uint8_t satelliteCount;
        GPSTimeData dateTime;

        // If false, fixType and satellite count are still valid,
        // but positional and speed/altitude data are not.
        bool valid;

        FixData() :
            latitude(0.0),
            longitude(0.0),
            speed(0.0),
            altitude(0.0),
            fixType(0),
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
