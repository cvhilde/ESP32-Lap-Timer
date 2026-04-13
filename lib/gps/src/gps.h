#ifndef GPS_H
#define GPS_H

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <globals.h>

class GPS {
    public:
        struct Time {
            uint16_t year;
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t minute;
            uint8_t second;
            bool valid;

            Time() :
                valid(false)
            {}
        };

        static const double MM_S_TO_MPH;

        static const double MM_TO_FEET;

        static const double LAT_LONG_TO_DEGREES;

        // Initialize the U-BLOX NEO M9N GPS module.
        static void InitializeGPS();

        // Get the latitude returned in degrees.
        static double Latitude();

        // Get the longitude returned in degrees.
        static double Longitude();

        // Get the number of satellites in view of the GPS antenna.
        static int SatelliteCount();

        // Get the ground speed measured in MPH.
        static double Speed();

        // Get the altitude above sea level measured in feet.
        static double GPSAltitude();

        // Get the fix type.
        // 0 - No fix
        // 1 - Dead reckoning only (not available with NEO-M9N)
        // 2 - 2D
        // 3 - 3D
        // 4 - GNSS + Dead Reckoning combined
        // 5 - Time only fix
        // With the NEO-M9N, the best fix type will be 3.
        static uint8_t FixType();

        // Grab the latest gps data from the module.
        // WARNING: This function performs a blocking action. It will pause until
        // new data is acquired. So if the frequency is set to 10Hz, and you try
        // to call this function at a rate of 20Hz, it will still result in a
        // 10Hz function call rate.
        static void UpdateUBLOX();

        // Get the date and time from the GPS, returned with the GPS::Time struct.
        static Time GPSTime();

    private:
        // Private declaration of the gps object used to interface with the
        // actual gps module.
        static SFE_UBLOX_GNSS _gps;

        // Private declaration of the hardware serial used by the ESP32 to
        // talk to the gps module.
        static HardwareSerial _GPSHardwareSerial;
};

#endif
