///===========================================================================
///
/// This file is specifically for interfacing with the U-BLOX NEO M9N GPS
/// module and all related logic to it. This is the only file that should be
/// directly calling functions from the gps declaration. Any other file that
/// needs data from the GPS should have a helper function within this file.
///
///===========================================================================

#include <gps.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>

//----------------------------------------------------------------------------
// Private declarations
//----------------------------------------------------------------------------
namespace
{
    // Private declaration of the gps object used to interface with the
    // actual gps module.
    SFE_UBLOX_GNSS _gps;

    // Private declaration of the hardware serial used by the ESP32 to
    // talk to the gps module.
    HardwareSerial _GPSHardwareSerial(1);

    // Cached gps data
    GPS::FixData _fixData;

    // Physical gps data pins connected to the ESP32.
    constexpr uint8_t GPS_Rx = 19;
    constexpr uint8_t GPS_Tx = 20;

    // Initial setup flag. If false, prevent other code from running.
    bool initialized = false;
}

//----------------------------------------------------------------------------
// GPS Public namespace
//----------------------------------------------------------------------------
namespace GPS
{

    //------------------------------------------------------------------------
    bool InitializeUBLOX()
    {    
        _GPSHardwareSerial.begin(115200, SERIAL_8N1, GPS_Rx, GPS_Tx);
        delay(1000);
        
        initialized = _gps.begin(_GPSHardwareSerial);
        delay(1000);

        return initialized;
    }

    //------------------------------------------------------------------------
    void UpdateUBLOX()
    {
        _fixData = FixData{};

        if (initialized)
        {
            _gps.checkUblox();

            uint8_t fix = _gps.getFixType();

            if (fix >= 2 && fix <= 4)
            {
                _fixData.latitude  = _gps.getLatitude()    * LAT_LONG_TO_DEGREES;
                _fixData.longitude = _gps.getLongitude()   * LAT_LONG_TO_DEGREES;
                _fixData.speed     = _gps.getGroundSpeed() * MM_S_TO_MPH;
                _fixData.altitude  = _gps.getAltitudeMSL() * MM_TO_FEET;
                
                _fixData.dateTime.year   = _gps.getYear();
                _fixData.dateTime.month  = _gps.getMonth();
                _fixData.dateTime.day    = _gps.getDay();
                _fixData.dateTime.hour   = ((_gps.getHour() - 4) + 24) % 24;
                _fixData.dateTime.minute = _gps.getMinute();
                _fixData.dateTime.second = _gps.getSecond();

                _fixData.dateTime.valid = true;
                _fixData.valid          = true;
            }
            else if (fix == 5)
            {
                _fixData.dateTime.year   = _gps.getYear();
                _fixData.dateTime.month  = _gps.getMonth();
                _fixData.dateTime.day    = _gps.getDay();
                _fixData.dateTime.hour   = ((_gps.getHour() - 4) + 24) % 24;
                _fixData.dateTime.minute = _gps.getMinute();
                _fixData.dateTime.second = _gps.getSecond();

                _fixData.dateTime.valid = true;
            }

            _fixData.fixType        = fix;
            _fixData.satelliteCount = _gps.getSIV();
        }
    }

    //------------------------------------------------------------------------
    const FixData& GetFixData()
    {
        return _fixData;
    }
}
