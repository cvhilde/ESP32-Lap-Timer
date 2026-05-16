///===========================================================================
///
/// gps.cpp
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
// Private namespace
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
    portMUX_TYPE _fixDataMux = portMUX_INITIALIZER_UNLOCKED;

    // Physical gps data pins connected to the ESP32.
    constexpr uint8_t GPS_Rx = 20;
    constexpr uint8_t GPS_Tx = 19;

    // Time to continuously try to initialize the GPS
    constexpr unsigned long MAX_GPS_INIT = 5000U;

    // Initial setup flag.
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

        const unsigned long start = millis();
        while (millis() - start < MAX_GPS_INIT)
        {
            if (_gps.begin(_GPSHardwareSerial))
            {
                // Make sure to set the update frequency to 1Hz when the GPS
                // is powered on.
                delay(1000);
                _gps.setNavigationFrequency(1U);

                initialized = true;
                return true;
            }

            delay(250);
        }

        initialized = false;
        return false;
    }

    //------------------------------------------------------------------------
    void UpdateUBLOX()
    {
        FixData nextFixData;

        _gps.checkUblox();

        FixType fix = static_cast<FixType>(_gps.getFixType());
        bool gnssFixOk = _gps.getGnssFixOk();
        bool invalidLlh = _gps.getInvalidLlh();
        uint32_t speedAccEstMmps = _gps.getSpeedAccEst();

        if (    fix >= FixType::DEAD_RECKONING
             && fix <= FixType::GNSS_DEAD_RECKONING
             && gnssFixOk
             && !invalidLlh)
        {
            nextFixData.coord.lat = _gps.getLatitude()    * LAT_LONG_TO_DEGREES;
            nextFixData.coord.lng = _gps.getLongitude()   * LAT_LONG_TO_DEGREES;
            nextFixData.speed     = max(0, _gps.getGroundSpeed()) * MM_S_TO_MPH;
            nextFixData.altitude  = _gps.getAltitudeMSL() * MM_TO_FEET;
            nextFixData.speedAccEstMmps = speedAccEstMmps;
            nextFixData.speedAccValid = speedAccEstMmps > 0U;
            
            nextFixData.dateTime.year   = _gps.getYear();
            nextFixData.dateTime.month  = _gps.getMonth();
            nextFixData.dateTime.day    = _gps.getDay();
            nextFixData.dateTime.hour   = ((_gps.getHour() - 4) + 24) % 24;
            nextFixData.dateTime.minute = _gps.getMinute();
            nextFixData.dateTime.second = _gps.getSecond();

            nextFixData.dateTime.valid = true;
            nextFixData.valid          = true;
        }
        else if (fix == FixType::TIME_ONLY)
        {
            nextFixData.dateTime.year   = _gps.getYear();
            nextFixData.dateTime.month  = _gps.getMonth();
            nextFixData.dateTime.day    = _gps.getDay();
            nextFixData.dateTime.hour   = ((_gps.getHour() - 4) + 24) % 24;
            nextFixData.dateTime.minute = _gps.getMinute();
            nextFixData.dateTime.second = _gps.getSecond();

            nextFixData.dateTime.valid = true;
        }

        nextFixData.fixType        = fix;
        nextFixData.satelliteCount = _gps.getSIV();
        nextFixData.updateTimeMs   = millis();

        portENTER_CRITICAL(&_fixDataMux);
        _fixData = nextFixData;
        portEXIT_CRITICAL(&_fixDataMux);
    }

    //------------------------------------------------------------------------
    FixData GetFixData()
    {
        FixData snapshot;

        portENTER_CRITICAL(&_fixDataMux);
        snapshot = _fixData;
        portEXIT_CRITICAL(&_fixDataMux);

        return snapshot;
    }

    //------------------------------------------------------------------------
    void SetUpdateFrequency(unsigned hz)
    {
        hz = constrain(hz, 1U, 25U);
        _gps.setNavigationFrequency(hz);
    }
}
